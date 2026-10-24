package subsystem.rme
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLBundleA
import freechips.rocketchip.regmapper._
import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy.BufferParams.flow





case class ControlUnitRequestorPort(tlParams : TLBundleParameters) extends Bundle
{
    val ID = Output(UInt(tlParams.sourceBits.W))
}

case class ControlUnitTrapperPort(tlParams : TLBundleParameters, inMaxID: Int) extends Bundle 
{
    val baseReqSource = Output(UInt(log2Ceil(inMaxID).W))
    val cacheLine = Output(UInt(512.W))
    
}


// we only need the cached manager edge



/*
    Let's parameterize the width of this as well.

    We will have a single input into the control unit. We will output the active config #s along with the IDs

    We maybe can feed back the active config numbers to the Requestors such that they only output valid if that config is active 


    Let us actually reduce the size of the receiving fetch unit buffers. Lets place some asserts on them as well to reduce the size
*/

class ControlUnitRME(params: RelMemParams, tlOutEdge: TLEdgeOut, tlCachedEdge: TLEdgeIn, instance: Int) (
    implicit p: Parameters) extends Module {

    val tlParams = tlCachedEdge.bundle
    val tlOutParams = tlOutEdge.bundle
    val outMaxID = (math.pow(2, tlOutParams.sourceBits)-1).toInt
    val inMaxID = (math.pow(2, tlParams.sourceBits)-1).toInt
    val beatWidth = 8
    val dataRegWidth = (math.pow(2, params.maxDataSize+1)).toInt * beatWidth // this should give us the extra byte we need to extract excesses
    val io = IO(new Bundle{
        // Config Port 
        //val Config = Input(RMEConfigPortIO())


        // Fetch Unit Port
        val FetchUnitPort = Flipped(DecoupledIO(FetchUnitControlPort(tlParams, inMaxID, outMaxID, dataRegWidth)))
        val PackerAdmission = Flipped(Vec(2, new PackerAdmissionQuery(inMaxID)))

        //val ticket = Output(UInt(16.W)) // help enforce ordering semantics

        // Trapper Port
        val TrapperPort = DecoupledIO(ControlUnitTrapperPort(tlParams, inMaxID))



        // Requestor Port
        //val RequestorPort = Valid(ControlUnitRequestorPort(tlParams))

    }).suggestName(s"ctrlrio_$instance")
    
    /*
        We need to orchestrate the following:

        1. When a packed line can be sent back to trapper
        2. When a packed line is being written back, to write it to SPM and then back to memory
        3. When a new request comes in, we need to update its metadata entry in the Metadata SPM
        4. When a reply comes from the fetch unit, we need to write it to the Data SPM
        5. When we have all data needed to construct a packed line, we notify the packer
        6. Track where to write and read data from
    
    */

        // Number of modules
        val nPackers = 8

        // Instantiate modules (distinct hardware instances)
        val packers = Seq.fill(nPackers) {
            Module(new PackerRME(params, inMaxID, outMaxID))
        }

        val colExtractors = Seq.fill(nPackers) {
            Module(new ColumnExtractor(params, inMaxID, outMaxID))
        }

        // If you want Vecs of their IOs (for indexing / bulk wiring)
        val packerIOs = VecInit(packers.map(_.io))
        val colExtractorIOs = VecInit(colExtractors.map(_.io))
        val currentlyPacking = RegInit(VecInit(Seq.fill(nPackers)(false.B)))
        val BaseReqSrc = RegInit(VecInit(Seq.fill(nPackers)(0.U(log2Ceil(inMaxID).W))))

        val baseIDFromDesc = io.FetchUnitPort.bits.reqTableEntry.descriptor.baseID


        val matchVec = VecInit((0 until nPackers).map { i =>
            (BaseReqSrc(i) === baseIDFromDesc) && currentlyPacking(i)
        })

        val freeVec = VecInit((0 until nPackers).map { i => 
            !currentlyPacking(i)
        })

        val hasMatch = matchVec.reduce(_||_)
        val matchEntry = PriorityEncoder(matchVec)
        val freeEntry = PriorityEncoder(freeVec)
        

        for (i <- 0 until nPackers) {
            colExtractorIOs(i).CtrlUnit.valid := false.B
            colExtractorIOs(i).CtrlUnit.bits  := 0.U.asTypeOf(new CtrlUnitColExtractorIO(inMaxID, outMaxID))
            packerIOs(i).ColExtractor <> colExtractorIOs(i).Packer
        }
    
        def canAcceptBaseID(baseID: UInt): Bool = {
            val matches = VecInit((0 until nPackers).map { i =>
                currentlyPacking(i) && BaseReqSrc(i) === baseID
            })
            Mux(matches.asUInt.orR,
                colExtractorIOs(PriorityEncoder(matches)).CtrlUnit.ready,
                freeVec.asUInt.orR)
        }

        io.PackerAdmission.foreach { query =>
            query.ready := canAcceptBaseID(query.baseID)
        }
        io.FetchUnitPort.ready := canAcceptBaseID(baseIDFromDesc)


        when (io.FetchUnitPort.fire)
        {
//             SynthesizePrintf(
//                 "[ControlUnit] FetchUnitPort FIRE inputValidMask=0x%x\n",
//                 io.FetchUnitPort.bits.reqTableEntry.extractionDescriptorsValid.asUInt
//             )
        }
        




        when (hasMatch)
        {
            when (io.FetchUnitPort.fire) {
             //   SynthesizePrintf("[ControlUnit] in.fire! total desc 0x%x baseID %d ---> to %d\n", io.FetchUnitPort.bits.reqTableEntry.activeDesc, io.FetchUnitPort.bits.reqTableEntry.descriptor.baseID, matchEntry)
            }
            colExtractorIOs(matchEntry).CtrlUnit.valid := io.FetchUnitPort.fire
            colExtractorIOs(matchEntry).CtrlUnit.bits.data := io.FetchUnitPort.bits.data
            colExtractorIOs(matchEntry).CtrlUnit.bits.position := io.FetchUnitPort.bits.reqTableEntry.descriptor.requestPlacement
            colExtractorIOs(matchEntry).CtrlUnit.bits.extractionDescriptors := io.FetchUnitPort.bits.reqTableEntry.extractionDescriptors
            colExtractorIOs(matchEntry).CtrlUnit.bits.descriptorIn := io.FetchUnitPort.bits.reqTableEntry.descriptor
             colExtractorIOs(matchEntry).CtrlUnit.bits.extractionDescriptorsValid := io.FetchUnitPort.bits.reqTableEntry.extractionDescriptorsValid
            //colExtractorIOs(matchEntry).CtrlUnit.bits.nDesc := io.FetchUnitPort.bits.reqTableEntry.activeDesc
        }
        .otherwise {
            when (io.FetchUnitPort.fire) {
              //  SynthesizePrintf("[ControlUnit] in.fire! total desc 0x%x baseID %d ---> to %d\n", io.FetchUnitPort.bits.reqTableEntry.activeDesc, io.FetchUnitPort.bits.reqTableEntry.descriptor.baseID, freeEntry)
                currentlyPacking(freeEntry) := true.B
                BaseReqSrc(freeEntry) := baseIDFromDesc
            }
            colExtractorIOs(freeEntry).CtrlUnit.valid := io.FetchUnitPort.fire
            colExtractorIOs(freeEntry).CtrlUnit.bits.data := io.FetchUnitPort.bits.data
            colExtractorIOs(freeEntry).CtrlUnit.bits.position := io.FetchUnitPort.bits.reqTableEntry.descriptor.requestPlacement
            colExtractorIOs(freeEntry).CtrlUnit.bits.extractionDescriptors := io.FetchUnitPort.bits.reqTableEntry.extractionDescriptors
            colExtractorIOs(freeEntry).CtrlUnit.bits.descriptorIn := io.FetchUnitPort.bits.reqTableEntry.descriptor
            //colExtractorIOs(freeEntry).CtrlUnit.bits.nDesc := io.FetchUnitPort.bits.reqTableEntry.activeDesc
            colExtractorIOs(freeEntry).CtrlUnit.bits.extractionDescriptorsValid := io.FetchUnitPort.bits.reqTableEntry.extractionDescriptorsValid

        }


        when (io.FetchUnitPort.fire) {
            assert(hasMatch || freeVec.reduce(_||_))
        }


        // Observation-only ownership/progress records. Preserve the user's packer count.
        val diagnosticCycle = RegInit(0.U(48.W)); diagnosticCycle := diagnosticCycle + 1.U
        val packerConfig = RegInit(VecInit(Seq.fill(nPackers)(0.U(log2Ceil(params.maxConfigs).W))))
        val packerAllocatedAt = RegInit(VecInit(Seq.fill(nPackers)(0.U(48.W))))
        val packerLastInputAt = RegInit(VecInit(Seq.fill(nPackers)(0.U(48.W))))
        val packerLastPackedAt = RegInit(VecInit(Seq.fill(nPackers)(0.U(48.W))))
        val packerDeliveredMask = RegInit(VecInit(Seq.fill(nPackers)(0.U(16.W))))
        val packerPackedMask = RegInit(VecInit(Seq.fill(nPackers)(0.U(16.W))))
        val packerDuplicateMask = RegInit(VecInit(Seq.fill(nPackers)(0.U(16.W))))
        val packerObservedBytes = RegInit(VecInit(Seq.fill(nPackers)(0.U(7.W))))
        when (io.FetchUnitPort.fire) {
            val index = Mux(hasMatch, matchEntry, freeEntry)
            val mask = io.FetchUnitPort.bits.reqTableEntry.extractionDescriptorsValid.asUInt
            packerDeliveredMask(index) := Mux(hasMatch, packerDeliveredMask(index) | mask, mask)
            packerLastInputAt(index) := diagnosticCycle
            when (!hasMatch) {
                packerConfig(index) := io.FetchUnitPort.bits.reqTableEntry.descriptor.config
                packerAllocatedAt(index) := diagnosticCycle
                packerLastPackedAt(index) := 0.U
            }
        }
        for (index <- 0 until nPackers) {
            when (packerIOs(index).ColExtractor.fire) {
                val mask = packerIOs(index).ColExtractor.bits.dataInValid.asUInt
                packerDuplicateMask(index) := packerDuplicateMask(index) | (packerPackedMask(index) & mask)
                packerPackedMask(index) := packerPackedMask(index) | mask
                packerObservedBytes(index) := packerObservedBytes(index) +
                    (PopCount(mask) << packerIOs(index).ColExtractor.bits.dataSize)
                packerLastPackedAt(index) := diagnosticCycle
            }
            when (packerIOs(index).Trapper.fire) {
                packerPackedMask(index) := 0.U
                packerDuplicateMask(index) := 0.U
                packerObservedBytes(index) := 0.U
            }
        }

        // Print once after approximately 2^10 cycles without progress while a packer is owned.
        val packerStallCycles = RegInit(0.U(10.W))
        val packerWork = currentlyPacking.asUInt.orR
        when (!packerWork || io.FetchUnitPort.fire || io.TrapperPort.fire) {
            packerStallCycles := 0.U
        }.elsewhen (!packerStallCycles.andR) {
            packerStallCycles := packerStallCycles + 1.U
        }
        val packerStallSnapshot = packerWork && !io.FetchUnitPort.fire && !io.TrapperPort.fire &&
              packerStallCycles === ((1 << 10) - 2).U
        when (packerStallSnapshot) {
            SynthesizePrintf(
                "[DTU STALL packers] used=0x%x bases=0x%x extractorReady=0x%x complete=0x%x trapReady=%d inValid=%d inBase=%d count=%d t=%d\n",
                currentlyPacking.asUInt, BaseReqSrc.asUInt,
                VecInit(colExtractorIOs.map(_.CtrlUnit.ready)).asUInt,
                VecInit(packerIOs.map(_.Trapper.valid)).asUInt,
                io.TrapperPort.ready, io.FetchUnitPort.valid, baseIDFromDesc, nPackers.U, diagnosticCycle)
        }

        val packerDumpActive = RegInit(false.B)
        val packerDumpIndex = RegInit(0.U(log2Ceil(nPackers).W))
        when (!packerWork || io.FetchUnitPort.fire || io.TrapperPort.fire) {
            packerDumpActive := false.B
        }.elsewhen (packerStallSnapshot) {
            packerDumpActive := true.B; packerDumpIndex := 0.U
        }.elsewhen (packerDumpActive) {
            when (packerDumpIndex === (nPackers - 1).U) { packerDumpActive := false.B
            }.otherwise { packerDumpIndex := packerDumpIndex + 1.U }
        }
        val ownedByConfig = PopCount((0 until nPackers).map { j =>
            currentlyPacking(j) && packerConfig(j) === packerConfig(packerDumpIndex)
        })
        when (packerDumpActive && !io.FetchUnitPort.fire && !io.TrapperPort.fire && currentlyPacking(packerDumpIndex)) {
            SynthesizePrintf(
                "[DTU STALL packer] slot=%d base=%d cfg=%d cfgOwned=%d allocT=%d inputT=%d packedT=%d delivered=0x%x packed=0x%x duplicates=0x%x bytes=%d extractorVR=0x%x complete=%d\n",
                packerDumpIndex, BaseReqSrc(packerDumpIndex), packerConfig(packerDumpIndex), ownedByConfig,
                packerAllocatedAt(packerDumpIndex), packerLastInputAt(packerDumpIndex), packerLastPackedAt(packerDumpIndex),
                packerDeliveredMask(packerDumpIndex), packerPackedMask(packerDumpIndex), packerDuplicateMask(packerDumpIndex),
                packerObservedBytes(packerDumpIndex),
                Cat(packerIOs(packerDumpIndex).ColExtractor.valid, packerIOs(packerDumpIndex).ColExtractor.ready),
                packerIOs(packerDumpIndex).Trapper.valid)
        }

        val packerOutVec = VecInit(packerIOs.map(_.Trapper))
        val to_trapper_arb = Module(new RRArbiter(new PackerTrapperIO(inMaxID), nPackers))
        to_trapper_arb.io.in <> packerOutVec
        io.TrapperPort.valid := to_trapper_arb.io.out.valid
        io.TrapperPort.bits.baseReqSource := to_trapper_arb.io.out.bits.BaseReqSrc
        io.TrapperPort.bits.cacheLine := to_trapper_arb.io.out.bits.PackedLine
        to_trapper_arb.io.out.ready := io.TrapperPort.ready
        val chosenIdxOut = to_trapper_arb.io.chosen

        when (io.TrapperPort.fire) {
            currentlyPacking(chosenIdxOut) := false.B
        }

}