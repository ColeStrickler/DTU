package subsystem.rme
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLBundleA
import freechips.rocketchip.regmapper._
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

class ControlUnitRME(params: RelMemParams, tlOutEdge: TLEdgeOut, tlCachedEdge: TLEdgeIn, instance: Int)(
    implicit p: Parameters) extends Module {
    val tlParams = tlCachedEdge.bundle
    val outMaxID = (math.pow(2, tlOutEdge.bundle.sourceBits)-1).toInt
    val inMaxID = (math.pow(2, tlParams.sourceBits)-1).toInt
    val dataRegWidth = (1 << (params.maxDataSize + 1)) * 8
    val io = IO(new Bundle {
        val FetchUnitPort = Flipped(Decoupled(
            FetchUnitControlPort(tlParams, inMaxID, outMaxID, dataRegWidth, params = params)))
        // Two independent response-buffer queries for every config; queries never allocate.
        val PackerAdmission = Flipped(Vec(params.maxConfigs, Vec(2, new PackerAdmissionQuery(inMaxID))))
        val TrapperPort = Decoupled(ControlUnitTrapperPort(tlParams, inMaxID))
        val PackerRetired = Output(UInt(math.max(1, log2Ceil(params.maxConfigs)).W))
    }).suggestName(s"ctrlrio_$instance")

    val packerBanks = params.packerCounts.map { count =>
        Seq.fill(count)(Module(new PackerRME(params, inMaxID, outMaxID)))
    }
    val extractor = Module(new ColumnExtractor(params, inMaxID, outMaxID))
    val completed = Module(new RRArbiter(new PackerTrapperIO(inMaxID), params.packerCounts.sum))
    val inputConfig = io.FetchUnitPort.bits.reqTableEntry.descriptor.config
    val inputBaseID = io.FetchUnitPort.bits.reqTableEntry.descriptor.baseID
    val inputSlot = WireDefault(0.U(math.max(1, log2Ceil(params.packerCounts.sum)).W))
    val extractionSlot = RegEnable(inputSlot, io.FetchUnitPort.fire)
    io.FetchUnitPort.ready := false.B
    extractor.io.CtrlUnit.valid := false.B
    extractor.io.CtrlUnit.bits.data := io.FetchUnitPort.bits.data
    extractor.io.CtrlUnit.bits.position := io.FetchUnitPort.bits.reqTableEntry.descriptor.requestPlacement
    extractor.io.CtrlUnit.bits.extractionDescriptors := io.FetchUnitPort.bits.reqTableEntry.extractionDescriptors
    extractor.io.CtrlUnit.bits.extractionDescriptorsValid := io.FetchUnitPort.bits.reqTableEntry.extractionDescriptorsValid
    extractor.io.CtrlUnit.bits.descriptorIn := io.FetchUnitPort.bits.reqTableEntry.descriptor
    extractor.io.Packer.ready := false.B

    for (config <- 0 until params.maxConfigs) {
        val packers = packerBanks(config)
        val count = packers.size
        val active = RegInit(VecInit(Seq.fill(count)(false.B)))
        val baseIDs = Reg(Vec(count, UInt(log2Ceil(inMaxID).W)))
        val free = VecInit(active.map(a => !a))
        def matches(baseID: UInt): Vec[Bool] =
            VecInit((0 until count).map(i => active(i) && baseIDs(i) === baseID))
        def canAccept(baseID: UInt): Bool = {
            val hit = matches(baseID)
            val packerReady = VecInit(packers.map(_.io.ColExtractor.ready))
            extractor.io.CtrlUnit.ready && Mux(hit.asUInt.orR,
                packerReady(PriorityEncoder(hit)), free.asUInt.orR)
        }
        io.PackerAdmission(config).foreach(q => q.ready := canAccept(q.baseID))

        val hit = matches(inputBaseID)
        val slot = Mux(hit.asUInt.orR, PriorityEncoder(hit), PriorityEncoder(free))
        val selected = inputConfig === config.U
        val accepted = selected && io.FetchUnitPort.fire
        when (selected) {
            io.FetchUnitPort.ready := canAccept(inputBaseID)
            extractor.io.CtrlUnit.valid := io.FetchUnitPort.valid && canAccept(inputBaseID)
            inputSlot := params.packerCounts.take(config).sum.U +& slot
        }
        when (accepted) {
            assert(hit.asUInt.orR || free.asUInt.orR)
        }
        for ((packer, slotIndex) <- packers.zipWithIndex) {
            val globalSlot = params.packerCounts.take(config).sum + slotIndex
            when (accepted && !hit.asUInt.orR && slot === slotIndex.U) {
                active(slotIndex) := true.B
                baseIDs(slotIndex) := inputBaseID
            }
            packer.io.ColExtractor.bits := extractor.io.Packer.bits
            packer.io.ColExtractor.valid := extractor.io.Packer.valid && extractionSlot === globalSlot.U
            when (extractionSlot === globalSlot.U) {
                extractor.io.Packer.ready := packer.io.ColExtractor.ready
            }
            completed.io.in(globalSlot) <> packer.io.Trapper
            when (packer.io.Trapper.fire) { active(slotIndex) := false.B }
            when (packer.io.ColExtractor.fire) {
                assert(active(slotIndex))
                assert(extractor.io.Packer.bits.descriptorIn.config === config.U)
                assert(baseIDs(slotIndex) === extractor.io.Packer.bits.descriptorIn.baseID)
            }
        }
    }
    when (io.FetchUnitPort.valid) { assert(inputConfig < params.maxConfigs.U) }
    io.TrapperPort.valid := completed.io.out.valid
    io.TrapperPort.bits.baseReqSource := completed.io.out.bits.BaseReqSrc
    io.TrapperPort.bits.cacheLine := completed.io.out.bits.PackedLine
    completed.io.out.ready := io.TrapperPort.ready
    val packerConfigs = VecInit(params.packerCounts.zipWithIndex.flatMap { case (count, config) =>
        Seq.fill(count)(config.U(math.max(1, log2Ceil(params.maxConfigs)).W))
    })
    io.PackerRetired := packerConfigs(completed.io.chosen)
}
