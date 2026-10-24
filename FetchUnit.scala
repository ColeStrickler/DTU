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




case class RequestTableEntry(inMaxID : Int, outMaxID : Int, nExtractDesc : Int) extends Bundle
{
    val descriptor = new RequestDescriptor(inMaxID, outMaxID)
    val extractionDescriptors = Vec(nExtractDesc, new ExtractionDescriptor(4))
    val extractionDescriptorsValid = Vec(nExtractDesc, Bool())
    //val activeDesc = UInt(log2Ceil(nExtractDesc+1).W)
    val active = Bool()
}


case class FetchUnitControlPort(tlParams : TLBundleParameters, inMaxID : Int, outMaxID : Int, dataRegWidth: Int, nExtractDesc: Int = 16) extends Bundle
{
    val data = Output(UInt(512.W)) // 64 bytes = 1 cache line
    val reqTableEntry = Output(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc))
}


// Independent readiness query; it never allocates a packer.
case class PackerAdmissionQuery(inMaxID: Int) extends Bundle {
    val baseID = Output(UInt(log2Ceil(inMaxID).W))
    val ready = Input(Bool())
}

case class FetchUnitIO(tlInParams: TLBundleParameters, tlOutParams: TLBundleParameters, inMaxID : Int, outMaxID : Int, dataRegWidth : Int, toCacheParams: TLBundleParameters, params: RelMemParams) extends Bundle
{

    

// Requestor Port
        val Requestor = Flipped(Decoupled(new RequestorFetchUnitPort(inMaxID, outMaxID))) // Receive address to request from the Requestor Module]
        val Config = Flipped(RMEConfigPortIO(params))
        val Prefetch = Flipped(new FetchUnitPrefetchUnitPort(inMaxID, outMaxID))
        //val FetchReq = Flipped(Decoupled(Output(new TLBundleA(tlInEdge.bundle))))
        //val isBaseRequest = Flipped(Output(Bool()))
        //val Requestor_isBaseRequest = Flipped(Decoupled(Bool()))
        //val Requestor_FetchReq = Flipped(Decoupled(new TLBundleA(tlInEdge.bundle)))


        // LLC Port
        val IncomingReqInCache = Input(Bool())
        val DiagnosticStall = Output(Bool())
        val LLCOutReq = Decoupled(new TLBundleA(toCacheParams)) // send outbound memory requests to DRAM
        val LLCInReply = Flipped(Decoupled(new TLBundleD(toCacheParams))) // receive inbound data from DRAM


        /*
            We will probable want to tag this A channel request on as metadata so we can easily form
            D channel replies to cache
        */

        // DRAM Port
        val OutReq = Decoupled(new TLBundleA(tlOutParams)) // send outbound memory requests to DRAM
        val inReply = Flipped(Decoupled(new TLBundleD(tlOutParams))) // receive inbound data from DRAM
        // Control Unit Port
        val ControlUnit = Decoupled(FetchUnitControlPort(tlInParams, inMaxID, outMaxID, dataRegWidth))
        val PackerAdmission = Vec(2, new PackerAdmissionQuery(inMaxID))
        val PrefetchReady = Input(Vec(params.maxConfigs, Bool()))

}





class FetchUnitRME(params: RelMemParams, adapter: TLAdapterNode, cachedRegionEdge: TLEdgeIn, toCacheNode : TLClientNode, instance: Int, subInstance: Int)(
    implicit p: Parameters) extends Module {

        val (out, tlOutEdge) = adapter.out(0)
        val (in, tlInEdge) = adapter.in(0)

        val (toCacheOutBundle, toCacheOutTLInEdge) = toCacheNode.out(0)
        val toCacheTLParams = toCacheOutTLInEdge.bundle

        val tlOutA = out.a
        val tlOutD = out.d
        val tlOutParams = tlOutEdge.bundle
        val tlInParams = cachedRegionEdge.bundle
        val inMaxID = (math.pow(2, cachedRegionEdge.bundle.sourceBits)-1).toInt
        val outMaxID = (math.pow(2, tlOutParams.sourceBits)-1).toInt
        val beatWidth = 8
        val dataRegWidth = (math.pow(2, params.maxDataSize+1)).toInt * beatWidth // this should give us the extra byte we need to extract excesses
        val srcID = (outMaxID - subInstance).U
        println(s"inMaxID $inMaxID outMaxID $outMaxID using SrCID ${outMaxID - subInstance}")
        println(s"dataRegWidth $dataRegWidth")
        val io = IO(new FetchUnitIO(tlInParams, tlOutParams, inMaxID, outMaxID, dataRegWidth, toCacheTLParams, params)).suggestName(s"fetchunitio_$instance-$subInstance")

        println(s"\n\n\nout data width ${io.OutReq.bits.data.getWidth}\n\n\n")
        io.OutReq.valid := false.B
        io.OutReq.bits := 0.U.asTypeOf(new TLBundleA(tlOutParams))

        io.LLCOutReq.valid := false.B
        io.LLCOutReq.bits := 0.U.asTypeOf(new TLBundleA(toCacheTLParams))

        // Maximum coalescable requests
        val nExtractDesc = 16
        val FREE_REQ_ENTRY = nExtractDesc+1


        // Request Table Init //
        val emptyEntry = Wire(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc))
        emptyEntry.descriptor := 0.U.asTypeOf(RequestDescriptor(inMaxID, outMaxID))
        emptyEntry.extractionDescriptors := VecInit(Seq.fill(nExtractDesc) 
        { // extractionDescriptors init (each element must be initialized)
            val ed = Wire(new ExtractionDescriptor(4))
            ed.start := 0.U
            //ed.size  := 0.U
            ed.pos   := 0.U
            ed
        })
        emptyEntry.active := false.B
        emptyEntry.extractionDescriptorsValid := VecInit(Seq.fill(nExtractDesc)(false.B))
        //emptyEntry.activeDesc := 0.U
                // Request Table Init //




        /*
            DATA REGISTERS
        */
        val dataReg = RegInit(0.U(512.W)) // store a single cache line we get from DRAM
        val dataReg2 = RegInit(0.U(512.W)) // store a single cache line we get from DRAM
        val receivingEntry = RegInit(0.U(log2Ceil(params.nFetchUnits).W))
        val receivingEntry2 = RegInit(0.U(log2Ceil(params.nFetchUnits).W))
        val dataRegFull = RegInit(false.B)
        val dataRegActive = RegInit(false.B)
        val dataReg2Full = RegInit(false.B)
        val dataReg2Active = RegInit(false.B)
        /*
            DATA REGISTERS
        */


        /*
            Store information on outbound requests here
        */
        val requestTable = RegInit(VecInit(Seq.fill(params.nFetchUnits)(emptyEntry)))
        // Diagnostic bookkeeping only; these bits never control request admission or routing.
        val requestIssuedDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(false.B)))
        val requestLLCDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(false.B)))
        val diagnosticCycle = RegInit(0.U(48.W)); diagnosticCycle := diagnosticCycle + 1.U
        val nextIssueOrder = RegInit(1.U(32.W))
        val requestIssueOrderDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(0.U(32.W))))
        val requestIssuedAtDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(0.U(48.W))))
        val requestRxStartedDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(false.B)))
        val requestRxDoneDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(false.B)))
        val requestRxFirstAtDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(0.U(48.W))))
        val requestRxLastAtDiagnostic = RegInit(VecInit(Seq.fill(params.nFetchUnits)(0.U(48.W))))
        // HELPER FUNCTIONS 


        def ResetEntry(entry: UInt) : Unit = {
            requestTable(entry) := emptyEntry
            requestIssuedDiagnostic(entry) := false.B
            requestLLCDiagnostic(entry) := false.B
        }


        def GetCoalesceEntry(coalesceVec: Vec[Bool]) : UInt = {
            PriorityEncoder(coalesceVec)
        }

        def GetCoalesceVec(incomingDesc: RequestDescriptor) : Vec[Bool] = {
            VecInit(requestTable.zipWithIndex.map{ case (entry,i) =>
                entry.active && (entry.descriptor.addr === incomingDesc.addr) && (incomingDesc.dst.asUInt === 0.U) && (entry.descriptor.dst.asUInt === 0.U) && (entry.descriptor.baseID === incomingDesc.baseID) && !(receivingEntry === i.U && dataRegFull) && !(receivingEntry2 === i.U && dataReg2Full)
            })
        }

        def CanCoalesce(coalesceVec: Vec[Bool]): Bool = {
            VecInit(coalesceVec.zipWithIndex.map { case (b, i) =>
                b && !(receivingEntry === i.U && dataRegFull) && !(receivingEntry2 === i.U && dataReg2Full)
            }).reduce(_ || _)
        }       

        val availableEntryVec = VecInit(requestTable.map{ en =>
            !en.active
        })


        def HasAvailableEntries() : Bool = {
            availableEntryVec.reduce(_ || _)
        }

        def AvailableEntries() : Vec[Bool] = { // only use once to avoid duplicating comparator logic
            availableEntryVec
        }

        def FirstAvailableEntry() : UInt = {
            PriorityEncoder(AvailableEntries())
        }


        def AllocateEntry(entry: UInt, reqPort: RequestorFetchUnitPort) : Unit = {
            val reqTableEntry = Wire(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc))
            reqTableEntry := 0.U.asTypeOf(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc))
            reqTableEntry.descriptor := reqPort.descriptor
            val pos = reqPort.extractionDescriptor.pos
            reqTableEntry.extractionDescriptors(pos) := reqPort.extractionDescriptor
            reqTableEntry.extractionDescriptorsValid(pos) := true.B
            reqTableEntry.active := true.B
            requestTable(entry) := reqTableEntry
            requestIssuedDiagnostic(entry) := false.B
            requestLLCDiagnostic(entry) := false.B
            requestIssueOrderDiagnostic(entry) := 0.U
            requestIssuedAtDiagnostic(entry) := 0.U
            requestRxStartedDiagnostic(entry) := false.B
            requestRxDoneDiagnostic(entry) := false.B
            requestRxFirstAtDiagnostic(entry) := 0.U
            requestRxLastAtDiagnostic(entry) := 0.U
        }


        def CoalesceEntry(entry: UInt, extractionDescript: ExtractionDescriptor) : Unit = {
            //val active_desc = requestTable(entry).activeDesc
            val pos = extractionDescript.pos
            //requestTable(entry).activeDesc := active_desc + 1.U
            //SynthesizePrintf("(FetchUnit) pos incoming %d\n", pos)
            requestTable(entry).extractionDescriptors(pos) := extractionDescript
            assert(!requestTable(entry).extractionDescriptorsValid(pos))
            requestTable(entry).extractionDescriptorsValid(pos) := true.B
        }


        def DescriptorToOutReq(desc: RequestDescriptor, src: UInt): TLBundleA = {
            val (legal, ret) = tlOutEdge.Get(          // use the edge you already have!
                fromSource = src,
                toAddress  = Mux(desc.addr < (params.rmeaddress + params.rmeAddressSize).U, desc.addr, params.rmeaddress.U),
                lgSize     = 6.U
            )
            // legal should be true — add assert(legal) in synthesis if you want

            ret  // the helper already sets opcode, param, size, address, mask, data=0, corrupt=false, etc. correctly
        }


        def DescriptorToCacheReq(desc: RequestDescriptor, src: UInt, toDTUCond: Bool): TLBundleA = {
            val (legal, ret) = toCacheOutTLInEdge.Get(          // use the edge you already have!
                fromSource = src, // may need to do something different with the sources here....
                toAddress  = Mux(desc.addr < (params.rmeaddress + params.rmeAddressSize).U || toDTUCond, desc.addr, params.rmeaddress.U),
                lgSize     = 6.U
            )
            // legal should be true — add assert(legal) in synthesis if you want
              when (io.LLCOutReq.valid) {
                assert(legal)
            }
            ret  // the helper already sets opcode, param, size, address, mask, data=0, corrupt=false, etc. correctly
        }


        def AddrIsValidConfig(addr: UInt, config: UInt): Bool = {
            val inEphemeralRegion =
                io.Config.EphemeralRegionConfig_PhysStart
                    .zip(io.Config.EphemeralRegionConfig_Size)
                    .zipWithIndex
                    .map { case ((start, size), i) =>
                        // i is the Scala index here
                        (addr >= start) && (addr < (start +& size)) && (i.U =/= config)
                    }
                    .reduce(_ || _)

            inEphemeralRegion
        }


        println(s"(FetchUnit) MASK ${new TLBundleA(tlOutParams).mask.getWidth}")
        println(s"(FetchUnit) DATA ${new TLBundleA(tlOutParams).data.getWidth}")

        /*
            When we have an incoming request we have 3 options:
            1.  Zero flag is set, we can immediately forward the value itself to the ControlUnit.
                Still must allocate entry in RequestTable for easy arbitration.

            2. Cannot coalesce entry. So allocate new entry in request table.

            3. Can coalesce entry, so we add an extraction descriptor to the already existing entry
               We need to ensure that if this happens on same cycle as that data is being removed, it is handled properly
        */


        val hasOutReqToSend = RegInit(false.B)
        val outReqEntryToSend = RegInit(0.U(log2Ceil(params.nFetchUnits).W))
        val hasOutReqToCache = RegInit(false.B)
        val alloc_entry = FirstAvailableEntry()
        val cvec = GetCoalesceVec(io.Requestor.bits.descriptor)
        val can_coalesce = CanCoalesce(cvec)
        val coalesce_entry = GetCoalesceEntry(cvec)
        hasOutReqToSend := false.B
        outReqEntryToSend := outReqEntryToSend
        io.OutReq.valid := false.B
        hasOutReqToCache := false.B


        val firstWaitCycle = RegInit(false.B)
        assert((math.pow(2, toCacheTLParams.sourceBits)) >= params.nFetchUnits) // otherwise we would use source IDs larger than we can accomodate
        when (io.Requestor.fire)
        {
            //SynthesizePrintf("incoming 0x%x -- dst %d coalesce %d  baseID %d\n", io.Requestor.bits.descriptor.addr, io.Requestor.bits.descriptor.dst.asUInt, can_coalesce, io.Requestor.bits.descriptor.baseID)
            when (!can_coalesce)
            {
                //SynthesizePrintf("No Coalesce 0x%x, Allocate entry %d DescState %d BaseID %d\n", io.Requestor.bits.descriptor.addr, alloc_entry, requestTable(alloc_entry).active, io.Requestor.bits.descriptor.baseID)
                AllocateEntry(alloc_entry, io.Requestor.bits)
                hasOutReqToSend := true.B //!io.LLCOutReq.fire && !io.OutReq.fire
                //hasOutReqToCache := false.B //io.IncomingReqInCache && !io.LLCOutReq.fire
                io.OutReq.valid := false.B //!io.IncomingReqInCache
                outReqEntryToSend := alloc_entry
                val src = outMaxID.U-alloc_entry
                //io.OutReq.bits := DescriptorToOutReq(io.Requestor.bits.descriptor, src)
                firstWaitCycle := true.B

                //io.LLCOutReq.bits := DescriptorToCacheReq(io.Requestor.bits.descriptor, alloc_entry)
                io.LLCOutReq.valid :=false.B // io.IncomingReqInCache

                when (io.OutReq.fire) {
                 //   SynthesizePrintf("[FetchUnit] io.OutReq.fire baseReq %d\n", io.Requestor.bits.descriptor.baseID)
                }

            }
            .otherwise {
                //SynthesizePrintf("(FetchUnit) incoming source %d\n", io.Requestor.bits.descriptor.dst.asUInt)
                //SynthesizePrintf("Coalesce Entry! 0x%x BaseID %d Entry %d\n", io.Requestor.bits.descriptor.addr, io.Requestor.bits.descriptor.baseID, coalesce_entry)
                CoalesceEntry(coalesce_entry, io.Requestor.bits.extractionDescriptor)
            }
            
            /* 
                We try to just pass the request throught,
                but if we can't we will try again the next cycle

                For now, when hasOutReqToSend is true, we will not take in another request
                ---> this will simplify things greatly
            */
        }


        when (hasOutReqToSend)
        {
          //  SynthesizePrintf("HasOutReqEntryToSend %d\n", outReqEntryToSend)
            val outReq = Wire(new TLBundleA(tlOutParams))
            val outReqToCache = Wire(new TLBundleA(toCacheTLParams))

            val src = outMaxID.U-outReqEntryToSend
            val maxDRAM = math.pow(2, 33).toLong
            val descriptorAddr = requestTable(outReqEntryToSend).descriptor.addr
            val descriptorConfig = requestTable(outReqEntryToSend).descriptor.config
            val isDTUAddr = descriptorAddr >= maxDRAM.U
            outReq := DescriptorToOutReq(requestTable(outReqEntryToSend).descriptor, src)
            val toDTUCond = isDTUAddr && AddrIsValidConfig(descriptorAddr, descriptorConfig)
            outReqToCache := DescriptorToCacheReq(requestTable(outReqEntryToSend).descriptor, outReqEntryToSend, toDTUCond)

            val ReqIsToCache = Mux(firstWaitCycle, io.IncomingReqInCache || toDTUCond, hasOutReqToCache) 
            hasOutReqToCache := Mux(firstWaitCycle, Mux((io.OutReq.fire || io.LLCOutReq.fire), false.B, ReqIsToCache), ReqIsToCache && (!io.OutReq.fire && !io.LLCOutReq.fire))
            firstWaitCycle := false.B

            io.OutReq.valid := hasOutReqToSend && !ReqIsToCache
            io.OutReq.bits := outReq

            io.LLCOutReq.valid := hasOutReqToSend && ReqIsToCache
            io.LLCOutReq.bits := outReqToCache


            when (hasOutReqToSend && io.OutReq.fire) {
            //    SynthesizePrintf("[FetchUnit]: outReqEntry %d hasOutReqToSend.fire baseReq %d\n", outReqEntryToSend, requestTable(outReqEntryToSend).descriptor.baseID)
            }

            val (a_first, a_last, a_done) = tlOutEdge.firstlast(io.OutReq)
            hasOutReqToSend := !io.OutReq.fire && !io.LLCOutReq.fire
        }
        /*
            Basically we need to figure out if we can reuse the same src scheme for requests into the cache.

            Once we know this, we can figure out if we need to handle them together or separately.
        
        
        */    

    
        io.Requestor.ready := (HasAvailableEntries() && !hasOutReqToSend) || can_coalesce 

        
        when (io.OutReq.fire) {
//         SynthesizePrintf(
//             "[DTU-TRACE fetch-dram] addr=0x%x size=%d source=%d\n",
//             io.OutReq.bits.address,
//             io.OutReq.bits.size,
//             io.OutReq.bits.source)
             // SynthesizePrintf(
 //   "[DTU-A] addr=0x%x size=%d mask=0x%x beatFirst=%d beatLast=%d\n",
 //   io.OutReq.bits.address,
 //   io.OutReq.bits.size,
 //   io.OutReq.bits.mask,
 //   tlOutEdge.firstlast(io.OutReq)._1,
 //   tlOutEdge.firstlast(io.OutReq)._2
 // )
        }

        // Show the exact request which reaches the cache-facing TileLink edge.
        // Print a legal request only when it handshakes, and print an illegal
        // request once even if it remains valid while the receiver is blocked.
        val (llcOutLegal, _) = toCacheOutTLInEdge.Get(
            fromSource = io.LLCOutReq.bits.source,
            toAddress = io.LLCOutReq.bits.address,
            lgSize = io.LLCOutReq.bits.size)
        val illegalLLCPrinted = RegInit(false.B)
        when (!io.LLCOutReq.valid || llcOutLegal) {
            illegalLLCPrinted := false.B
        }
        when (io.LLCOutReq.fire) {
//             SynthesizePrintf(
//                 "[DTU-TRACE fetch-llc] addr=0x%x size=%d source=%d legal=%d\n",
//                 io.LLCOutReq.bits.address,
//                 io.LLCOutReq.bits.size,
//                 io.LLCOutReq.bits.source,
//                 llcOutLegal)
        }
        when (io.LLCOutReq.valid && !llcOutLegal && !illegalLLCPrinted) {
            SynthesizePrintf(
                "[DTU-TRACE ILLEGAL fetch-llc] addr=0x%x size=%d source=%d ready=%d\n",
                io.LLCOutReq.bits.address,
                io.LLCOutReq.bits.size,
                io.LLCOutReq.bits.source,
                io.LLCOutReq.ready)
            illegalLLCPrinted := true.B
        }

        val (d_first, d_last, d_done, _, d_count) = tlOutEdge.firstlast2(io.inReply)
        val (d_first_cache, d_last_cache, d_done_cache, _, d_count_cache) = tlOutEdge.firstlast2(io.LLCInReply)

        io.inReply.ready := false.B
       // io.LLCInReply.ready := false.B

        // we give preference to the cache
        val DRAMInReplyActive  = RegInit(false.B)
        val LLCInReplyActive = RegInit(false.B)

        io.inReply.ready := !dataRegFull  // can not receive more replies until we have done something with current data
        io.LLCInReply.ready := !dataReg2Full 

        when (io.inReply.fire) {
           // SynthesizePrintf("[FetchUnit]: inReply.firesrc %d (%d,%d,%d) dataRegfull %d\n", io.inReply.bits.source, d_first_cache, d_last_cache, d_done_cache, dataReg2Full)
        }
        when (d_first && io.inReply.fire) {
            receivingEntry  := outMaxID.U - io.inReply.bits.source
        }

        when (d_first_cache && io.LLCInReply.fire) {
            receivingEntry2  := io.LLCInReply.bits.source
        }


        // shift in new data
        val dataWidth = io.inReply.bits.data.getWidth
        val shiftNewData = io.inReply.bits.data //+ d_count // count is to test

        val dataWidthCache = io.LLCInReply.bits.data.getWidth
        val shiftNewDataCache = io.LLCInReply.bits.data
        val dramEntry = requestTable(receivingEntry)
        val llcEntry = requestTable(receivingEntry2)
        io.PackerAdmission(0).baseID := dramEntry.descriptor.baseID
        io.PackerAdmission(1).baseID := llcEntry.descriptor.baseID

        def canAdvance(entry: RequestTableEntry, packerReady: Bool): Bool = {
            val prefetchReady = VecInit((0 until params.maxConfigs).map { i =>
                io.PrefetchReady(i) && entry.descriptor.requestPlacement === i.U
            }).asUInt.orR
            Mux(entry.descriptor.dst === DESTINATION.CONTROL_UNIT,
                packerReady, prefetchReady)
        }

        val dramCanAdvance = canAdvance(dramEntry, io.PackerAdmission(0).ready)
        val llcCanAdvance = canAdvance(llcEntry, io.PackerAdmission(1).ready)
        val dramEligible = dataRegFull && dramCanAdvance
        val llcEligible = dataReg2Full && llcCanAdvance
        val preferDRAM = RegInit(true.B)
        val selectDRAM = dramEligible && (!llcEligible || preferDRAM)
        val currTableEntry = Mux(selectDRAM, dramEntry, llcEntry)
        val hasFullData = dramEligible || llcEligible
        val fullDataToSend = Mux(selectDRAM, dataReg, dataReg2)
        val outputFire = io.Prefetch.ToPre.fire || io.ControlUnit.fire

        // Retire only the buffer selected by the response arbiter.
        when (outputFire) {
            preferDRAM := !selectDRAM
        }
        dataReg2 := Mux(io.LLCInReply.fire, Cat(shiftNewDataCache, (dataReg2 >> dataWidthCache)((dataReg2.getWidth - 1)-dataWidthCache, 0)), dataReg2)
        dataReg2Full := Mux(
            dataReg2Full,
            !(outputFire && !selectDRAM),
            d_last_cache && io.LLCInReply.fire
        )

        dataReg := Mux(io.inReply.fire, Cat(shiftNewData, (dataReg >> dataWidth)((dataReg.getWidth - 1)-dataWidth, 0)), dataReg)
        dataRegFull := Mux(
            dataRegFull,
            !(outputFire && selectDRAM),
            d_last && io.inReply.fire
        )

        io.ControlUnit.valid := hasFullData && currTableEntry.descriptor.dst === DESTINATION.CONTROL_UNIT
        io.ControlUnit.bits.reqTableEntry := currTableEntry
        io.ControlUnit.bits.data := fullDataToSend

        io.Prefetch.ToPre.valid := hasFullData && currTableEntry.descriptor.dst === DESTINATION.PREFETCH_UNIT
        io.Prefetch.ToPre.bits.config := currTableEntry.descriptor.requestPlacement
        io.Prefetch.ToPre.bits.addr := currTableEntry.descriptor.addr
        io.Prefetch.ToPre.bits.data := fullDataToSend

        when (io.LLCOutReq.fire)
        {
//             SynthesizePrintf("(FetchUnit) TO LLC\n")
        }

        when (io.LLCInReply.fire)
        {
//             SynthesizePrintf("(FetchUnit) FROM LLC 0x%x\n", io.LLCInReply.bits.data)
        }


        when (io.OutReq.fire)
        {
//             SynthesizePrintf("(FetchUnit) TO DRAM\n")
        }

        when (io.inReply.fire)
        {
//             SynthesizePrintf("(FetchUnit) FROM DRAM 0x%x\n", io.LLCInReply.bits.data)
        }



        //when (io.LLCInReply.valid)
        //{
        //    //SynthesizePrintf("(FetchUnit) FROM LLC VALID\n")
        //}


        when (io.OutReq.fire || io.LLCOutReq.fire) {
            requestIssuedDiagnostic(outReqEntryToSend) := true.B
            requestLLCDiagnostic(outReqEntryToSend) := io.LLCOutReq.fire
            requestIssueOrderDiagnostic(outReqEntryToSend) := nextIssueOrder
            nextIssueOrder := nextIssueOrder + 1.U
            requestIssuedAtDiagnostic(outReqEntryToSend) := diagnosticCycle
        }

        // First/last accepted reply beats, independent of later buffer delivery.
        val diagnosticDRAMHeadSlot = (outMaxID.U - io.inReply.bits.source)(log2Ceil(params.nFetchUnits) - 1, 0)
        val diagnosticLLCHeadSlot = io.LLCInReply.bits.source(log2Ceil(params.nFetchUnits) - 1, 0)
        val dramReplySlot = Mux(d_first, diagnosticDRAMHeadSlot, receivingEntry)
        val llcReplySlot = Mux(d_first_cache, diagnosticLLCHeadSlot, receivingEntry2)
        for ((port, first, last, slot) <- Seq(
            (io.inReply, d_first, d_last, dramReplySlot),
            (io.LLCInReply, d_first_cache, d_last_cache, llcReplySlot))) {
            when (port.fire && first) {
                requestRxStartedDiagnostic(slot) := true.B
                requestRxFirstAtDiagnostic(slot) := diagnosticCycle
            }
            when (port.fire && last) {
                requestRxDoneDiagnostic(slot) := true.B
                requestRxLastAtDiagnostic(slot) := diagnosticCycle
            }
        }

        // Keep the diagnostic narrow: IDs/addresses/control bits, never data lines.
        val responseStallCycles = RegInit(0.U(10.W))
        val activeRequests = VecInit(requestTable.map(_.active)).asUInt
        val responseWork = activeRequests.orR || hasOutReqToSend || dataRegFull || dataReg2Full
        when (!responseWork || outputFire) {
            responseStallCycles := 0.U
        }.elsewhen (!responseStallCycles.andR) {
            responseStallCycles := responseStallCycles + 1.U
        }
        val responseStallSnapshot = responseWork && !outputFire &&
            responseStallCycles === ((1 << 10) - 2).U
        io.DiagnosticStall := responseStallSnapshot
        val history = Module(new DTUFetchHistory(params.nFetchUnits,
            math.max(tlInParams.sourceBits, tlOutParams.sourceBits), log2Ceil(params.maxConfigs)))
        history.io.snapshot := responseStallSnapshot
        history.io.progress := !responseWork || outputFire
        def traceEvent(lane: Int, valid: Bool, slot: UInt, llc: Bool,
                       source: UInt, edge: UInt, order: UInt): Unit = {
            val event = history.io.events(lane)
            val desc = requestTable(slot).descriptor
            event.valid := valid
            event.bits.llc := llc; event.bits.slot := slot; event.bits.source := source
            event.bits.order := order; event.bits.config := desc.config
            event.bits.dst := desc.dst.asUInt; event.bits.base := desc.baseID
            event.bits.place := desc.requestPlacement; event.bits.addr := desc.addr
            event.bits.edge := edge
        }
        traceEvent(0, io.OutReq.fire || io.LLCOutReq.fire, outReqEntryToSend,
            io.LLCOutReq.fire, Mux(io.LLCOutReq.fire, io.LLCOutReq.bits.source, io.OutReq.bits.source),
            0.U, nextIssueOrder)
        traceEvent(1, io.inReply.fire && (d_first || d_last), dramReplySlot,
            false.B, io.inReply.bits.source, Cat(d_last, d_first), requestIssueOrderDiagnostic(dramReplySlot))
        traceEvent(2, io.LLCInReply.fire && (d_first_cache || d_last_cache), llcReplySlot,
            true.B, io.LLCInReply.bits.source, Cat(d_last_cache, d_first_cache), requestIssueOrderDiagnostic(llcReplySlot))
        val deliveredSlot = Mux(selectDRAM, receivingEntry, receivingEntry2)
        traceEvent(3, outputFire, deliveredSlot, !selectDRAM,
            Mux(selectDRAM, outMaxID.U - receivingEntry, receivingEntry2),
            0.U, requestIssueOrderDiagnostic(deliveredSlot))
        when (responseStallSnapshot) {
            SynthesizePrintf(
                "[DTU STALL fetch] active=0x%x pending=%d outAddr=0x%x outV=0x%x outR=0x%x full=0x%x can=0x%x dst=0x%x dBase=%d lBase=%d dAddr=0x%x lAddr=0x%x inV=0x%x inR=0x%x dSrc=%d lSrc=%d dRemain=%d lRemain=%d dRx=%d lRx=%d t=%d nextOrder=%d dHeadBase=%d dHeadCfg=%d dHeadOrder=%d\n",
                activeRequests, hasOutReqToSend,
                requestTable(outReqEntryToSend).descriptor.addr,
                Cat(io.LLCOutReq.valid, io.OutReq.valid),
                Cat(io.LLCOutReq.ready, io.OutReq.ready),
                Cat(dataReg2Full, dataRegFull), Cat(llcCanAdvance, dramCanAdvance),
                Cat(llcEntry.descriptor.dst.asUInt, dramEntry.descriptor.dst.asUInt),
                dramEntry.descriptor.baseID, llcEntry.descriptor.baseID,
                dramEntry.descriptor.addr, llcEntry.descriptor.addr,
                Cat(io.LLCInReply.valid, io.inReply.valid),
                Cat(io.LLCInReply.ready, io.inReply.ready),
                io.inReply.bits.source, io.LLCInReply.bits.source,
                d_count, d_count_cache, receivingEntry, receivingEntry2,
                diagnosticCycle, nextIssueOrder,
                requestTable(diagnosticDRAMHeadSlot).descriptor.baseID,
                requestTable(diagnosticDRAMHeadSlot).descriptor.config,
                requestIssueOrderDiagnostic(diagnosticDRAMHeadSlot))
        }

        // One bounded pass, one shared print site, rather than a site per table slot.
        // This is a live scan during the same stall, not an atomic copy of the table.
        val tableDumpActive = RegInit(false.B)
        val tableDumpIndex = RegInit(0.U(log2Ceil(params.nFetchUnits).W))
        when (!responseWork || outputFire) {
            tableDumpActive := false.B
        }.elsewhen (responseStallSnapshot) {
            tableDumpActive := true.B
            tableDumpIndex := 0.U
        }.elsewhen (tableDumpActive) {
            when (tableDumpIndex === (params.nFetchUnits - 1).U) {
                tableDumpActive := false.B
            }.otherwise {
                tableDumpIndex := tableDumpIndex + 1.U
            }
        }
        val dumpEntry = requestTable(tableDumpIndex)
        val dumpLLC = requestLLCDiagnostic(tableDumpIndex)
        val dumpSource = Mux(dumpLLC, tableDumpIndex, outMaxID.U - tableDumpIndex)
        when (tableDumpActive && responseWork && !outputFire && dumpEntry.active) {
            SynthesizePrintf(
                "[DTU STALL entry] slot=%d issued=%d llc=%d src=%d cfg=%d dst=%d base=%d place=%d addr=0x%x extracts=0x%x order=%d issueT=%d rxStarted=%d firstT=%d rxDone=%d lastT=%d\n",
                tableDumpIndex, requestIssuedDiagnostic(tableDumpIndex), dumpLLC,
                dumpSource, dumpEntry.descriptor.config, dumpEntry.descriptor.dst.asUInt,
                dumpEntry.descriptor.baseID, dumpEntry.descriptor.requestPlacement,
                dumpEntry.descriptor.addr, dumpEntry.extractionDescriptorsValid.asUInt,
                requestIssueOrderDiagnostic(tableDumpIndex), requestIssuedAtDiagnostic(tableDumpIndex),
                requestRxStartedDiagnostic(tableDumpIndex), requestRxFirstAtDiagnostic(tableDumpIndex),
                requestRxDoneDiagnostic(tableDumpIndex), requestRxLastAtDiagnostic(tableDumpIndex))
        }

        when (io.ControlUnit.fire || io.Prefetch.ToPre.fire)
        {
            when (selectDRAM)
            {
                ResetEntry(receivingEntry)
            }.otherwise
            {
                ResetEntry(receivingEntry2)
            }

            
            when (io.ControlUnit.fire)
            {
           //     SynthesizePrintf("ToControlUnit: receiving entry %d, BaseReqSrc %d\n", receivingEntry, requestTable(receivingEntry).descriptor.dst.asUInt)
            }
            .otherwise
            {
               // SynthesizePrintf("ToPrefetchUnit: receiving entry %d, BaseReqSrc %d data 0x%x\n", receivingEntry, requestTable(receivingEntry).descriptor.dst.asUInt, dataReg)
            }
        }
}


