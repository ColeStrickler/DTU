package subsystem.rme

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLBundleA
import freechips.rocketchip.regmapper._
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy.BufferParams.flow




case class RequestTableEntry(inMaxID : Int, outMaxID : Int, nExtractDesc : Int, params: RelMemParams = RelMemParams()) extends Bundle
{
    val descriptor = new RequestDescriptor(inMaxID, outMaxID, params)
    val extractionDescriptors = Vec(nExtractDesc, new ExtractionDescriptor(4))
    val extractionDescriptorsValid = Vec(nExtractDesc, Bool())
    //val activeDesc = UInt(log2Ceil(nExtractDesc+1).W)
    val active = Bool()
}


case class FetchUnitControlPort(tlParams : TLBundleParameters, inMaxID : Int, outMaxID : Int, dataRegWidth: Int, nExtractDesc: Int = 16, params: RelMemParams = RelMemParams()) extends Bundle
{
    val data = Output(UInt(512.W)) // 64 bytes = 1 cache line
    val reqTableEntry = Output(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc, params))
}


// Independent readiness query; it never allocates a packer.
case class PackerAdmissionQuery(inMaxID: Int) extends Bundle {
    val baseID = Output(UInt(log2Ceil(inMaxID).W))
    val ready = Input(Bool())
}

// Accepts one probe per cycle on the shared directory port and returns its
// one-cycle result only to the pipeline that accepted the descriptor.
class FetchDirectoryArbiter(params: RelMemParams, inMaxID: Int, outMaxID: Int) extends Module {
    val io = IO(new Bundle {
        val Requests = Flipped(Vec(params.maxConfigs, Decoupled(new RequestorFetchUnitPort(inMaxID, outMaxID, params))))
        val ToFetch = Vec(params.maxConfigs, Decoupled(new RequestorFetchUnitPort(inMaxID, outMaxID, params)))
        val DirectoryRequest = Valid(UInt(48.W))
        val DirectoryResponse = Flipped(Valid(Bool()))
        val CacheHit = Output(Vec(params.maxConfigs, Bool()))
    })
    val arb = Module(new RRArbiter(new RequestorFetchUnitPort(inMaxID, outMaxID, params), params.maxConfigs))
    val responsePending = RegNext(arb.io.out.fire, false.B)
    val owner = RegEnable(arb.io.chosen, arb.io.out.fire)
    arb.io.out.ready := true.B
    io.DirectoryRequest.valid := arb.io.out.fire
    io.DirectoryRequest.bits := arb.io.out.bits.descriptor.addr
    // FetchUnit consumes the directory decision on the cycle after allocation.
    // A response uses the registered owner while the next probe is accepted.
    assert(io.DirectoryResponse.valid === responsePending)
    for (config <- 0 until params.maxConfigs) {
        arb.io.in(config).bits := io.Requests(config).bits
        arb.io.in(config).valid := io.Requests(config).valid && io.ToFetch(config).ready
        io.Requests(config).ready := arb.io.in(config).ready && io.ToFetch(config).ready
        io.ToFetch(config).bits := io.Requests(config).bits
        io.ToFetch(config).valid := arb.io.out.valid && arb.io.chosen === config.U
        io.CacheHit(config) := responsePending && io.DirectoryResponse.valid && owner === config.U && io.DirectoryResponse.bits
    }
}

case class FetchUnitIO(tlInParams: TLBundleParameters, tlOutParams: TLBundleParameters, inMaxID : Int, outMaxID : Int, dataRegWidth : Int, toCacheParams: TLBundleParameters, params: RelMemParams) extends Bundle
{

    

// Requestor Port
        val Requestor = Flipped(Decoupled(new RequestorFetchUnitPort(inMaxID, outMaxID, params))) // Receive address to request from the Requestor Module]
        val Config = Flipped(RMEConfigPortIO(params))
        val Prefetch = Flipped(new FetchUnitPrefetchUnitPort(inMaxID, outMaxID, params))
        //val FetchReq = Flipped(Decoupled(Output(new TLBundleA(tlInEdge.bundle))))
        //val isBaseRequest = Flipped(Output(Bool()))
        //val Requestor_isBaseRequest = Flipped(Decoupled(Bool()))
        //val Requestor_FetchReq = Flipped(Decoupled(new TLBundleA(tlInEdge.bundle)))


        // LLC Port
        val IncomingReqInCache = Input(Bool())
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
        val ControlUnit = Decoupled(FetchUnitControlPort(tlInParams, inMaxID, outMaxID, dataRegWidth, params = params))
        val PackerAdmission = Vec(2, new PackerAdmissionQuery(inMaxID))
        val PrefetchReady = Input(Vec(params.maxConfigs, Bool()))

}





class FetchUnitRME(params: RelMemParams, tlOutEdge: TLEdgeOut, cachedRegionEdge: TLEdgeIn,
    toCacheOutTLInEdge: TLEdgeOut, instance: Int)(implicit p: Parameters) extends Module {

        val toCacheTLParams = toCacheOutTLInEdge.bundle

        val tlOutParams = tlOutEdge.bundle
        val tlInParams = cachedRegionEdge.bundle
        val inMaxID = (math.pow(2, cachedRegionEdge.bundle.sourceBits)-1).toInt
        val outMaxID = (math.pow(2, tlOutParams.sourceBits)-1).toInt
        val beatWidth = 8
        val dataRegWidth = (math.pow(2, params.maxDataSize+1)).toInt * beatWidth // this should give us the extra byte we need to extract excesses
        require(instance >= 0 && instance < params.maxConfigs)
        val nEntries = params.fetchEntries(instance)
        val sourceOffset = params.fetchOffset(instance)
        require(params.nFetchUnits <= (outMaxID + 1) / 2,
            "DTU fetch IDs must fit in the expanded source-ID space")
        val io = IO(new FetchUnitIO(tlInParams, tlOutParams, inMaxID, outMaxID, dataRegWidth, toCacheTLParams, params)).suggestName(s"fetchunitio_$instance")

        io.OutReq.valid := false.B
        io.OutReq.bits := 0.U.asTypeOf(new TLBundleA(tlOutParams))

        io.LLCOutReq.valid := false.B
        io.LLCOutReq.bits := 0.U.asTypeOf(new TLBundleA(toCacheTLParams))

        // Maximum coalescable requests
        val nExtractDesc = 16
        val FREE_REQ_ENTRY = nExtractDesc+1


        // Request Table Init //
        val emptyEntry = Wire(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc, params))
        emptyEntry.descriptor := 0.U.asTypeOf(RequestDescriptor(inMaxID, outMaxID, params))
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
        val receivingEntry = RegInit(0.U(math.max(1, log2Ceil(nEntries)).W))
        val receivingEntry2 = RegInit(0.U(math.max(1, log2Ceil(nEntries)).W))
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
        val requestTable = RegInit(VecInit(Seq.fill(nEntries)(emptyEntry)))
        // HELPER FUNCTIONS 


        def ResetEntry(entry: UInt) : Unit = {
            requestTable(entry) := emptyEntry
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
            val reqTableEntry = Wire(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc, params))
            reqTableEntry := 0.U.asTypeOf(new RequestTableEntry(inMaxID, outMaxID, nExtractDesc, params))
            reqTableEntry.descriptor := reqPort.descriptor
            val pos = reqPort.extractionDescriptor.pos
            reqTableEntry.extractionDescriptors(pos) := reqPort.extractionDescriptor
            reqTableEntry.extractionDescriptorsValid(pos) := true.B
            reqTableEntry.active := true.B
            requestTable(entry) := reqTableEntry
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



        /*
            When we have an incoming request we have 3 options:
            1.  Zero flag is set, we can immediately forward the value itself to the ControlUnit.
                Still must allocate entry in RequestTable for easy arbitration.

            2. Cannot coalesce entry. So allocate new entry in request table.

            3. Can coalesce entry, so we add an extraction descriptor to the already existing entry
               We need to ensure that if this happens on same cycle as that data is being removed, it is handled properly
        */


        val hasOutReqToSend = RegInit(false.B)
        val outReqEntryToSend = RegInit(0.U(math.max(1, log2Ceil(nEntries)).W))
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
                val src = outMaxID.U - sourceOffset.U - alloc_entry
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

            val src = outMaxID.U - sourceOffset.U - outReqEntryToSend
            val maxDRAM = math.pow(2, 33).toLong
            val descriptorAddr = requestTable(outReqEntryToSend).descriptor.addr
            val descriptorConfig = requestTable(outReqEntryToSend).descriptor.config
            val isDTUAddr = descriptorAddr >= maxDRAM.U
            outReq := DescriptorToOutReq(requestTable(outReqEntryToSend).descriptor, src)
            val toDTUCond = isDTUAddr && AddrIsValidConfig(descriptorAddr, descriptorConfig)
            outReqToCache := DescriptorToCacheReq(requestTable(outReqEntryToSend).descriptor, sourceOffset.U +& outReqEntryToSend, toDTUCond)

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

        val (d_first, d_last, d_done, _, d_count) = tlOutEdge.firstlast2(io.inReply)
        val (d_first_cache, d_last_cache, d_done_cache, _, d_count_cache) = toCacheOutTLInEdge.firstlast2(io.LLCInReply)

        io.inReply.ready := false.B
       // io.LLCInReply.ready := false.B

        // we give preference to the cache
        val DRAMInReplyActive  = RegInit(false.B)
        val LLCInReplyActive = RegInit(false.B)

        val dramDrained = Wire(Bool())
        // Accept the next line's first beat while forwarding the buffered line.
        io.inReply.ready := !dataRegFull || dramDrained
        io.LLCInReply.ready := !dataReg2Full 

        when (io.inReply.fire) {
           // SynthesizePrintf("[FetchUnit]: inReply.firesrc %d (%d,%d,%d) dataRegfull %d\n", io.inReply.bits.source, d_first_cache, d_last_cache, d_done_cache, dataReg2Full)
        }
        when (io.inReply.valid) {
            val entry = outMaxID.U - io.inReply.bits.source
            assert(entry >= sourceOffset.U && entry < (sourceOffset + nEntries).U)
        }
        when (io.LLCInReply.valid) {
            assert(io.LLCInReply.bits.source >= sourceOffset.U &&
                io.LLCInReply.bits.source < (sourceOffset + nEntries).U)
        }
        when (io.Requestor.fire) { assert(io.Requestor.bits.descriptor.config === instance.U) }
        when (d_first && io.inReply.fire) {
            receivingEntry  := outMaxID.U - io.inReply.bits.source - sourceOffset.U
        }

        when (d_first_cache && io.LLCInReply.fire) {
            receivingEntry2  := io.LLCInReply.bits.source - sourceOffset.U
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
        dramDrained := outputFire && selectDRAM

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
        dataRegFull := (dataRegFull && !dramDrained) ||
            (d_last && io.inReply.fire)

        io.ControlUnit.valid := hasFullData && currTableEntry.descriptor.dst === DESTINATION.CONTROL_UNIT
        io.ControlUnit.bits.reqTableEntry := currTableEntry
        io.ControlUnit.bits.data := fullDataToSend

        io.Prefetch.ToPre.valid := hasFullData && currTableEntry.descriptor.dst === DESTINATION.PREFETCH_UNIT
        io.Prefetch.ToPre.bits.config := currTableEntry.descriptor.requestPlacement
        io.Prefetch.ToPre.bits.addr := currTableEntry.descriptor.addr
        io.Prefetch.ToPre.bits.data := fullDataToSend



        when (io.ControlUnit.fire || io.Prefetch.ToPre.fire)
        {
            when (selectDRAM)
            {
                ResetEntry(receivingEntry)
            }.otherwise
            {
                ResetEntry(receivingEntry2)
            }

        }
}


