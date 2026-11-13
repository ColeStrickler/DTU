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
import freechips.rocketchip.subsystem.{CacheBlockBytes}
import _root_.subsystem.rme._


case class SingleNextLinePrefetcherParams(
  ahead: Int = 4,
  waitForHit: Boolean = false,
  handleVA: Boolean = false
) 

class SnoopRME(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt()
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}

class PrefetchRME(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt()
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}

class PrefetcherIORME(implicit p: Parameters) extends Bundle {
  val snoop = Input(Valid(new SnoopRME))
  val request = Decoupled(new PrefetchRME)
  val hit = Output(Bool())
}

abstract class AbstractPrefetcherRME(implicit p: Parameters) extends Module {
  val io = IO(new PrefetcherIORME)

  io.request.valid := false.B
  io.request.bits := DontCare
  io.request.bits.address := 0.U(1.W)
  io.hit := false.B
}

class SingleNextLinePrefetcherRME(params: SingleNextLinePrefetcherParams)(implicit p: Parameters) extends AbstractPrefetcherRME()(p) {
  // Assume 4KB pages
  val lowerBits = 12 - log2Ceil(p(CacheBlockBytes))
  val s_idle :: s_wait :: s_active :: s_done :: Nil = Enum(4)
  val state = RegInit(s_idle)
  val write = Reg(Bool())
  val block_upper = Reg(UInt())
  val block_lower = Reg(UInt(lowerBits.W))
  val prefetch = Reg(UInt(lowerBits.W))
  val wrap = RegInit(false.B)
  val wrap_block_upper = block_upper + 1.U
  val delta = Mux(wrap, Cat(1.U(1.W), prefetch) - block_lower, prefetch - block_lower)

  val addr_hit = if (params.handleVA) {
    (io.snoop.bits.block >= Cat(block_upper, block_lower)) &&
    (io.snoop.bits.block <= Cat(Mux(wrap, wrap_block_upper, block_upper), prefetch))
  } else {
    (block_upper === io.snoop.bits.block >> lowerBits) &&
    (io.snoop.bits.block(lowerBits-1,0) >= block_lower) &&
    (io.snoop.bits.block(lowerBits-1,0) <= prefetch)
  }
  io.hit := state =/= s_idle && addr_hit

  val snoop_next_block = (io.snoop.bits.block(lowerBits-1,0) + 1.U)(lowerBits-1,0)

  when ((state === s_idle || (state === s_wait && !io.hit)) && io.snoop.valid) {
    when (~io.snoop.bits.block(lowerBits-1,0) =/= 0.U || params.handleVA.B) {
      state := (if (params.waitForHit) s_wait else s_active)
    }
    block_upper := io.snoop.bits.block >> lowerBits
    block_lower := io.snoop.bits.block
    prefetch := snoop_next_block
    when (params.handleVA.B && !io.hit) {
      wrap := snoop_next_block === 0.U
    }
    write := io.snoop.bits.write
  }

  io.request.valid := state === s_active
  io.request.bits.write := write
  io.request.bits.address := Cat(Mux(wrap, wrap_block_upper, block_upper), prefetch) << log2Up(io.request.bits.blockBytes)


  when (io.request.fire) {
    prefetch := prefetch + 1.U
    when (prefetch === ~(0.U(lowerBits.W))) {
      if (params.handleVA) {
        state := Mux(delta >= params.ahead.U, s_wait, s_active)
        prefetch := 0.U
        wrap := true.B
      } else {
        state := s_done
        prefetch := prefetch
      }
    } .elsewhen (delta >= params.ahead.U) {
      state := s_wait
    } .otherwise {
      state := s_active
    }
  }
  when (state === s_done && block_lower === prefetch) {
    state := s_idle
  }

  when (io.hit && io.snoop.valid) {
    when (state =/= s_done && io.snoop.bits.block =/= Cat(block_upper, block_lower)) {
      state := s_active
    }
    write := io.snoop.bits.write
    block_lower := io.snoop.bits.block
    block_upper := io.snoop.bits.block >> lowerBits
    when (io.snoop.bits.block(lowerBits-1,0) === prefetch) {
      prefetch := prefetch + 1.U
    }
    when (wrap && ((io.snoop.bits.block >> lowerBits) =/= block_upper)) {
      wrap := false.B
    }
  }
}

case class RequestorInjectionRequest(params: RelMemParams) extends Bundle {
    val RequestAddr = Input(UInt(log2Ceil(params.rmeAddressSize).W))
    val InjectionReqNum = Input(UInt(log2Ceil(32).W)) // for now we will just use this to select
}


case class PrefetchUnitAGUIO(params: RelMemParams) extends Bundle {
   // val AsyncInjectionRequest = Flipped(Valid(UInt(log2Ceil(params.rmeAddressSize).W)))
    val InjectionRequest = Flipped(Decoupled(new RequestorInjectionRequest(params)))
    val Injection = Valid(Output(UInt(32.W)))
    val config_StreamPhysRegisters = Input(Vec(1, UInt(64.W)))
    val config_StreamDataSize = Input(UInt(32.W))
    val rst = Input(Bool())
}


case class PrefetchUnitFetchUnitPortIn(params: RelMemParams = RelMemParams()) extends Bundle
{
  val data = UInt(512.W) // 64 bytes = 1 cache line
  val addr = UInt(47.W) // will take out of the reqTableEntry
  val config = UInt(math.max(1, log2Ceil(params.maxConfigs)).W)
}

case class PrefetchUnitFetchUnitPort(inMaxID:Int, outmaxID : Int, params: RelMemParams = RelMemParams()) extends Bundle
{
  val ToFetchUnit = DecoupledIO(new RequestorFetchUnitPort(inMaxID, outmaxID, params))
  val ToPre = Flipped(Decoupled(new PrefetchUnitFetchUnitPortIn(params)))
}

case class FetchUnitPrefetchUnitPort(inMaxID:Int, outmaxID : Int, params: RelMemParams = RelMemParams()) extends Bundle
{
  //val ToFetchUnit = DecoupledIO(new RequestorFetchUnitPort(inMaxID, outmaxID, params))
  val ToPre = Flipped(Decoupled(new PrefetchUnitFetchUnitPortIn(params)))
}





case class PreFetchUnitIO(params: RelMemParams, inMaxID : Int, outmaxID : Int) extends Bundle {
    val Requestor = new PrefetchUnitAGUIO(params)
    val FetchUnit = new PrefetchUnitFetchUnitPort(inMaxID, outmaxID, params)
}





/*
    Track outbound requests
*/
case class PrefetchOutboundTableEntry(params: RelMemParams) extends Bundle {
    val address = UInt(47.W)
    val valid = Bool()
    val sent = Bool()
    val stale = Bool()
}



class PreFetchUnitRME(params: RelMemParams, tlInEdge : TLEdge, tlOutEdge: TLEdge, tlOutBundle: TLBundle, config: Int)(
    implicit p: Parameters) extends Module
{  
    val tlOutParams = tlOutEdge.bundle
    val tlInParams = tlInEdge.bundle
    val outMaxID = (math.pow(2, tlOutParams.sourceBits)-1).toInt
    val inMaxID = (math.pow(2, tlInParams.sourceBits)-1).toInt
    val depthAhead = 4

    val io = IO(new PreFetchUnitIO(params, inMaxID, outMaxID))



    


    val maxRMEOffsetBitWidth = log2Ceil(params.rmeAddressSize)
    val nInjectionPackets = depthAhead * 2
    val injectionPackets = RegInit(VecInit(Seq.fill(nInjectionPackets)(0.U(512.W))))
    // Physical tags also distinguish metadata streams across reconfiguration.
    val injectionPacketAddr = RegInit(VecInit(Seq.fill(nInjectionPackets)(0.U(47.W))))
    val injectionPacketsValid = RegInit(VecInit(Seq.fill(nInjectionPackets)(false.B)))
    val OldestInjectionPacket = RegInit(0.U(log2Ceil(nInjectionPackets).W))
    val stream2PhysicalAddressStart = RegNext(io.Requestor.config_StreamPhysRegisters(0),
      (params.rmeaddress + 0x08000000L).U(64.W))

    val OutBoundReqTable = RegInit(VecInit(Seq.fill(nInjectionPackets)(
      0.U.asTypeOf(new PrefetchOutboundTableEntry(params)))))
    def pendingMatches(addr: UInt): Vec[Bool] =
      VecInit(OutBoundReqTable.map(e => e.valid && e.address === addr))
    def cachedMatches(addr: UInt): Vec[Bool] =
      VecInit(injectionPacketAddr.zip(injectionPacketsValid).map {
        case (tag, valid) => valid && tag === addr
      })

    // Track all requests from queue admission to response, including speculation.
    // A configuration reset cancels queued requests but drains already issued ones.
    val localReset = reset.asBool || io.Requestor.rst
    val PrefetcherRME = withReset(localReset) {
      Module(new SingleNextLinePrefetcherRME(
        SingleNextLinePrefetcherParams(ahead=depthAhead, handleVA=true)))
    }
    val DownstreamReqQueue = withReset(localReset) {
      Module(new Queue(new RequestDescriptor(inMaxID, outMaxID, params), depthAhead, flow=false))
    }
    val RequestQueueArb = Module(new RRArbiter(
      new RequestDescriptor(inMaxID, outMaxID, params), 2))

    def MakeReqDescriptor(addr: UInt): RequestDescriptor = {
      val descriptor = Wire(new RequestDescriptor(inMaxID, outMaxID, params))
      descriptor := 0.U.asTypeOf(descriptor)
      descriptor.requestPlacement := config.U
      descriptor.dst := DESTINATION.PREFETCH_UNIT
      descriptor.addr := addr
      descriptor.size := 6.U
      descriptor.config := config.U
      descriptor
    }

    io.FetchUnit.ToPre.ready := true.B
    val responseAddr = io.FetchUnit.ToPre.bits.addr
    val responseMatches = pendingMatches(responseAddr)
    val responseEntry = PriorityEncoder(responseMatches)
    val freshResponse = io.FetchUnit.ToPre.fire && responseMatches.asUInt.orR &&
      !OutBoundReqTable(responseEntry).stale && !io.Requestor.rst

    // Updating a matching slot avoids duplicate tags consuming cache capacity.
    val responseCached = cachedMatches(responseAddr)
    val refillSlot = Mux(responseCached.asUInt.orR,
      PriorityEncoder(responseCached), OldestInjectionPacket)
    when (freshResponse) {
      injectionPackets(refillSlot) := io.FetchUnit.ToPre.bits.data
      injectionPacketAddr(refillSlot) := responseAddr
      injectionPacketsValid(refillSlot) := true.B
      when (!responseCached.asUInt.orR) {
        OldestInjectionPacket := Mux(OldestInjectionPacket === (nInjectionPackets-1).U,
          0.U, OldestInjectionPacket + 1.U)
      }
    }

    val InjectionReqAddr = io.Requestor.InjectionRequest.bits.RequestAddr
    val demandAddr = (stream2PhysicalAddressStart + InjectionReqAddr)(46, 0)
    val demandCached = cachedMatches(demandAddr)
    val demandReturning = freshResponse && responseAddr === demandAddr
    val demandAvailable = demandCached.asUInt.orR || demandReturning
    val demandPending = pendingMatches(demandAddr).asUInt.orR
    val demandPacket = Mux(demandReturning, io.FetchUnit.ToPre.bits.data,
      injectionPackets(PriorityEncoder(demandCached)))
    io.Requestor.InjectionRequest.ready := demandAvailable && !io.Requestor.rst
    io.Requestor.Injection.valid := io.Requestor.InjectionRequest.valid &&
      io.Requestor.InjectionRequest.ready
    io.Requestor.Injection.bits := demandPacket.asTypeOf(Vec(16, UInt(32.W)))(
      io.Requestor.InjectionRequest.bits.InjectionReqNum(3, 0))

    PrefetcherRME.io.snoop.valid := io.Requestor.InjectionRequest.fire
    PrefetcherRME.io.snoop.bits.write := false.B
    PrefetcherRME.io.snoop.bits.address := InjectionReqAddr

    val speculativeOffset = PrefetcherRME.io.request.bits.address
    val speculativeAddr = (stream2PhysicalAddressStart + speculativeOffset)(46, 0)
    val speculativeRedundant = cachedMatches(speculativeAddr).asUInt.orR ||
      pendingMatches(speculativeAddr).asUInt.orR ||
      (freshResponse && responseAddr === speculativeAddr)
    val skipSpeculation = speculativeRedundant || speculativeOffset >= params.rmeAddressSize.U
    RequestQueueArb.io.in(0).bits := MakeReqDescriptor(speculativeAddr)
    RequestQueueArb.io.in(0).valid := PrefetcherRME.io.request.valid &&
      !skipSpeculation && !io.Requestor.rst
    // Skipped candidates must still advance the next-line predictor.
    PrefetcherRME.io.request.ready := !io.Requestor.rst &&
      (skipSpeculation || RequestQueueArb.io.in(0).ready)
    RequestQueueArb.io.in(1).bits := MakeReqDescriptor(demandAddr)
    RequestQueueArb.io.in(1).valid := io.Requestor.InjectionRequest.valid &&
      !demandAvailable && !demandPending && !io.Requestor.rst

    val freeEntries = VecInit(OutBoundReqTable.map(e => !e.valid))
    val hasFreeEntry = freeEntries.asUInt.orR
    DownstreamReqQueue.io.enq.bits := RequestQueueArb.io.out.bits
    DownstreamReqQueue.io.enq.valid := RequestQueueArb.io.out.valid && hasFreeEntry && !io.Requestor.rst
    RequestQueueArb.io.out.ready := DownstreamReqQueue.io.enq.ready && hasFreeEntry && !io.Requestor.rst
    when (DownstreamReqQueue.io.enq.fire) {
      val entry = PriorityEncoder(freeEntries)
      assert(!pendingMatches(DownstreamReqQueue.io.enq.bits.addr).asUInt.orR)
      OutBoundReqTable(entry).address := DownstreamReqQueue.io.enq.bits.addr
      OutBoundReqTable(entry).valid := true.B
      OutBoundReqTable(entry).sent := false.B
      OutBoundReqTable(entry).stale := false.B
    }

    io.FetchUnit.ToFetchUnit.valid := DownstreamReqQueue.io.deq.valid && !io.Requestor.rst
    io.FetchUnit.ToFetchUnit.bits.descriptor := DownstreamReqQueue.io.deq.bits
    io.FetchUnit.ToFetchUnit.bits.extractionDescriptor :=
      0.U.asTypeOf(io.FetchUnit.ToFetchUnit.bits.extractionDescriptor)
    DownstreamReqQueue.io.deq.ready := io.FetchUnit.ToFetchUnit.ready && !io.Requestor.rst
    val sentMatches = pendingMatches(DownstreamReqQueue.io.deq.bits.addr)
    when (io.FetchUnit.ToFetchUnit.fire) {
      assert(PopCount(sentMatches) === 1.U)
    }
    for (i <- 0 until nInjectionPackets) {
      when (io.FetchUnit.ToFetchUnit.fire && sentMatches(i)) {
        OutBoundReqTable(i).sent := true.B
      }
      when (io.FetchUnit.ToPre.fire && responseMatches(i)) {
        OutBoundReqTable(i).valid := false.B
      }
      when (io.Requestor.rst) {
        injectionPacketsValid(i) := false.B
        OutBoundReqTable(i).valid := OutBoundReqTable(i).valid &&
          OutBoundReqTable(i).sent && !(io.FetchUnit.ToPre.fire && responseMatches(i))
        OutBoundReqTable(i).stale := true.B
      }
    }
    when (io.Requestor.rst) { OldestInjectionPacket := 0.U }
}
