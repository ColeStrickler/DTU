package subsystem.rme

import chisel3._
import chisel3.util._
import midas.targetutils.SynthesizePrintf

// Observation only: these modules never drive functional ready/valid or data.
class DTUTraceEvent(nFetch: Int, sourceBits: Int, configBits: Int) extends Bundle {
  val llc = Bool()
  val slot = UInt(log2Ceil(nFetch).W)
  val source = UInt(sourceBits.W)
  val order = UInt(32.W)
  val config = UInt(configBits.W)
  val dst = UInt(1.W)
  val base = UInt(sourceBits.W)
  val place = UInt(7.W)
  val addr = UInt(47.W)
  val edge = UInt(2.W) // reply first in bit 0, last in bit 1; zero for issue/delivery
}

class DTUFetchHistory(nFetch: Int, sourceBits: Int, configBits: Int) extends Module {
  private val depth = 16
  val io = IO(new Bundle {
    // Lanes: 0=outgoing request accepted, 1=DRAM first/last beat accepted,
    // 2=LLC first/last beat accepted, 3=buffer delivered to its destination.
    val events = Input(Vec(4, Valid(new DTUTraceEvent(nFetch, sourceBits, configBits))))
    val snapshot = Input(Bool())
    val progress = Input(Bool())
  })
  val tick = RegInit(0.U(48.W)); tick := tick + 1.U
  val records = Reg(Vec(depth, chiselTypeOf(io.events)))
  val times = Reg(Vec(depth, UInt(48.W)))
  val writeIndex = RegInit(0.U(log2Ceil(depth).W))
  val count = RegInit(0.U(log2Ceil(depth + 1).W))
  val dumping = RegInit(false.B)
  val readIndex = RegInit(0.U(log2Ceil(depth).W))
  val remaining = RegInit(0.U(log2Ceil(depth + 1).W))
  val lane = RegInit(0.U(2.W))
  when (io.events.map(_.valid).reduce(_ || _) && !dumping && !io.snapshot) {
    records(writeIndex) := io.events
    times(writeIndex) := tick
    writeIndex := writeIndex + 1.U
    when (count =/= depth.U) { count := count + 1.U }
  }
  when (io.progress) {
    dumping := false.B
  }.elsewhen (io.snapshot) {
    dumping := count.orR
    readIndex := writeIndex - count // wrapping index: oldest retained event cycle
    remaining := count
    lane := 0.U
  }.elsewhen (dumping) {
    lane := lane + 1.U
    when (lane === 3.U) {
      readIndex := readIndex + 1.U
      remaining := remaining - 1.U
      when (remaining === 1.U) { dumping := false.B }
    }
  }
  val event = records(readIndex)(lane)
  when (dumping && !io.progress && event.valid) {
    SynthesizePrintf(
      "[DTU HISTORY] t=%d kind=%d edge=0x%x llc=%d slot=%d src=%d order=%d cfg=%d dst=%d base=%d place=%d addr=0x%x\n",
      times(readIndex), lane, event.bits.edge, event.bits.llc, event.bits.slot,
      event.bits.source, event.bits.order, event.bits.config, event.bits.dst,
      event.bits.base, event.bits.place, event.bits.addr)
  }
}

class DTUParentTrace(sourceBits: Int, addressBits: Int, maxConfigs: Int) extends Module {
  val io = IO(new Bundle {
    val accepted = Input(Valid(new Bundle {
      val source = UInt(sourceBits.W)
      val address = UInt(addressBits.W)
      val config = UInt(log2Ceil(maxConfigs).W)
    }))
    val started = Input(Vec(maxConfigs, Valid(UInt(sourceBits.W))))
    val generated = Input(Vec(maxConfigs, Valid(UInt(sourceBits.W))))
    val completed = Input(Valid(UInt(sourceBits.W)))
    val snapshot = Input(Bool())
    val progress = Input(Bool())
    val outstanding = Output(Vec(maxConfigs, UInt((sourceBits + 1).W)))
  })
  private val nParents = 1 << sourceBits
  val tick = RegInit(0.U(48.W)); tick := tick + 1.U
  val nextOrder = RegInit(1.U(32.W))
  val live = RegInit(VecInit(Seq.fill(nParents)(false.B)))
  val address = Reg(Vec(nParents, UInt(addressBits.W)))
  val config = Reg(Vec(nParents, UInt(log2Ceil(maxConfigs).W)))
  val order = Reg(Vec(nParents, UInt(32.W)))
  val acceptedAt = Reg(Vec(nParents, UInt(48.W)))
  val startedAt = Reg(Vec(nParents, UInt(48.W)))
  val generatedAt = Reg(Vec(nParents, UInt(48.W)))
  val started = RegInit(VecInit(Seq.fill(nParents)(false.B)))
  val generated = RegInit(VecInit(Seq.fill(nParents)(false.B)))
  when (io.completed.valid) { live(io.completed.bits) := false.B }
  when (io.accepted.valid) {
    val id = io.accepted.bits.source
    live(id) := true.B
    address(id) := io.accepted.bits.address
    config(id) := io.accepted.bits.config
    order(id) := nextOrder; nextOrder := nextOrder + 1.U
    acceptedAt(id) := tick
    started(id) := false.B; generated(id) := false.B
    startedAt(id) := 0.U; generatedAt(id) := 0.U
  }
  // These come after admission so flow-through queue starts in the same cycle
  // are retained. Source IDs may be reused after the prior final response.
  for (c <- 0 until maxConfigs) {
    when (io.started(c).valid) {
      started(io.started(c).bits) := true.B
      startedAt(io.started(c).bits) := tick
    }
    when (io.generated(c).valid) {
      generated(io.generated(c).bits) := true.B
      generatedAt(io.generated(c).bits) := tick
    }
    io.outstanding(c) := PopCount((0 until nParents).map(i => live(i) && config(i) === c.U))
  }
  val dumping = RegInit(false.B)
  val index = RegInit(0.U(sourceBits.W))
  when (io.progress) { dumping := false.B
  }.elsewhen (io.snapshot) { dumping := true.B; index := 0.U
  }.elsewhen (dumping) {
    index := index + 1.U
    when (index === (nParents - 1).U) { dumping := false.B }
  }
  when (dumping && !io.progress && live(index)) {
    SynthesizePrintf(
      "[DTU STALL parent] t=%d base=%d cfg=%d outstanding=%d addr=0x%x order=%d acceptT=%d started=%d startT=%d generated=%d genT=%d\n",
      tick, index, config(index), io.outstanding(config(index)), address(index),
      order(index), acceptedAt(index), started(index), startedAt(index),
      generated(index), generatedAt(index))
  }
}
