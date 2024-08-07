package ara

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.amba.axi4._

import shuttle.common.{ShuttleVectorUnit, ShuttleVectorUnitModuleImp}


trait HasLazyAra { this: LazyModule =>
  def atlNode: TLNode
  def nLanes: Int
  def axiIdBits: Int
  val axiDataWidth = (nLanes * 64) / 2

  val memAXI4Node = AXI4MasterNode(
    Seq(AXI4MasterPortParameters(
      masters = Seq(AXI4MasterParameters(
        name = "ara",
        aligned = true,
        maxFlight = Some(1), // how many does ara do???
        id = IdRange(0, 1 << axiIdBits))))))

  (atlNode := TLFIFOFixer(TLFIFOFixer.all) // fix FIFO ordering
    := TLWidthWidget(axiDataWidth/8) // reduce size of TL
    := UnsafeAXI4ToTL(32) // convert to TL
    := AXI4Buffer()
    := AXI4UserYanker(Some(8))
    := AXI4Fragmenter()
    := memAXI4Node)
}

trait HasLazyAraImpl { this: LazyModuleImp =>
  def nLanes: Int
  def vLen: Int
  def axiIdBits: Int
  def axiDataWidth: Int
  def enableDelay: Boolean

  val status = Wire(new MStatus)
  val ex_valid = Wire(Bool())
  val ex_inst = Wire(UInt(32.W))
  val ex_pc = Wire(UInt())
  val ex_vconfig = Wire(new VConfig)
  val ex_vstart = Wire(UInt())
  val ex_rs1 = Wire(UInt(64.W))
  val ex_rs2 = Wire(UInt(64.W))
  val mem_kill = Wire(Bool())
  val mem_frs1 = Wire(UInt(64.W))
  val wb_store_pending = Wire(Bool())
  val wb_frm = Wire(UInt())
  val resp_ready = Wire(Bool())

  def memNode: AXI4MasterNode
  def eLen: Int

  val nXacts = 8

  def eewByteMask(eew: UInt) = (0 until (1+log2Ceil(64/8))).map { e =>
    Mux(e.U === eew, ((1 << (1 << e)) - 1).U, 0.U)
  }.reduce(_|_)((eLen/8)-1,0)
  def eewBitMask(eew: UInt) = FillInterleaved(8, eewByteMask(eew))

  val ara = Module(new AraBlackbox(vLen, nXacts, nLanes, axiIdBits, 64, 1, axiDataWidth))

  val mem_valid = RegNext(ex_valid, false.B) && !mem_kill
  val mem_inst = RegEnable(ex_inst, ex_valid)
  val mem_vconfig = RegEnable(ex_vconfig, ex_valid)
  val mem_rs1 = Mux(mem_inst(14,12).isOneOf(1.U, 5.U) && !mem_inst(6,0).isOneOf(7.U, 39.U),
    mem_frs1 & eewBitMask(mem_vconfig.vtype.vsew) | (~(0.U(64.W)) & ~eewBitMask(mem_vconfig.vtype.vsew)),
    RegEnable(ex_rs1, ex_valid))
  val mem_rs2 = RegEnable(ex_rs2, ex_valid)
  val mem_pc = RegEnable(ex_pc, ex_valid)

  val wb_valid = RegNext(mem_valid, false.B)
  val wb_inst = RegEnable(mem_inst, mem_valid)
  val wb_vconfig = RegEnable(mem_vconfig, mem_valid)
  val wb_rs1 = RegEnable(mem_rs1, mem_valid)
  val wb_rs2 = RegEnable(mem_rs2, mem_valid)
  val wb_pc = RegEnable(mem_pc, mem_valid)
  val wb_dec = Module(new AraEarlyVectorDecode)
  wb_dec.io.inst := wb_inst
  wb_dec.io.vconfig := wb_vconfig

  val wb_set = Seq(Instructions.VSETVLI, Instructions.VSETIVLI, Instructions.VSETVL).map(_ === wb_inst).orR

  val iss_q = Module(new Queue(new Bundle {
    val insn = UInt(32.W)
    val rs1 = UInt(64.W)
    val rs2 = UInt(64.W)
    val frm = UInt(2.W)
    val trans_id = UInt(log2Ceil(nXacts).W)
  }, 2))

  val resp_q = Module(new Queue(new Bundle {
    val result = UInt(32.W)
    val trans_id = UInt(log2Ceil(nXacts).W)
  }, 2))

  class Xact extends Bundle {
    val wfd = Bool()
    val wxd = Bool()
    val size = UInt(2.W)
    val rd = UInt(5.W)
    val store = Bool()
    val load = Bool()
    val vset = Bool()
    def writes = wfd || wxd
  }

  val xact_valids = RegInit(VecInit.fill(nXacts)(false.B))
  val xacts = Reg(Vec(nXacts, new Xact))
  val store_pending = xact_valids.zip(xacts).map(t => t._1 && t._2.store).orR || ara.io.resp.store_pending
  val load_pending = xact_valids.zip(xacts).map(t => t._1 && t._2.load).orR

  val wb_store = wb_inst(6,0) === "b0100111".U
  val wb_load = wb_inst(6,0) === "b0000111".U
  val wb_ready = !(xact_valids.andR) && !(wb_store_pending && (wb_load || wb_store))
  val next_xact_id = PriorityEncoder(~xact_valids)
  val wb_retire = wb_valid && iss_q.io.enq.fire

  when (wb_valid && wb_retire && wb_ready) {
    xact_valids(next_xact_id) := true.B
    xacts(next_xact_id).wxd := wb_dec.io.write_rd || wb_set
    xacts(next_xact_id).wfd := wb_dec.io.write_frd && !wb_set
    xacts(next_xact_id).size := wb_vconfig.vtype.vsew
    xacts(next_xact_id).rd := wb_inst(11,7)
    xacts(next_xact_id).store := wb_store
    xacts(next_xact_id).load := wb_load
    xacts(next_xact_id).vset := Seq(Instructions.VSETVLI, Instructions.VSETIVLI, Instructions.VSETVL).map(_ === wb_inst).orR
  }

  when (resp_q.io.deq.fire) {
    xact_valids(resp_q.io.deq.bits.trans_id) := false.B
  }

  iss_q.io.enq.valid := wb_valid && (wb_dec.io.legal || wb_set) && wb_ready
  iss_q.io.enq.bits.insn := wb_inst
  iss_q.io.enq.bits.rs1 := wb_rs1
  iss_q.io.enq.bits.rs2 := wb_rs2
  iss_q.io.enq.bits.frm := wb_frm
  iss_q.io.enq.bits.trans_id := next_xact_id
  iss_q.io.deq.ready := ara.io.resp.req_ready

  resp_q.io.enq.valid := ara.io.resp.resp_valid
  resp_q.io.enq.bits.result := ara.io.resp.result
  resp_q.io.enq.bits.trans_id := ara.io.resp.trans_id
  ara.io.req.resp_ready :=  resp_q.io.enq.ready

  val resp_id = resp_q.io.deq.bits.trans_id
  val resp_valid = resp_q.io.deq.valid && xact_valids(resp_id) && xacts(resp_id).writes && !xacts(resp_id).vset
  val resp_fp = xacts(resp_q.io.deq.bits.trans_id).wfd
  val resp_size = xacts(resp_q.io.deq.bits.trans_id).size
  val resp_rd = xacts(resp_q.io.deq.bits.trans_id).rd
  val resp_data = resp_q.io.deq.bits.result
  resp_q.io.deq.ready := resp_ready || xacts(resp_id).vset

  ara.io.clk_i := clock
  ara.io.rst_ni := !reset.asBool
  ara.io.req.req_valid := iss_q.io.deq.valid
  ara.io.req.insn := iss_q.io.deq.bits.insn
  ara.io.req.rs1 := iss_q.io.deq.bits.rs1
  ara.io.req.rs2 := iss_q.io.deq.bits.rs2
  ara.io.req.frm := iss_q.io.deq.bits.frm
  ara.io.req.trans_id := iss_q.io.deq.bits.trans_id
  ara.io.req.store_pending := false.B // ????
  ara.io.req.acc_cons_en := false.B // ????
  ara.io.req.inval_ready := false.B // ????

  // connect the axi interface
  memNode.out foreach { case (out, edgeOut) =>
    val aw = Wire(Decoupled(out.aw.bits.cloneType))
    ara.io.axi_resp_i_aw_ready    := aw.ready
    aw.valid                   := ara.io.axi_req_o_aw_valid
    aw.bits.id                 := ara.io.axi_req_o_aw_bits_id
    aw.bits.addr               := ara.io.axi_req_o_aw_bits_addr
    aw.bits.len                := ara.io.axi_req_o_aw_bits_len
    aw.bits.size               := ara.io.axi_req_o_aw_bits_size
    aw.bits.burst              := ara.io.axi_req_o_aw_bits_burst
    aw.bits.lock               := ara.io.axi_req_o_aw_bits_lock
    aw.bits.cache              := ara.io.axi_req_o_aw_bits_cache
    aw.bits.prot               := ara.io.axi_req_o_aw_bits_prot
    aw.bits.qos                := ara.io.axi_req_o_aw_bits_qos
    // unused signals
    assert(ara.io.axi_req_o_aw_bits_region === 0.U)
    assert(ara.io.axi_req_o_aw_bits_atop === 0.U)
    assert(ara.io.axi_req_o_aw_bits_user === 0.U)

    val w = Wire(Decoupled(out.w.bits.cloneType))
    ara.io.axi_resp_i_w_ready     := w.ready
    w.valid                    := ara.io.axi_req_o_w_valid
    w.bits.data                := ara.io.axi_req_o_w_bits_data
    w.bits.strb                := ara.io.axi_req_o_w_bits_strb
    w.bits.last                := ara.io.axi_req_o_w_bits_last
    // unused signals
    assert(ara.io.axi_req_o_w_bits_user === 0.U)

    out.b.ready                    := ara.io.axi_req_o_b_ready
    ara.io.axi_resp_i_b_valid     := out.b.valid
    ara.io.axi_resp_i_b_bits_id   := out.b.bits.id
    ara.io.axi_resp_i_b_bits_resp := out.b.bits.resp
    ara.io.axi_resp_i_b_bits_user := 0.U // unused

    val ar = Wire(Decoupled(out.ar.bits.cloneType))
    ara.io.axi_resp_i_ar_ready    := ar.ready
    ar.valid                   := ara.io.axi_req_o_ar_valid
    ar.bits.id                 := ara.io.axi_req_o_ar_bits_id
    ar.bits.addr               := ara.io.axi_req_o_ar_bits_addr
    ar.bits.len                := ara.io.axi_req_o_ar_bits_len
    ar.bits.size               := ara.io.axi_req_o_ar_bits_size
    ar.bits.burst              := ara.io.axi_req_o_ar_bits_burst
    ar.bits.lock               := ara.io.axi_req_o_ar_bits_lock
    ar.bits.cache              := ara.io.axi_req_o_ar_bits_cache
    ar.bits.prot               := ara.io.axi_req_o_ar_bits_prot
    ar.bits.qos                := ara.io.axi_req_o_ar_bits_qos
    // unused signals
    assert(ara.io.axi_req_o_ar_bits_region === 0.U)
    assert(ara.io.axi_req_o_ar_bits_user === 0.U)

    if (enableDelay) {
      val latency = Wire(UInt(32.W))
      latency := PlusArg("ara_mem_latency")
      val delay_timer = RegInit(0.U(64.W))
      delay_timer := delay_timer + 1.U
      val ar_delay = Module(new DelayQueue(ar.bits.cloneType, 1024, 64))
      val aw_delay = Module(new DelayQueue(aw.bits.cloneType, 1024, 64))
      val w_delay = Module(new DelayQueue(w.bits.cloneType, 1024, 64))
      ar_delay.io.timer := delay_timer
      aw_delay.io.timer := delay_timer
      w_delay.io.timer := delay_timer
      ar_delay.io.delay := latency
      aw_delay.io.delay := latency
      w_delay.io.delay := latency
      ar_delay.io.enq <> ar
      aw_delay.io.enq <> aw
      w_delay.io.enq <> w
      out.ar <> ar_delay.io.deq
      out.aw <> aw_delay.io.deq
      out.w <> w_delay.io.deq
    } else {
      out.ar <> ar
      out.aw <> aw
      out.w <> w
    }

    out.r.ready                    := ara.io.axi_req_o_r_ready
    ara.io.axi_resp_i_r_valid     := out.r.valid
    ara.io.axi_resp_i_r_bits_id   := out.r.bits.id
    ara.io.axi_resp_i_r_bits_data := out.r.bits.data
    ara.io.axi_resp_i_r_bits_resp := out.r.bits.resp
    ara.io.axi_resp_i_r_bits_last := out.r.bits.last
    ara.io.axi_resp_i_r_bits_user := 0.U // unused
  }

}

class AraShuttleUnit(val nLanes: Int, val axiIdBits: Int, val enableDelay: Boolean)(implicit p: Parameters) extends ShuttleVectorUnit()(p) with HasCoreParameters with HasLazyAra {
  override lazy val module = new AraShuttleImpl(this)
  class AraShuttleImpl(outer: AraShuttleUnit) extends ShuttleVectorUnitModuleImp(outer) with HasCoreParameters with HasLazyAraImpl {
    def nLanes = outer.nLanes
    def axiIdBits = outer.axiIdBits
    def axiDataWidth = outer.axiDataWidth
    def enableDelay = outer.enableDelay
    status := io.status
    ex_valid := io.ex.valid && io.ex.fire
    ex_inst := io.ex.uop.inst
    ex_pc := io.ex.uop.pc
    ex_vconfig := io.ex.vconfig
    ex_vstart := io.ex.vstart
    ex_rs1 := io.ex.uop.rs1_data
    ex_rs2 := io.ex.uop.rs2_data
    mem_kill := io.mem.kill || io.wb.block_all
    mem_frs1 := io.mem.frs1
    wb_store_pending := io.wb.store_pending
    wb_frm := io.wb.frm
    def memNode = memAXI4Node

    io.resp.valid := resp_valid
    io.resp.bits.fp := resp_fp
    io.resp.bits.size := resp_size
    io.resp.bits.rd := resp_rd
    io.resp.bits.data := resp_data
    resp_ready := io.resp.ready

    io.ex.ready := true.B
    io.wb.block_all := wb_valid && !iss_q.io.enq.fire
    io.wb.internal_replay := false.B
    io.wb.retire_late := false.B
    io.wb.inst := wb_inst
    io.wb.rob_should_wb := wb_dec.io.write_rd || wb_set
    io.wb.rob_should_wb_fp := wb_dec.io.write_frd && !wb_set
    io.wb.pc := wb_pc
    io.wb.xcpt := false.B // ara does not support precise traps
    io.wb.cause := DontCare
    io.wb.tval := DontCare
    io.wb.scalar_check.ready := !(store_pending || (io.wb.scalar_check.bits.store && load_pending))

    io.set_vstart.valid := wb_valid && iss_q.io.enq.fire // ara does not need to support vstart != 0, since no precise traps
    io.set_vstart.bits := 0.U
    io.set_vxsat := false.B // ara does not set_vxsat
    io.set_vconfig.valid := false.B // ara does not support fault-first
    io.set_vconfig.bits := DontCare
    io.set_fflags.valid := ara.io.resp.fflags_valid
    io.set_fflags.bits := ara.io.resp.fflags
    io.trap_check_busy := false.B // ara does not have trap-check for precise faults
    io.backend_busy := xact_valids.orR

    io.mem.tlb_req.valid := false.B
    io.mem.tlb_req.bits := DontCare
  }
}

class AraRocketUnit(val nLanes: Int, val axiIdBits: Int, val enableDelay: Boolean)(implicit p: Parameters) extends RocketVectorUnit()(p) with HasCoreParameters with HasLazyAra {
  override lazy val module = new AraRocketImpl(this)

  class AraRocketImpl(outer: AraRocketUnit) extends RocketVectorUnitModuleImp(outer) with HasCoreParameters with HasLazyAraImpl {
    def nLanes = outer.nLanes
    def axiIdBits = outer.axiIdBits
    def axiDataWidth = outer.axiDataWidth
    def enableDelay = outer.enableDelay
    status := io.core.status
    ex_valid := io.core.ex.valid && io.core.ex.ready && !io.core.wb.replay
    ex_inst := io.core.ex.inst
    ex_pc := io.core.ex.pc
    ex_vconfig := io.core.ex.vconfig
    ex_vstart := io.core.ex.vstart
    ex_rs1 := io.core.ex.rs1
    ex_rs2 := io.core.ex.rs2
    mem_kill := io.core.killm || io.core.wb.replay
    mem_frs1 := io.core.mem.frs1
    wb_store_pending := io.core.wb.store_pending
    wb_frm := io.core.wb.frm
    def memNode = memAXI4Node

    io.core.resp.valid := resp_valid 
    io.core.resp.bits.fp := resp_fp
    io.core.resp.bits.size := resp_size
    io.core.resp.bits.rd := resp_rd
    io.core.resp.bits.data := resp_data
    resp_ready := io.core.resp.ready

    io.core.ex.ready := true.B
    io.core.mem.block_mem := store_pending || (isWrite(io.tlb.s1_resp.cmd) && load_pending)
    io.core.mem.block_all := false.B
    io.core.wb.replay := wb_valid && !iss_q.io.enq.fire
    io.core.wb.retire := wb_valid && iss_q.io.enq.fire
    io.core.wb.inst := wb_inst
    io.core.wb.rob_should_wb := wb_dec.io.write_rd || wb_set
    io.core.wb.rob_should_wb_fp := wb_dec.io.write_frd && !wb_set
    io.core.wb.pc := wb_pc
    io.core.wb.xcpt := false.B // ara does not support precise traps
    io.core.wb.cause := DontCare
    io.core.wb.tval := DontCare

    io.core.set_vstart.valid := wb_valid && io.core.wb.retire // ara does not need to support vstart != 0, since no precise traps
    io.core.set_vstart.bits := 0.U
    io.core.set_vxsat := false.B // ara does not set_vxsat
    io.core.set_vconfig.valid := false.B // ara does not support fault-first
    io.core.set_vconfig.bits := DontCare
    io.core.set_fflags.valid := ara.io.resp.fflags_valid
    io.core.set_fflags.bits := ara.io.resp.fflags
    io.core.trap_check_busy := false.B // ara does not have trap-check for precise faults
    io.core.backend_busy := xact_valids.orR

    io.tlb.req.valid := false.B // ara does not support virtual memory
    io.tlb.req.bits := DontCare
    io.tlb.s2_kill := false.B
    io.dmem := DontCare
    io.dmem.req.valid := false.B // ara uses axi-tilelink
    io.dmem.req.bits := DontCare
    io.fp_req.valid := false.B
    io.fp_req.bits := DontCare
    io.fp_resp.ready := false.B
  }
}
