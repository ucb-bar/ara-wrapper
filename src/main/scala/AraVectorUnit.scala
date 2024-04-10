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
    := AXI4ToTL() // convert to TL
    := AXI4UserYanker(Some(8))
    := AXI4Fragmenter()
    := memAXI4Node)
}

trait HasLazyAraImpl { this: LazyModuleImp =>
  def nLanes: Int
  def axiIdBits: Int
  def axiDataWidth: Int

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

  def memNode: AXI4MasterNode
  def eLen: Int

  val nXacts = 8

  def eewByteMask(eew: UInt) = (0 until (1+log2Ceil(64/8))).map { e =>
    Mux(e.U === eew, ((1 << (1 << e)) - 1).U, 0.U)
  }.reduce(_|_)((eLen/8)-1,0)
  def eewBitMask(eew: UInt) = FillInterleaved(8, eewByteMask(eew))

  val ara = Module(new AraBlackbox(nXacts, nLanes, axiIdBits, 64, 1, axiDataWidth))

  val mem_valid = RegNext(ex_valid, false.B) && !mem_kill
  val mem_inst = RegEnable(ex_inst, ex_valid)
  val mem_vconfig = RegEnable(ex_vconfig, ex_valid)
  val mem_rs1 = Mux(mem_inst(14,12).isOneOf(1.U, 5.U) && !mem_inst(6,0).isOneOf(7.U, 39.U),
    mem_frs1 & eewBitMask(mem_vconfig.vtype.vsew),
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
    def writes = wfd || wxd
  }

  val xact_valids = RegInit(VecInit.fill(nXacts)(false.B))
  val xacts = Reg(Vec(nXacts, new Xact))
  val store_pending = xact_valids.zip(xacts).map(t => t._1 && t._2.store).orR
  val load_pending = xact_valids.zip(xacts).map(t => t._1 && t._2.load).orR

  val wb_store = wb_inst(6,0) === "b0100111".U
  val wb_load = wb_inst(6,0) === "b0000111".U
  val wb_ready = !(xact_valids.andR) && !(wb_store_pending && (wb_load || wb_store))
  val next_xact_id = PriorityEncoder(~xact_valids)
  val wb_retire = wb_valid && iss_q.io.enq.fire

  when (wb_valid && wb_retire && wb_ready) {
    xact_valids(next_xact_id) := true.B
    xacts(next_xact_id).wxd := wb_dec.io.write_rd && !wb_set
    xacts(next_xact_id).wfd := wb_dec.io.write_frd && !wb_set
    xacts(next_xact_id).size := wb_vconfig.vtype.vsew
    xacts(next_xact_id).rd := wb_inst(11,7)
    xacts(next_xact_id).store := wb_store
    xacts(next_xact_id).load := wb_load
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
    ara.io.axi_resp_i_aw_ready    := out.aw.ready
    out.aw.valid                   := ara.io.axi_req_o_aw_valid
    out.aw.bits.id                 := ara.io.axi_req_o_aw_bits_id
    out.aw.bits.addr               := ara.io.axi_req_o_aw_bits_addr
    out.aw.bits.len                := ara.io.axi_req_o_aw_bits_len
    out.aw.bits.size               := ara.io.axi_req_o_aw_bits_size
    out.aw.bits.burst              := ara.io.axi_req_o_aw_bits_burst
    out.aw.bits.lock               := ara.io.axi_req_o_aw_bits_lock
    out.aw.bits.cache              := ara.io.axi_req_o_aw_bits_cache
    out.aw.bits.prot               := ara.io.axi_req_o_aw_bits_prot
    out.aw.bits.qos                := ara.io.axi_req_o_aw_bits_qos
    // unused signals
    assert(ara.io.axi_req_o_aw_bits_region === 0.U)
    assert(ara.io.axi_req_o_aw_bits_atop === 0.U)
    assert(ara.io.axi_req_o_aw_bits_user === 0.U)

    ara.io.axi_resp_i_w_ready     := out.w.ready
    out.w.valid                    := ara.io.axi_req_o_w_valid
    out.w.bits.data                := ara.io.axi_req_o_w_bits_data
    out.w.bits.strb                := ara.io.axi_req_o_w_bits_strb
    out.w.bits.last                := ara.io.axi_req_o_w_bits_last
    // unused signals
    assert(ara.io.axi_req_o_w_bits_user === 0.U)

    out.b.ready                    := ara.io.axi_req_o_b_ready
    ara.io.axi_resp_i_b_valid     := out.b.valid
    ara.io.axi_resp_i_b_bits_id   := out.b.bits.id
    ara.io.axi_resp_i_b_bits_resp := out.b.bits.resp
    ara.io.axi_resp_i_b_bits_user := 0.U // unused

    ara.io.axi_resp_i_ar_ready    := out.ar.ready
    out.ar.valid                   := ara.io.axi_req_o_ar_valid
    out.ar.bits.id                 := ara.io.axi_req_o_ar_bits_id
    out.ar.bits.addr               := ara.io.axi_req_o_ar_bits_addr
    out.ar.bits.len                := ara.io.axi_req_o_ar_bits_len
    out.ar.bits.size               := ara.io.axi_req_o_ar_bits_size
    out.ar.bits.burst              := ara.io.axi_req_o_ar_bits_burst
    out.ar.bits.lock               := ara.io.axi_req_o_ar_bits_lock
    out.ar.bits.cache              := ara.io.axi_req_o_ar_bits_cache
    out.ar.bits.prot               := ara.io.axi_req_o_ar_bits_prot
    out.ar.bits.qos                := ara.io.axi_req_o_ar_bits_qos
    // unused signals
    assert(ara.io.axi_req_o_ar_bits_region === 0.U)
    assert(ara.io.axi_req_o_ar_bits_user === 0.U)

    out.r.ready                    := ara.io.axi_req_o_r_ready
    ara.io.axi_resp_i_r_valid     := out.r.valid
    ara.io.axi_resp_i_r_bits_id   := out.r.bits.id
    ara.io.axi_resp_i_r_bits_data := out.r.bits.data
    ara.io.axi_resp_i_r_bits_resp := out.r.bits.resp
    ara.io.axi_resp_i_r_bits_last := out.r.bits.last
    ara.io.axi_resp_i_r_bits_user := 0.U // unused
  }

}

class AraShuttleUnit(val nLanes: Int, val axiIdBits: Int)(implicit p: Parameters) extends ShuttleVectorUnit()(p) with HasCoreParameters with HasLazyAra {
  override lazy val module = new AraShuttleImpl
  class AraShuttleImpl extends ShuttleVectorUnitModuleImp(this) with HasCoreParameters {

  }
}

class AraRocketUnit(val nLanes: Int, val axiIdBits: Int)(implicit p: Parameters) extends RocketVectorUnit()(p) with HasCoreParameters with HasLazyAra {
  override lazy val module = new AraRocketImpl(this)

  class AraRocketImpl(outer: AraRocketUnit) extends RocketVectorUnitModuleImp(outer) with HasCoreParameters with HasLazyAraImpl {
    def nLanes = outer.nLanes
    def axiIdBits = outer.axiIdBits
    def axiDataWidth = outer.axiDataWidth
    status := io.core.status
    ex_valid := io.core.ex.valid
    ex_inst := io.core.ex.inst
    ex_pc := io.core.ex.pc
    ex_vconfig := io.core.ex.vconfig
    ex_vstart := io.core.ex.vstart
    ex_rs1 := io.core.ex.rs1
    ex_rs2 := io.core.ex.rs2
    mem_kill := io.core.killm
    mem_frs1 := io.core.mem.frs1
    wb_store_pending := io.core.wb.store_pending
    wb_frm := io.core.wb.frm
    def memNode = memAXI4Node

    io.core.resp.valid := resp_q.io.deq.valid && xact_valids(resp_q.io.deq.bits.trans_id) && xacts(resp_q.io.deq.bits.trans_id).writes
    io.core.resp.bits.fp := xacts(resp_q.io.deq.bits.trans_id).wfd
    io.core.resp.bits.size := xacts(resp_q.io.deq.bits.trans_id).size
    io.core.resp.bits.rd := xacts(resp_q.io.deq.bits.trans_id).rd
    io.core.resp.bits.data := resp_q.io.deq.bits.result
    resp_q.io.deq.ready := io.core.resp.ready

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

class WithAraRocketVectorUnit(nLanes: Int = 2, axiIdBits: Int = 4, cores: Option[Seq[Int]] = None) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      val buildVector = cores.map(_.contains(tp.tileParams.tileId)).getOrElse(true)
      require(nLanes >= 2)
      if (buildVector) tp.copy(tileParams = tp.tileParams.copy(
        core = tp.tileParams.core.copy(
          vector = Some(RocketCoreVectorParams(
            build = ((p: Parameters) => new AraRocketUnit(nLanes, axiIdBits)(p)),
            vLen = 4096,
            vMemDataBits = 0,
            decoder = ((p: Parameters) => {
              val decoder = Module(new AraEarlyVectorDecode()(p))
              decoder
            }),
            useDCache = false,
            issueVConfig = true
          )),
        )
      )) else tp
    }
    case other => other
  }
})
