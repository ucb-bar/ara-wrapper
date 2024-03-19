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


class AraRocketUnit(nLanes: Int, axiIdBits: Int)(implicit p: Parameters) extends RocketVectorUnit()(p) with HasCoreParameters {
  override lazy val module = new AraRocketImpl
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
    := memAXI4Node)

  class AraRocketImpl extends RocketVectorUnitModuleImp(this) with HasCoreParameters {
    val nXacts = 32

    val ara = Module(new AraBlackbox(nXacts, nLanes, axiIdBits, 64, 1, axiDataWidth))

    val mem_valid = RegNext(io.core.ex.valid, false.B) && !io.core.killm
    val mem_inst = RegEnable(io.core.ex.inst, io.core.ex.valid)
    val mem_rs1 = RegEnable(io.core.ex.rs1, io.core.ex.valid)
    val mem_rs2 = RegEnable(io.core.ex.rs2, io.core.ex.valid)
    val mem_vconfig = RegEnable(io.core.ex.vconfig, io.core.ex.valid)
    val mem_pc = RegEnable(io.core.ex.pc, io.core.ex.valid)

    val wb_valid = RegNext(mem_valid, false.B)
    val wb_inst = RegEnable(mem_inst, mem_valid)
    val wb_vconfig = RegEnable(mem_vconfig, mem_valid)
    val wb_rs1 = RegEnable(mem_rs1, mem_valid)
    val wb_rs2 = RegEnable(mem_rs2, mem_valid)
    val wb_pc = RegEnable(mem_pc, mem_valid)
    val wb_dec = Module(new AraEarlyVectorDecode)
    wb_dec.io.inst := wb_inst
    wb_dec.io.vconfig := wb_vconfig

    class Xact extends Bundle {
      val wfd = Bool()
      val wxd = Bool()
      val size = UInt(2.W)
      val rd = UInt(5.W)
    }

    val xact_valids = RegInit(VecInit.fill(nXacts)(false.B))
    val xacts = Reg(Vec(nXacts, new Xact))

    val wb_ready = !(xact_valids.andR)
    val next_xact_id = PriorityEncoder(xact_valids)

    when (wb_valid && io.core.wb.retire) {
      xact_valids(next_xact_id) := false.B
      xacts(next_xact_id).wxd := wb_dec.io.write_rd
      xacts(next_xact_id).wfd := wb_dec.io.write_frd
      xacts(next_xact_id).size := wb_vconfig.vtype.vsew
      xacts(next_xact_id).rd := wb_inst(11,7)
    }


    io.core.ex.ready := true.B
    io.core.mem.block_mem := ara.io.resp.store_pending
    io.core.mem.block_all := false.B
    io.core.wb.replay := !ara.io.resp.req_ready
    io.core.wb.retire := ara.io.resp.req_ready && wb_dec.io.legal
    io.core.wb.inst := wb_inst
    io.core.wb.rob_should_wb := wb_dec.io.write_rd
    io.core.wb.rob_should_wb_fp := wb_dec.io.write_frd
    io.core.wb.pc := wb_pc
    io.core.wb.xcpt := false.B // ara does not support precise traps
    io.core.wb.cause := DontCare
    io.core.wb.tval := DontCare
    io.core.resp.valid := ara.io.resp.resp_valid && (xacts(ara.io.resp.trans_id).wxd || xacts(ara.io.resp.trans_id).wfd)
    io.core.resp.bits.fp := xacts(ara.io.resp.trans_id).wfd
    io.core.resp.bits.size := xacts(ara.io.resp.trans_id).size
    io.core.resp.bits.rd := xacts(ara.io.resp.trans_id).rd
    io.core.resp.bits.data := ara.io.resp.result

    io.core.set_vstart.valid := true.B // ara does not need to support vstart != 0, since no precise traps
    io.core.set_vstart.bits := 0.U
    io.core.set_vxsat := false.B // ara does not set_vxsat
    io.core.set_vconfig.valid := false.B // ara does not support fault-first
    io.core.set_vconfig.bits := DontCare
    io.core.set_fflags.valid := ara.io.resp.fflags_valid
    io.core.set_fflags.bits := ara.io.resp.fflags
    io.core.trap_check_busy := false.B // ara does not have trap-check for precise faults
    io.core.backend_busy := xact_valids.orR

    when (ara.io.resp.resp_valid && ara.io.req.resp_ready) {
      xact_valids(ara.io.resp.trans_id) := false.B
    }

    io.tlb.req.valid := false.B // ara does not support virtual memory
    io.tlb.req.bits := DontCare
    io.tlb.s2_kill := false.B
    io.dmem := DontCare
    io.dmem.req.valid := false.B // ara uses axi-tilelink
    io.dmem.req.bits := DontCare
    io.fp_req.valid := false.B
    io.fp_req.bits := DontCare
    io.fp_resp.ready := false.B

    ara.io.clk_i := clock
    ara.io.rst_ni := !reset.asBool
    ara.io.req.req_valid := wb_valid && io.core.wb.retire && wb_dec.io.legal
    ara.io.req.resp_ready := io.core.resp.ready
    ara.io.req.insn := wb_inst
    ara.io.req.rs1 := wb_rs1
    ara.io.req.rs2 := wb_rs2
    ara.io.req.frm := io.core.wb.frm
    ara.io.req.trans_id := next_xact_id
    ara.io.req.store_pending := false.B // ????
    ara.io.req.acc_cons_en := false.B // ????
    ara.io.req.inval_ready := false.B // ????


    // connect the axi interface
    memAXI4Node.out foreach { case (out, edgeOut) =>
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
            useDCache = false
          )),
        )
      )) else tp
    }
    case other => other
  }
})
