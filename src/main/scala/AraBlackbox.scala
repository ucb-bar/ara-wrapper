package ara

import sys.process._

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, StringParam}

import scala.collection.mutable.{ListBuffer}

import org.chipsalliance.cde.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._

import freechips.rocketchip.rocket._
import freechips.rocketchip.subsystem.{RocketCrossingParams}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._ 
import freechips.rocketchip.amba.axi4._

class AraBlackbox(vLen: Int, nXacts: Int, nLanes: Int, axiIdWidth: Int, axiAddrWidth: Int, axiUserWidth: Int, axiDataWidth: Int) extends BlackBox(Map(
  "TRANS_ID_WIDTH" -> IntParam(log2Ceil(nXacts)),
  "VLEN" -> IntParam(vLen), 
  "NLANES" -> IntParam(nLanes),
  "AXI_ID_WIDTH" -> IntParam(axiIdWidth),
  "AXI_ADDR_WIDTH" -> IntParam(axiAddrWidth),
  "AXI_USER_WIDTH" -> IntParam(axiUserWidth),
  "AXI_DATA_WIDTH" -> IntParam(axiDataWidth)
)) with HasBlackBoxResource { 
  require(axiDataWidth == (64 * nLanes / 2))
  val xactWidth = log2Ceil(nXacts)
  val io = IO(new Bundle {
    val clk_i = Input(Clock())
    val rst_ni = Input(Bool())

    val axi_resp_i_aw_ready      = Input(Bool())
    val axi_req_o_aw_valid       = Output(Bool())
    val axi_req_o_aw_bits_id     = Output(UInt(axiIdWidth.W))
    val axi_req_o_aw_bits_addr   = Output(UInt(axiAddrWidth.W))
    val axi_req_o_aw_bits_len    = Output(UInt(8.W))
    val axi_req_o_aw_bits_size   = Output(UInt(3.W))
    val axi_req_o_aw_bits_burst  = Output(UInt(2.W))
    val axi_req_o_aw_bits_lock   = Output(Bool())
    val axi_req_o_aw_bits_cache  = Output(UInt(4.W))
    val axi_req_o_aw_bits_prot   = Output(UInt(3.W))
    val axi_req_o_aw_bits_qos    = Output(UInt(4.W))
    val axi_req_o_aw_bits_region = Output(UInt(4.W))
    val axi_req_o_aw_bits_atop   = Output(UInt(6.W))
    val axi_req_o_aw_bits_user   = Output(UInt(axiUserWidth.W))

    val axi_resp_i_w_ready    = Input(Bool())
    val axi_req_o_w_valid     = Output(Bool())
    val axi_req_o_w_bits_data = Output(UInt(axiDataWidth.W))
    val axi_req_o_w_bits_strb = Output(UInt((axiDataWidth/8).W))
    val axi_req_o_w_bits_last = Output(Bool())
    val axi_req_o_w_bits_user = Output(UInt(axiUserWidth.W))

    val axi_resp_i_ar_ready      = Input(Bool())
    val axi_req_o_ar_valid       = Output(Bool())
    val axi_req_o_ar_bits_id     = Output(UInt(axiIdWidth.W))
    val axi_req_o_ar_bits_addr   = Output(UInt(axiAddrWidth.W))
    val axi_req_o_ar_bits_len    = Output(UInt(8.W))
    val axi_req_o_ar_bits_size   = Output(UInt(3.W))
    val axi_req_o_ar_bits_burst  = Output(UInt(2.W))
    val axi_req_o_ar_bits_lock   = Output(Bool())
    val axi_req_o_ar_bits_cache  = Output(UInt(4.W))
    val axi_req_o_ar_bits_prot   = Output(UInt(3.W))
    val axi_req_o_ar_bits_qos    = Output(UInt(4.W))
    val axi_req_o_ar_bits_region = Output(UInt(4.W))
    val axi_req_o_ar_bits_user   = Output(UInt(axiUserWidth.W))

    val axi_req_o_b_ready      = Output(Bool())
    val axi_resp_i_b_valid     = Input(Bool())
    val axi_resp_i_b_bits_id   = Input(UInt(axiIdWidth.W))
    val axi_resp_i_b_bits_resp = Input(UInt(2.W))
    val axi_resp_i_b_bits_user = Input(UInt(axiUserWidth.W))

    val axi_req_o_r_ready      = Output(Bool())
    val axi_resp_i_r_valid     = Input(Bool())
    val axi_resp_i_r_bits_id   = Input(UInt(axiIdWidth.W))
    val axi_resp_i_r_bits_data = Input(UInt(axiDataWidth.W))
    val axi_resp_i_r_bits_resp = Input(UInt(2.W))
    val axi_resp_i_r_bits_last = Input(Bool())
    val axi_resp_i_r_bits_user = Input(UInt(axiUserWidth.W))

    val req = Input(new Bundle {
      val req_valid = Bool()
      val resp_ready = Bool()
      val insn = UInt(32.W)
      val rs1 = UInt(64.W)
      val rs2 = UInt(64.W)
      val frm = UInt(2.W)
      val trans_id = UInt(xactWidth.W)
      val store_pending = Bool()
      val acc_cons_en = Bool()
      val inval_ready = Bool()
    })
    val resp = Output(new Bundle {
      val req_ready = Bool()
      val resp_valid = Bool()
      val result = UInt(64.W)
      val trans_id = UInt(xactWidth.W)
      val store_pending = Bool()
      val store_complete = Bool()
      val load_complete = Bool()
      val fflags = UInt(5.W)
      val fflags_valid = Bool()
      val inval_valid = Bool()
      val inval_addr = UInt(64.W)
    })
  })
  addResource("/ara/vsrc/AraBlackbox.sv")

}
