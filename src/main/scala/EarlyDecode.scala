package ara

import chisel3._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._

class AraEarlyVectorDecode(implicit p: Parameters) extends RocketVectorDecoder()(p) {
  def mopUnit      = 0.U(2.W)
  def mopUnordered = 1.U(2.W)
  def mopStrided   = 2.U(2.W)
  def mopOrdered   = 3.U(2.W)

  def lumopUnit  = "b00000".U
  def lumopWhole = "b01000".U
  def lumopMask  = "b01011".U
  def lumopFF    = "b10000".U


  def sumopUnit  = "b00000".U
  def sumopWhole = "b01000".U
  def sumopMask  = "b01011".U

  def opcLoad   = "b0000111".U
  def opcStore  = "b0100111".U
  def opcVector = "b1010111".U

  def OPIVV = "b000".U(3.W)
  def OPFVV = "b001".U(3.W)
  def OPMVV = "b010".U(3.W)
  def OPIVI = "b011".U(3.W)
  def OPIVX = "b100".U(3.W)
  def OPFVF = "b101".U(3.W)
  def OPMVX = "b110".U(3.W)
  def OPCFG = "b111".U(3.W)

  io.legal := false.B
  io.fp := false.B
  io.read_rs1 := false.B
  io.read_rs2 := false.B
  io.read_frs1 := false.B
  io.write_rd := false.B
  io.write_frd := false.B

  val opcode = io.inst(6,0)

  val width = io.inst(14,12)
  val lumop = io.inst(24,20)
  val sumop = lumop
  val vm = io.inst(25)
  val mop = io.inst(27,26)
  val mew = io.inst(28)
  val nf = io.inst(31,29)
  val funct3 = io.inst(14,12)
  val funct6 = io.inst(31,26)

  val v_load = opcode === opcLoad
  val v_store = opcode === opcStore
  val v_arith = opcode === opcVector && funct3 =/= 7.U

  when (v_load || v_store) {
    io.legal := mew === 0.U && width.isOneOf(0.U, 5.U, 6.U, 7.U)
    val unit = mop === 0.U
    when (unit) {
      when (v_load && !lumop.isOneOf(lumopUnit, lumopWhole, lumopMask, lumopFF)) { io.legal := false.B }
      when (v_store && !sumop.isOneOf(sumopUnit, sumopWhole, sumopMask)) { io.legal := false.B }
    }
    when (mew === 1.U) { io.legal := false.B }
    io.read_rs1 := true.B
    io.read_rs2 := mop === mopStrided
  } .elsewhen (v_arith) {
    io.legal := true.B
    io.read_rs1 := funct3.isOneOf(OPIVX, OPMVX)
    io.read_frs1 := funct3 === OPFVF
    io.write_rd := funct3 === OPMVV && funct6 === "b010000".U
    io.write_frd := funct3 === OPFVV && funct6 === "b010000".U
    io.fp := funct3 === OPFVF
  }
}
