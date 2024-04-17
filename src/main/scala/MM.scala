package MM

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.subsystem.BaseSubsystem
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.UIntIsOneOf

// DOC include start: GCD params
case class MMParams(
  address: BigInt = 0x4000,
  width: Int = 32,
  width_addr: Int = 10,
  width_data: Int = 32,
  width_addr_c: Int = 6,
  width_data_c: Int = 64,
  useAXI4: Boolean = false,
  useBlackBox: Boolean = true)
// DOC include end: GCD params

// DOC include start: GCD key
case object MMKey extends Field[Option[MMParams]](None)
// DOC include end: GCD key

class MMIO(val w: Int) extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val input_ready = Output(Bool())
  val input_valid = Input(Bool())
  val output_ready = Input(Bool())
  val output_valid = Output(Bool())
  val sram_rdata_a0 = Input(UInt(32.W))
  val sram_rdata_a1 = Input(UInt(32.W))
  val sram_rdata_b0 = Input(UInt(32.W))
  val sram_rdata_b1 = Input(UInt(32.W))
  val sram_raddr_a0 = Input(UInt(10.W))
  val sram_raddr_a1 = Input(UInt(10.W))
  val sram_raddr_b0 = Input(UInt(10.W))
  val sram_raddr_b1 = Input(UInt(10.W))
  val sram_wdata_c00 = Output(UInt(64.W))
  val sram_wdata_c01 = Output(UInt(64.W))
  val sram_wdata_c10 = Output(UInt(64.W))
  val sram_wdata_c11 = Output(UInt(64.W))
  val sram_wdata_c20 = Output(UInt(64.W))
  val sram_wdata_c21 = Output(UInt(64.W))
  val sram_waddr_c0 = Output(UInt(6.W))
  val sram_waddr_c1 = Output(UInt(6.W))
  val sram_waddr_c2 = Output(UInt(6.W))
  val sram_write_enable_c0 = Output(UInt(1.W))
  val sram_write_enable_c1 = Output(UInt(1.W))
  val sram_write_enable_c2 = Output(UInt(1.W))
  val in = Input(UInt(1.W))
  // val busy = Output(Bool())
}

// trait MMTopIO extends Bundle {
//   val mm_busy = Output(Bool())
// }

trait HasMMIO extends BaseModule {
  val w: Int
  val io = IO(new MMIO(w))
}

// DOC include start: GCD blackbox
class MMMMIOBlackBox(
  val w: Int
) extends BlackBox(Map("WIDTH" -> IntParam(w))) 
  with HasBlackBoxResource
  with HasMMIO
{
  addResource("/vsrc/addr_sel.v")
  addResource("/vsrc/blackbox.v")
  addResource("/vsrc/quantize.v")
  addResource("/vsrc/systolic_controll.v")
  addResource("/vsrc/systolic.v")
  addResource("/vsrc/tpu_top.v")
  addResource("/vsrc/write_out.v")
}
// DOC include end: GCD blackbox

// DOC include start: GCD instance regmap

trait MMModule extends HasRegMap {
  // val io: MMTopIO

  implicit val p: Parameters
  def params: MMParams
  val clock: Clock
  val reset: Reset


  // How many clock cycles in a PWM cycle?
  // val rdata_a0 = Wire(new DecoupledIO(UInt(params.width_data.W)))
  val rdata_a0 = Reg(UInt(params.width_data.W))
  val rdata_a1 = Reg(UInt(params.width_data.W))
  val rdata_b0 = Reg(UInt(params.width_data.W))
  val rdata_b1 = Reg(UInt(params.width_data.W))
  // val raddr_a0 = Wire(new DecoupledIO(UInt(params.width_addr.W)))
  val raddr_a0 = Wire(UInt(params.width_addr.W))
  val raddr_a1 = Wire(UInt(params.width_addr.W))
  val raddr_b0 = Wire(UInt(params.width_addr.W))
  val raddr_b1 = Wire(UInt(params.width_addr.W))
  val wdata_c00 = Wire(UInt(params.width_data_c.W))
  val wdata_c01 = Wire(UInt(params.width_data_c.W))
  val wdata_c10 = Wire(UInt(params.width_data_c.W))
  val wdata_c11 = Wire(UInt(params.width_data_c.W))
  val wdata_c20 = Wire(UInt(params.width_data_c.W))
  val wdata_c21 = Wire(UInt(params.width_data_c.W))
  val waddr_c0 = Wire(UInt(params.width_addr_c.W))
  val waddr_c1 = Wire(UInt(params.width_addr_c.W))
  val waddr_c2 = Wire(UInt(params.width_addr_c.W))
  val enable_c0 = Wire(UInt(1.W))
  val enable_c1 = Wire(UInt(1.W))
  val enable_c2 = Wire(UInt(1.W))
  val status = Wire(UInt(2.W))
  val in = Wire(new DecoupledIO(UInt(1.W)))

  val impl = Module(new MMMMIOBlackBox(params.width))
  // val impl = Module(new MMMMIOBlackBox(params.width, params.width_addr, params.width_data, params.width_addr_c, params.width_data_c))

  impl.io.clock := clock
  impl.io.reset := reset.asBool

  impl.io.sram_rdata_a0 := rdata_a0
  impl.io.sram_rdata_a1 := rdata_a1
  impl.io.sram_rdata_b0 := rdata_b0
  impl.io.sram_rdata_b1 := rdata_b1
  // raddr_a0.valid := impl.io.output_valid
  // impl.io.output_ready:= raddr_a0.ready
  // raddr_a0.bits := impl.io.sram_raddr_a0
  impl.io.sram_raddr_a0 := raddr_a0
  impl.io.sram_raddr_a1 := raddr_a1
  impl.io.sram_raddr_b0 := raddr_b0
  impl.io.sram_raddr_b1 := raddr_b1
  wdata_c00 := impl.io.sram_wdata_c00
  wdata_c01 := impl.io.sram_wdata_c01
  wdata_c10 := impl.io.sram_wdata_c10
  wdata_c11 := impl.io.sram_wdata_c11
  wdata_c20 := impl.io.sram_wdata_c20
  wdata_c21 := impl.io.sram_wdata_c21
  waddr_c0 := impl.io.sram_waddr_c0
  waddr_c1 := impl.io.sram_waddr_c1
  waddr_c2 := impl.io.sram_waddr_c2
  enable_c0 := impl.io.sram_write_enable_c0
  enable_c1 := impl.io.sram_write_enable_c1
  enable_c2 := impl.io.sram_write_enable_c2
  impl.io.input_valid := in.valid
  in.ready := impl.io.input_ready
  impl.io.in := in.bits

  // impl.io.b := b.bits
  // impl.io.input_valid := b.valid
  // b.ready := impl.io.input_ready

  // res.bits := impl.io.res
  // res.valid := impl.io.output_valid
  // impl.io.output_ready := res.ready

  status := Cat(impl.io.input_ready, impl.io.output_valid)
  // io.gcd_busy := impl.io.busy

  regmap(
    // 0x00 -> Seq(
    //   RegField.r(2, status)), // a read-only register capturing current status
    // 0x04 -> Seq(
    //   RegField.w(params.width, a)), // a plain, write-only register
    // 0x08 -> Seq(
    //   RegField.w(params.width, b)), // write-only, y.valid is set on write
    // 0x0C -> Seq(
    //   RegField.r(params.width, res))) // read-only, gcd.ready is set on read

    0x00 -> Seq(RegField.w(params.width_data, rdata_a0)),
    0x04 -> Seq(RegField.w(params.width_data, rdata_a1)),
    0x08 -> Seq(RegField.w(params.width_data, rdata_b0)),
    0x0C -> Seq(RegField.w(params.width_data, rdata_b1)),
    0x10 -> Seq(RegField.w(params.width_addr, raddr_a0)),
    0x12 -> Seq(RegField.w(params.width_addr, raddr_a1)),
    0x14 -> Seq(RegField.w(params.width_addr, raddr_b0)),
    0x16 -> Seq(RegField.w(params.width_addr, raddr_b1)),
    0x18 -> Seq(RegField.r(params.width_data_c, wdata_c00)),
    0x20 -> Seq(RegField.r(params.width_data_c, wdata_c01)),
    0x28 -> Seq(RegField.r(params.width_data_c, wdata_c10)),
    0x30 -> Seq(RegField.r(params.width_data_c, wdata_c11)),
    0x38 -> Seq(RegField.r(params.width_data_c, wdata_c20)),
    0x40 -> Seq(RegField.r(params.width_data_c, wdata_c21)),
    0x48 -> Seq(RegField.r(params.width_addr_c, waddr_c0)),
    0x49 -> Seq(RegField.r(params.width_addr_c, waddr_c1)),
    0x4A -> Seq(RegField.r(params.width_addr_c, waddr_c2)),
    0x4B -> Seq(RegField.r(1, enable_c0)),
    0x4C -> Seq(RegField.r(1, enable_c1)),
    0x4D -> Seq(RegField.r(1, enable_c2)),
    0x4E -> Seq(RegField.r(2, status)),
    0x4F -> Seq(RegField.w(1, in))
  )
}
// DOC include end: GCD instance regmap

// DOC include start: GCD router
class MMTL(params: MMParams, beatBytes: Int)(implicit p: Parameters)
  extends TLRegisterRouter(
    params.address, "mm", Seq("ucbbar,mm"),
    beatBytes = beatBytes)(
      new TLRegBundle(params, _))(
      // new TLRegBundle(params, _) with MMTopIO)(
      new TLRegModule(params, _, _) with MMModule)

class MMAXI4(params: MMParams, beatBytes: Int)(implicit p: Parameters)
  extends AXI4RegisterRouter(
    params.address,
    beatBytes=beatBytes)(
      new AXI4RegBundle(params, _))(
      // new AXI4RegBundle(params, _) with MMTopIO)(
      new AXI4RegModule(params, _, _) with MMModule)
// DOC include end: GCD router

// DOC include start: GCD lazy trait
trait CanHavePeripheryMM { this: BaseSubsystem =>
  private val portName = "mm"

  // Only build if we are using the TL (nonAXI4) version
  val mm_busy = p(MMKey) match {
    case Some(params) => {
      val mm = if (params.useAXI4) {
        val mm = pbus { LazyModule(new MMAXI4(params, pbus.beatBytes)(p)) }
        pbus.coupleTo(portName) {
          mm.node :=
          AXI4Buffer () :=
          TLToAXI4 () :=
          // toVariableWidthSlave doesn't use holdFirstDeny, which TLToAXI4() needsx
          TLFragmenter(pbus.beatBytes, pbus.blockBytes, holdFirstDeny = true) := _
        }
        mm
      } else {
        val mm = pbus { LazyModule(new MMTL(params, pbus.beatBytes)(p)) }
        pbus.coupleTo(portName) { mm.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
        mm
      }
      // val pbus_io = pbus { InModuleBody {
      //   val busy = IO(Output(Bool()))
      //   busy := gcd.module.io.gcd_busy
      //   busy
      // }}
      // val gcd_busy = InModuleBody {
      //   val busy = IO(Output(Bool())).suggestName("gcd_busy")
      //   busy := pbus_io
      //   busy
      // }
      // Some(gcd_busy)
    }
    case None => None
  }
}
// DOC include end: GCD lazy trait

// DOC include start: GCD config fragment
class WithMM(useAXI4: Boolean = false, useBlackBox: Boolean = false) extends Config((site, here, up) => {
  case MMKey => Some(MMParams(useAXI4 = useAXI4, useBlackBox = useBlackBox))
})
// DOC include end: GCD config fragment
