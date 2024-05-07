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
  address: BigInt = 0x1000,
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
  val in = Input(UInt(1.W))
  val input_ready = Output(Bool())
  val input_valid = Input(Bool())
  val output_ready = Input(Bool())
  val output_valid = Output(Bool())
  val sram_rdata_a0 = Input(UInt(32.W))
  val sram_rdata_a1 = Input(UInt(32.W))
  val sram_rdata_b0 = Input(UInt(32.W))
  val sram_rdata_b1 = Input(UInt(32.W))
  val sram_raddr = Input(UInt(10.W))
  // val sram_raddr_a1 = Input(UInt(10.W))
  // val sram_raddr_b0 = Input(UInt(10.W))
  // val sram_raddr_b1 = Input(UInt(10.W))
  val c00 = Output(UInt(64.W))
  val c01 = Output(UInt(64.W))
  val c10 = Output(UInt(64.W))
  val c11 = Output(UInt(64.W))
  val c20 = Output(UInt(64.W))
  val c21 = Output(UInt(64.W))
  val c30 = Output(UInt(64.W))
  val c31 = Output(UInt(64.W))
  val c40 = Output(UInt(64.W))
  val c41 = Output(UInt(64.W))
  val c50 = Output(UInt(64.W))
  val c51 = Output(UInt(64.W))
  val c60 = Output(UInt(64.W))
  val c61 = Output(UInt(64.W))
  val c70 = Output(UInt(64.W))
  val c71 = Output(UInt(64.W))

  val sram_num = Input(UInt(10.W))
  
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
  val rdata_a0 = Reg(UInt(params.width_data.W))
  val rdata_a1 = Reg(UInt(params.width_data.W))
  val rdata_b0 = Reg(UInt(params.width_data.W))
  val rdata_b1 = Reg(UInt(params.width_data.W))
  val raddr = Reg(UInt(params.width_addr.W))
  // val raddr_a1 = Reg(UInt(params.width_addr.W))
  // val raddr_b0 = Reg(UInt(params.width_addr.W))
  // val raddr_b1 = Reg(UInt(params.width_addr.W))
  val in = Wire(new DecoupledIO(UInt(1.W)))

  val c00 = Wire(UInt(params.width_data_c.W))
  val c01 = Wire(UInt(params.width_data_c.W))
  val c10 = Wire(UInt(params.width_data_c.W))
  val c11 = Wire(UInt(params.width_data_c.W))
  val c20 = Wire(UInt(params.width_data_c.W))
  val c21 = Wire(UInt(params.width_data_c.W))
  val c30 = Wire(UInt(params.width_data_c.W))
  val c31 = Wire(UInt(params.width_data_c.W))
  val c40 = Wire(UInt(params.width_data_c.W))
  val c41 = Wire(UInt(params.width_data_c.W))
  val c50 = Wire(UInt(params.width_data_c.W))
  val c51 = Wire(UInt(params.width_data_c.W))
  val c60 = Wire(UInt(params.width_data_c.W))
  val c61 = Wire(UInt(params.width_data_c.W))
  val c70 = Wire(UInt(params.width_data_c.W))
  val c71 = Wire(new DecoupledIO(UInt(params.width_data_c.W)))
  val status = Wire(UInt(2.W))


  val num = Reg(UInt(params.width_addr.W))
  

  val impl = Module(new MMMMIOBlackBox(params.width))
  // val impl = Module(new MMMMIOBlackBox(params.width, params.width_addr, params.width_data, params.width_addr_c, params.width_data_c))

  impl.io.clock := clock
  impl.io.reset := reset.asBool

  impl.io.sram_rdata_a0 := rdata_a0
  impl.io.sram_rdata_a1 := rdata_a1
  impl.io.sram_rdata_b0 := rdata_b0
  impl.io.sram_rdata_b1 := rdata_b1
  impl.io.sram_raddr := raddr
  impl.io.sram_num := num
  // impl.io.sram_raddr_a1 := raddr_a1
  // impl.io.sram_raddr_b0 := raddr_b0
  // impl.io.sram_raddr_b1 := raddr_b1
  impl.io.input_valid := in.valid
  in.ready := impl.io.input_ready
  impl.io.in := in.bits

  c00 := impl.io.c00
  c01 := impl.io.c01
  c10 := impl.io.c10
  c11 := impl.io.c11
  c20 := impl.io.c20
  c21 := impl.io.c21
  c30 := impl.io.c30
  c31 := impl.io.c31
  c40 := impl.io.c40
  c41 := impl.io.c41
  c50 := impl.io.c50
  c51 := impl.io.c51
  c60 := impl.io.c60
  c61 := impl.io.c61
  c70 := impl.io.c70
  c71.bits := impl.io.c71
  c71.valid := impl.io.output_valid
  impl.io.output_ready := c71.ready

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
    0x00 -> Seq(RegField.w(params.width_addr, num)),
    0x02 -> Seq(RegField.w(params.width_addr, raddr)),
    0x04 -> Seq(RegField.w(params.width_data, rdata_a0)),
    0x08 -> Seq(RegField.w(params.width_data, rdata_a1)),
    0x0C -> Seq(RegField.w(params.width_data, rdata_b0)),
    0x10 -> Seq(RegField.w(params.width_data, rdata_b1)),        

    // 0x12 -> Seq(RegField.w(params.width_addr, raddr_a1)),
    // 0x14 -> Seq(RegField.w(params.width_addr, raddr_b0)),
    // 0x16 -> Seq(RegField.w(params.width_addr, raddr_b1)),
    0x18 -> Seq(RegField.r(params.width_data_c, c00)),
    0x20 -> Seq(RegField.r(params.width_data_c, c01)),
    0x28 -> Seq(RegField.r(params.width_data_c, c10)),
    0x30 -> Seq(RegField.r(params.width_data_c, c11)),
    0x38 -> Seq(RegField.r(params.width_data_c, c20)),
    0x40 -> Seq(RegField.r(params.width_data_c, c21)),
    0x48 -> Seq(RegField.r(params.width_data_c, c30)),
    0x50 -> Seq(RegField.r(params.width_data_c, c31)),
    0x58 -> Seq(RegField.r(params.width_data_c, c40)),
    0x60 -> Seq(RegField.r(params.width_data_c, c41)),
    0x68 -> Seq(RegField.r(params.width_data_c, c50)),
    0x70 -> Seq(RegField.r(params.width_data_c, c51)),
    0x78 -> Seq(RegField.r(params.width_data_c, c60)),
    0x80 -> Seq(RegField.r(params.width_data_c, c61)),
    0x88 -> Seq(RegField.r(params.width_data_c, c70)),
    0x90 -> Seq(RegField.r(params.width_data_c, c71)),
    0x98 -> Seq(RegField.r(2, status)),
    0x99 -> Seq(RegField.w(1, in))
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
