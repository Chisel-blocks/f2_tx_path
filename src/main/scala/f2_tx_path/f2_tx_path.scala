// See LICENSE for license details.
//
//Start with a static tb and try to genererate a gnerator for it
package f2_tx_path
import chisel3._
import chisel3.util._
import chisel3.experimental._
import dsptools._
import dsptools.numbers._
import freechips.rocketchip.util._
import f2_interpolator._
import prog_delay._

class tx_path_adc_ioctrl (
        val outputn   : Int=9,
        val n        : Int=16,
        val users    : Int=4,
        val progdelay: Int=64,
        val finedelay: Int=32
    ) extends Bundle {
        val tx_lut_mode  = UInt(3.W)
        val tx_lut_write_addr = UInt(outputn.W)
        val tx_lut_write_val  = DspComplex(SInt(outputn.W), SInt(outputn.W))
        val tx_lut_write_en   = Bool()
        val user_delays       = Vec(users,UInt(log2Ceil(progdelay).W))
        val fine_delays       = UInt(log2Ceil(finedelay).W)
}

// How to extract the "resolution" of a type?
class f2_tx_path_io (
        val outputn    : Int=9, 
        val n         : Int=16,  
        val users     : Int=4,
        val progdelay : Int=64,
        val finedelay : Int=32

    ) extends Bundle {
    val interpolator_clocks   = new f2_decimator_clocks()
    val interpolator_controls = new f2_decimator_controls(gainbits=10)
    val adc_clock             = Input(Clock())
    val adc_ioctrl            = Input(new tx_path_adc_ioctrl(outputn=outputn, n=n,
                                        users=users,progdelay=progdelay,
                                        finedelay=finedelay))
    val iptr_A             = Input(DspComplex(SInt(outputn.W), SInt(outputn.W)))
    val Z                  = Output(Vec(users,DspComplex(SInt(n.W), SInt(n.W))))
}

class f2_tx_path (
        outputn   : Int=9, 
        n         : Int=16, 
        users     : Int=4,
        progdelay : Int=64,
        finedelay : Int=32
    ) extends Module {
    val io = IO( new f2_tx_path_io(outputn=outputn,users=users,progdelay=progdelay))
  
    val decimator  = Module ( new  f2_decimator (n=n, resolution=32, coeffres=16, gainbits=10)).io
    io.decimator_controls<>decimator.controls
    io.decimator_clocks<>decimator.clocks
    val adcproto=DspComplex(SInt(n.W),SInt(n.W))  
    val adcfifodepth=16
    val adcfifo = Module (new AsyncQueue(adcproto,depth=adcfifodepth)).io
    adcfifo.enq_clock:=io.adc_clock
    adcfifo.enq.valid:=true.B
    adcfifo.enq_reset:=io.adc_ioctrl.reset_adcfifo
    adcfifo.deq_reset:=io.adc_ioctrl.reset_adcfifo
    adcfifo.deq_clock:=clock
    adcfifo.deq.ready:=true.B

    // Adc delay compensation
    val adcdelay=Module( new prog_delay(adcproto, maxdelay=finedelay)).io
    
    adcdelay.iptr_A:=adcfifo.deq.bits
    adcdelay.select<>io.adc_ioctrl.fine_delays

    //ADC lookup tables
    val adclut_real= Mem(scala.math.pow(2,9).toInt,SInt(outputn.W))
    val adclut_imag= Mem(scala.math.pow(2,9).toInt,SInt(outputn.W))
    val w_lutoutdata= RegInit(DspComplex.wire(0.S(outputn.W),0.S(outputn.W)))
    //val w_lutoutdata = Wire(DspComplex(SInt(outputn.W), SInt(outputn.W)))
    val w_lutreadaddress= RegInit(DspComplex.wire(0.S(outputn.W),0.S(outputn.W)))

    //Input selection wire
    val w_inselect = Wire(DspComplex(SInt(outputn.W), SInt(outputn.W)))

    when (io.adc_ioctrl.adc_fifo_lut_mode===0.U) {
        //Bypass FIFO and LUT
        w_inselect:=io.iptr_A
        adcfifo.enq.bits:= io.iptr_A
        w_lutreadaddress.real:= io.adc_ioctrl.adc_lut_write_addr.asSInt
        w_lutreadaddress.imag:= io.adc_ioctrl.adc_lut_write_addr.asSInt
    } .elsewhen (io.adc_ioctrl.adc_fifo_lut_mode===1.U) {
        //LUT bypassed, FIFO active
        adcfifo.enq.bits:=io.iptr_A
        w_inselect:=adcdelay.optr_Z
    } .elsewhen (io.adc_ioctrl.adc_fifo_lut_mode===2.U) {
       //FIFO active, LUt active
       adcfifo.enq.bits:=io.iptr_A
       w_lutreadaddress:=adcdelay.optr_Z
       w_inselect:=w_lutoutdata
    } .elsewhen (io.adc_ioctrl.adc_fifo_lut_mode===3.U) {
       //FIFO active, LUT active, LUT first
       //Sync problem assumed
       w_lutreadaddress:=io.iptr_A
       adcfifo.enq.bits:=w_lutoutdata
       w_inselect:=adcdelay.optr_Z
    } .elsewhen (io.adc_ioctrl.adc_fifo_lut_mode===4.U) {
       //LUT active, FIFO bypassed
       //Sync problem assumed
       adcfifo.enq.bits:= io.iptr_A
       w_lutreadaddress:=io.iptr_A
       w_inselect:=w_lutoutdata
    } .otherwise {
       adcfifo.enq.bits:= io.iptr_A
       w_inselect:=adcdelay.optr_Z
    }
    
    //Enabled read
    when (io.adc_ioctrl.adc_lut_write_en===true.B) {
        adclut_real.write(io.adc_ioctrl.adc_lut_write_addr,io.adc_ioctrl.adc_lut_write_val.real)
        adclut_imag.write(io.adc_ioctrl.adc_lut_write_addr,io.adc_ioctrl.adc_lut_write_val.imag)
    } 
    .otherwise {
        w_lutoutdata.real:=adclut_real.read(w_lutreadaddress.real.asUInt)
        w_lutoutdata.imag:=adclut_imag.read(w_lutreadaddress.imag.asUInt)
    }
    //RX input assignments        
    decimator.iptr_A:=w_inselect

    //Gives a possibility to tune each user separately
    //even they are currently only one data stream
    val userdelay= Seq.fill(users){ 
        withClock(io.decimator_clocks.hb3clock_low)(
            Module( new prog_delay(adcproto, maxdelay=progdelay)).io
        )
    }
    
    userdelay.map(_.iptr_A:=decimator.Z)
    (userdelay,io.adc_ioctrl.user_delays).zipped.map(_.select:=_)

    when (io.decimator_controls.mode===0.U) {
        io.Z.map(_:=RegNext(decimator.Z))
    } .otherwise {
        //These are in the same clock domain if
        //the decimator in NOT bypassed
        (io.Z,userdelay).zipped.map(_:=_.optr_Z)
    }
}
//This gives you verilog
object f2_tx_path extends App {
  chisel3.Driver.execute(args, () => new f2_tx_path)
}

