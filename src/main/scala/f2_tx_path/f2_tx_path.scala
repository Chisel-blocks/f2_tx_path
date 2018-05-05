// See LICENSE for license details.
//
//Start with a static tb and try to genererate a gnerator for it
package f2_tx_path
import chisel3._
import chisel3.util._
import chisel3.experimental._
import dsptools._
import dsptools.numbers._
// Should I Use this or not? 
//import dsptools.numbers.implicits._
import freechips.rocketchip.util._
import f2_interpolator._
import prog_delay._

class tx_path_dsp_ioctrl (
        val outputn   : Int=9,
        val n         : Int=16,
        val users     : Int=4,
        val weightbits: Int=10,
        val progdelay : Int=64,
        val finedelay : Int=32

    ) extends Bundle {
        val dac_lut_mode       = UInt(3.W)
        val dac_lut_write_addr = UInt(outputn.W)
        val dac_lut_write_val  = DspComplex(SInt(outputn.W), SInt(outputn.W))
        val dac_lut_write_en   = Bool()
        val user_delays       = Vec(users,UInt(log2Ceil(progdelay).W))
        val user_weights      = Vec(users,DspComplex(SInt(weightbits.W),SInt(weightbits.W)))
        val fine_delays       = UInt(log2Ceil(finedelay).W)
}

// How to extract the "resolution" of a type?
class f2_tx_path_io (
        val outputn    : Int=9, 
        val n         : Int=16,  
        val users     : Int=4,
        val progdelay : Int=64,
        val finedelay : Int=32,
        val resolution: Int=32

    ) extends Bundle {
    val interpolator_clocks   = new f2_interpolator_clocks()
    val interpolator_controls = new f2_interpolator_controls(gainbits=10)
    val dsp_ioctrl            = Input(new tx_path_dsp_ioctrl(outputn=outputn, n=n,
                                        users=users,progdelay=progdelay,
                                        finedelay=finedelay))
    val iptr_A        = Input(Vec(users,DspComplex(SInt(n.W), SInt(n.W))))
    val Z             = Output(DspComplex(SInt(outputn.W), SInt(outputn.W)))
}

class f2_tx_path (
        outputn   : Int=9, 
        n         : Int=16, 
        users     : Int=4,
        progdelay : Int=64,
        finedelay : Int=32
    ) extends Module {
    val io = IO( new f2_tx_path_io(outputn=outputn,users=users,progdelay=progdelay))
    val sigproto= DspComplex(SInt(n.W), SInt(n.W))
    val sigzero=0.U.asTypeOf(sigproto) 
    
    //Gives a possibility to tune each user separately
    //even they are currently only one data stream
    val userdelay= Seq.fill(users){ 
        Module( new prog_delay(sigproto, maxdelay=progdelay)).io
    }
    (userdelay,io.iptr_A).zipped.map(_.iptr_A:=_)
    (userdelay,io.dsp_ioctrl.user_delays).zipped.map(_.select:=_)
    //Add some modes here if needed

    //val weighted_users=Wire(asTypeOf(io.user_weights))
    val weighted_users=Reg(Vec(users,DspComplex(SInt(n.W), SInt(n.W))))
    ( weighted_users, 
        ( userdelay,io.dsp_ioctrl.user_weights
        ).zipped.map( _.optr_Z * _)
    ).zipped.map(_:=_)
    
    //Sum the user data. TODO: add the real signal processing here
    val userssum=Wire(sigproto)
    
    userssum:=weighted_users.foldLeft(sigzero)((acc,i) => i+acc)
  
    //Then interpolate
    val interpolator  = Module ( new  f2_interpolator (n=n, resolution=32, coeffres=16, gainbits=10)).io
    io.interpolator_controls<>interpolator.controls
    io.interpolator_clocks<>interpolator.clocks
    interpolator.iptr_A:=userssum
    //DAC delay compensation
    val dacdelay=Module( new prog_delay(sigproto, maxdelay=finedelay)).io
    dacdelay.iptr_A:=interpolator.Z
    dacdelay.select<>io.dsp_ioctrl.fine_delays


    //Input selection wire
    //val w_inselect = Wire(DspComplex(SInt(outputn.W), SInt(outputn.W)))
    //
    ////coarse delay->interpolator-finedelay->lut
    //
    //  

    ////DAC lookup tables
    val daclut_real= SyncReadMem(scala.math.pow(2,outputn).toInt,SInt(outputn.W))
    val daclut_imag= SyncReadMem(scala.math.pow(2,outputn).toInt,SInt(outputn.W))
    val w_lutoutdata= RegInit(DspComplex.wire(0.S(outputn.W),0.S(outputn.W)))
    val w_lutreadaddress= RegInit(DspComplex.wire(0.S(outputn.W),0.S(outputn.W)))

    
    //Enabled read
    when (io.dsp_ioctrl.dac_lut_write_en===true.B) {
        daclut_real.write(io.dsp_ioctrl.dac_lut_write_addr,io.dsp_ioctrl.dac_lut_write_val.real)
        daclut_imag.write(io.dsp_ioctrl.dac_lut_write_addr,io.dsp_ioctrl.dac_lut_write_val.imag)
    } 
    .otherwise {
        w_lutoutdata.real:=daclut_real.read(w_lutreadaddress.real.asUInt)
        w_lutoutdata.imag:=daclut_imag.read(w_lutreadaddress.imag.asUInt)
    }
    //RX input assignments        

    //when (io.interpolator_controls.mode===0.U) {
    //    io.Z.map(_:=RegNext(interpolator.Z))
    //} .otherwise {
    //    //These are in the same clock domain if
    //    //the interpolator in NOT bypassed
        io.Z.real:=daclut_real.read(interpolator.Z.real.asUInt)
        io.Z.imag:=daclut_imag.read(interpolator.Z.imag.asUInt)
    //}
}
//This gives you verilog
object f2_tx_path extends App {
  chisel3.Driver.execute(args, () => new f2_tx_path)
}
