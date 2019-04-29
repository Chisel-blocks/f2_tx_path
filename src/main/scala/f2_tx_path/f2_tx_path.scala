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
import clkmux._

class segmented (val bin: Int=4, val thermo: Int=5)  extends Bundle {
    val b =UInt(bin.W)
    val t =UInt((scala.math.pow(2,thermo).toInt-1).W)
}

class segmented_bool (val bin: Int=4, val thermo: Int=5)  extends Bundle {
    val b =Vec(bin,Bool())
    val t =Vec(scala.math.pow(2,thermo).toInt-1,Bool())
}

class dac_io (val bin: Int=4, val thermo: Int=5) extends Bundle {
    val real=new segmented(bin=bin,thermo=thermo) 
    val imag=new segmented(bin=bin,thermo=thermo)
}
class dac_io_bool (val bin: Int=4, val thermo: Int=5) extends Bundle {
    val real=new segmented_bool(bin=bin,thermo=thermo) 
    val imag=new segmented_bool(bin=bin,thermo=thermo)
}

class tx_path_dsp_ioctrl (
        val outputn   : Int=9,
        val bin       : Int=4,
        val thermo    : Int=5,
        val n         : Int=16,
        val users     : Int=4,
        val weightbits: Int=10,
        val progdelay : Int=63,
        val finedelay : Int=31

    ) extends Bundle {
        val dac_data_mode      = UInt(3.W)
        val dac_lut_write_addr = UInt(outputn.W)
        val dac_lut_write_val  = DspComplex(SInt(outputn.W), SInt(outputn.W))
        val dac_lut_write_en   = Bool()
        val user_delays        = Vec(users,UInt(log2Ceil(progdelay).W))
        val user_weights       = Vec(users,DspComplex(SInt(weightbits.W),SInt(weightbits.W)))
        val user_sum_mode      = Bool()
        val user_select_index  = UInt(log2Ceil(users).W)
        val fine_delays        = UInt(log2Ceil(finedelay).W)
        val reset_dacfifo      = Bool()
}

// Clock is fast, and the fast clock is the interpolators master clock
class f2_tx_path_io (
        val outputn   : Int=9, 
        val bin       : Int=4,
        val thermo    : Int=5,
        val n         : Int=16,  
        val users     : Int=4,
        val progdelay : Int=63,
        val finedelay : Int=31,
        val resolution: Int=32,
        val weightbits: Int=10
    ) extends Bundle {
    val interpolator_clocks   = new f2_interpolator_clocks()
    val interpolator_controls = new f2_interpolator_controls(resolution=resolution,gainbits=10)
    val dsp_ioctrl            = Input(new tx_path_dsp_ioctrl(outputn=outputn, n=n,
                                        users=users,progdelay=progdelay,
                                        finedelay=finedelay,weightbits=weightbits))
    val dac_clock     = Input(Clock())
    val clock_symrate = Input(Clock()) 
    val iptr_A        = Input(Vec(users,DspComplex(SInt(n.W), SInt(n.W))))
    val bypass_input  = Input(Vec(users,DspComplex(SInt(n.W), SInt(n.W))))
    val bypass_Ndiv   = Input(UInt(8.W)) //Division at maximum with nuber of users
                                         // Consistent with the clockdivider
    val bypass_clock  = Input(Clock())  //Clock for bypass mode
    val Z             = Output(new dac_io(bin=bin,thermo=thermo)) 
}

class f2_tx_path (
        thermo    : Int=5,
        bin       : Int=4,
        outputn   : Int=9, 
        n         : Int=16, 
        resolution: Int=32, 
        users     : Int=4,
        progdelay : Int=63,
        finedelay : Int=31,
        weightbits: Int=10
    ) extends Module {
    val io = IO( new f2_tx_path_io(thermo=thermo,bin=bin,
            outputn=outputn,users=users,progdelay=progdelay))
    val sigproto= DspComplex(SInt(n.W), SInt(n.W))
    val sigzero=0.U.asTypeOf(sigproto) 
    //Gives a possibility to tune each user separately
    //even they are currently only one data stream
    val userdelay= withClock(io.clock_symrate){
        VecInit(Seq.fill(users){ 
                Module( new prog_delay(sigproto, maxdelay=progdelay)).io
            }
        )
    }
    (userdelay,io.iptr_A).zipped.map(_.iptr_A:=_)
    (userdelay,io.dsp_ioctrl.user_delays).zipped.map(_.select:=_)
    //Add some modes here if needed
    
    // Must extend the widths by 1 before multiplication
    val weighted_users=Seq.fill(users){
        withClock(io.clock_symrate){
            Reg(DspComplex(SInt(n.W), SInt(n.W)))
        }
    }
    val w_weights=Seq.fill(users){
        // sync the static Io to symrate)
        withClock(io.clock_symrate){
            Reg(DspComplex(
                FixedPoint((weightbits+1).W,(weightbits-1).BP),
                FixedPoint((weightbits+1).W,(weightbits-1).BP)
            ))
        }
    }
    val w_delays=Seq.fill(users){
        // Sum of fractional bits must eventually equal weightbits after multiplication
        // Already synced
        Wire(DspComplex(FixedPoint((n+1).W,1.BP),FixedPoint((n+1).W,1.BP)))
    }

    val w_weighted_users=Seq.fill(users){
        // Sum of fractional bits must eventually equal weightbits after multiplication
        Wire(DspComplex(
            FixedPoint((n+weightbits).W,(weightbits).BP),
            FixedPoint((n+weightbits).W,(weightbits).BP)
        ))
    }
    for (i <- 0 until users) {
        w_weights(i).real:=io.dsp_ioctrl.user_weights(i).real
            .asFixedPoint((weightbits-1).BP)
        w_weights(i).imag:=io.dsp_ioctrl.user_weights(i).imag
            .asFixedPoint((weightbits-1).BP)
        w_delays(i).real:=userdelay(i).optr_Z.real.asFixedPoint(1.BP)
        w_delays(i).imag:=userdelay(i).optr_Z.imag.asFixedPoint(1.BP)
        w_weighted_users(i):= w_delays(i) * w_weights(i) 
        weighted_users(i).real:=w_weighted_users(i).real.floor.asSInt 
        weighted_users(i).imag:=w_weighted_users(i).imag.floor.asSInt 

    }
    
    //Sum the user data. TODO: add the real signal processing here
    val userssum=Wire(sigproto)
    when { io.dsp_ioctrl.user_sum_mode===true.B } { 
        userssum:=weighted_users.foldLeft(sigzero)((acc,i) => i+acc)
    }.otherwise {
        userssum:=weighted_users(io.dsp_ioctrl.user_select_index)
    }
     
     //Then interpolate
     val interpolator_reset=Wire(Bool())
     val interpolator  = withReset(interpolator_reset)(
         Module ( 
             new  f2_interpolator (
                 n=n, resolution=resolution, coeffres=16, gainbits=10)
         ).io
     )
     io.interpolator_controls<>interpolator.controls
     io.interpolator_clocks<>interpolator.clocks
     interpolator.iptr_A:=userssum
     //DAC delay compensation
     val dacdelay=withClock(io.interpolator_clocks.cic3clockfast){ 
         Module( new prog_delay(sigproto, maxdelay=finedelay)).io
     }
     dacdelay.iptr_A:=interpolator.Z
     dacdelay.select<>io.dsp_ioctrl.fine_delays

     ////DAC lookup tables
     val daclut_real= withClock(io.interpolator_clocks.cic3clockfast){
         SyncReadMem(scala.math.pow(2,outputn).toInt,SInt(outputn.W))
     }
     val daclut_imag= withClock(io.interpolator_clocks.cic3clockfast){
         SyncReadMem(scala.math.pow(2,outputn).toInt,SInt(outputn.W))
     }
     val r_lutoutdata= withClock(io.interpolator_clocks.cic3clockfast){ 
         RegInit(DspComplex.wire(0.S(outputn.W),0.S(outputn.W)))
     }
     
     val r_lutreadaddress=withClock(io.interpolator_clocks.cic3clockfast){ 
         RegInit(DspComplex.wire(0.S(outputn.W),0.S(outputn.W)))
     }
     r_lutreadaddress.real:=dacdelay.optr_Z.real(n-1,n-outputn).asSInt
     r_lutreadaddress.imag:=dacdelay.optr_Z.imag(n-1,n-outputn).asSInt
     
     //Enabled read, sync to fast clock
     when (
         withClock(io.interpolator_clocks.cic3clockfast){
             RegNext(io.dsp_ioctrl.dac_lut_write_en)}===true.B
          ) {
         daclut_real.write(
             io.dsp_ioctrl.dac_lut_write_addr,
             io.dsp_ioctrl.dac_lut_write_val.real
         )
         daclut_imag.write(
             io.dsp_ioctrl.dac_lut_write_addr,
             io.dsp_ioctrl.dac_lut_write_val.imag
         )
     } 
     .otherwise {
         r_lutoutdata.real:=withClock(io.interpolator_clocks.cic3clockfast){
             daclut_real.read(r_lutreadaddress.real.asUInt)
         }
         r_lutoutdata.imag:=withClock(io.interpolator_clocks.cic3clockfast){
             daclut_imag.read(r_lutreadaddress.imag.asUInt)
         }
     }

    //TX input assignments        
    //Output selection wire
    val w_outselect = Wire(sigproto)

    //Default assignment
    interpolator_reset:=reset.toBool()
    w_outselect:=r_lutoutdata

    // Bypass methodology
    // Must sync the counter to bypass clock
    val reset_bypass_register=withClock(io.bypass_clock){Reg(Bool())}
    reset_bypass_register:=true.B
    val bypass_serdes_register=withClockAndReset(
            io.bypass_clock,reset_bypass_register
        ){
            RegInit(0.U.asTypeOf(io.bypass_input))
        }
    val bypass_serdes_serreg=withReset(reset_bypass_register){
            RegInit(0.U.asTypeOf(w_outselect))
        }

    val bypass_counter=withReset(reset_bypass_register){
        RegInit(0.U.asTypeOf(io.bypass_Ndiv))
    }

    //Count the index for the bypass mode cant exceed users
    when ( bypass_counter < users) {
        when ( bypass_counter < io.bypass_Ndiv-1.U) {
            bypass_counter:=bypass_counter+1.U
        } .otherwise {
            bypass_counter:=0.U
        }
    } .otherwise {
        bypass_counter:=0.U
    }

    //Modes asynchronous, NO need to sync false path 
    when (io.dsp_ioctrl.dac_data_mode===0.U){
        //mode 0 bypasses the interpolator
        reset_bypass_register:=false.B
        interpolator_reset:=true.B
        bypass_serdes_register:=io.bypass_input
        bypass_serdes_serreg:=bypass_serdes_register(bypass_counter)
        w_outselect:=bypass_serdes_serreg
    } .elsewhen (io.dsp_ioctrl.dac_data_mode===1.U){
        w_outselect:=userdelay(io.dsp_ioctrl.user_select_index).optr_Z
    }.elsewhen (io.dsp_ioctrl.dac_data_mode===2.U){
        w_outselect:=weighted_users(io.dsp_ioctrl.user_select_index)
    }.elsewhen (io.dsp_ioctrl.dac_data_mode===3.U){
        w_outselect:=userssum
    }.elsewhen (io.dsp_ioctrl.dac_data_mode===4.U){
        w_outselect:=interpolator.Z
    }.elsewhen (io.dsp_ioctrl.dac_data_mode===5.U){
        w_outselect:=dacdelay.optr_Z
    }.elsewhen (io.dsp_ioctrl.dac_data_mode===6.U){
        w_outselect:=r_lutoutdata
    }

     val realthermoind=Wire(UInt(thermo.W))
     val imagthermoind=Wire(UInt(thermo.W))
     realthermoind:=w_outselect.real(thermo+bin-1,bin)
     imagthermoind:=w_outselect.imag(thermo+bin-1,bin)
     val w_segmented=Wire(new dac_io_bool(bin=bin,thermo=thermo))
     w_segmented.real.b:=w_outselect.real(bin-1,0).toBools
     w_segmented.imag.b:=w_outselect.imag(bin-1,0).toBools

     for (i <- 0 to w_segmented.real.t.getWidth-1){
        when(i.asUInt <  realthermoind ) {
            w_segmented.real.t(i):=true.B
        }.otherwise {
            w_segmented.real.t(i):=false.B
        }
        when(  i.asUInt <  imagthermoind) {
            w_segmented.imag.t(i):=true.B
        }.otherwise {
            w_segmented.imag.t(i):=false.B
        }
     }
     //Fifo for careless IO
     val dacfifodepth=16
     val fifoproto=new dac_io(bin=bin,thermo=thermo)
     val dacfifo = Module (new AsyncQueue(fifoproto,depth=dacfifodepth)).io
     dacfifo.enq_clock:=clock
     dacfifo.enq.valid:=true.B
     dacfifo.enq_reset:=io.dsp_ioctrl.reset_dacfifo
     dacfifo.enq.bits.real.t:=w_segmented.real.t.asUInt
     dacfifo.enq.bits.real.b:=w_segmented.real.b.asUInt
     dacfifo.enq.bits.imag.t:=w_segmented.imag.t.asUInt
     dacfifo.enq.bits.imag.b:=w_segmented.imag.b.asUInt
     dacfifo.deq_reset:=io.dsp_ioctrl.reset_dacfifo
     dacfifo.deq_clock:=io.dac_clock
     dacfifo.deq.ready:=true.B
    
     io.Z:=dacfifo.deq.bits
    
}
//This gives you verilog
object f2_tx_path extends App {
  chisel3.Driver.execute(args, () => new f2_tx_path)
}

