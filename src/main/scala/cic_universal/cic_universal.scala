// Finitie impulse filter
package cic_universal
import config._
import config.{cicConfig}

import java.io.File

import chisel3._
import chisel3.experimental.FixedPoint
import chisel3.util.{log2Ceil}
import chisel3.stage.{ChiselStage, ChiselGeneratorAnnotation}
import chisel3.stage.ChiselGeneratorAnnotation

import dsptools._
import dsptools.numbers.DspComplex


class cic_universalCTRL(val resolution : Int, val gainBits: Int) extends Bundle {
    val reset_clk = Input(Bool())    
    val cic_en_clkdiv = Input(Bool())
    val Ndiv = Input(UInt(8.W))    
    val convmode = Input(UInt(1.W))
    val scale = Input(UInt(gainBits.W))
}

class cic_universalIO(resolution: Int, gainBits: Int) extends Bundle {
    val control = new cic_universalCTRL(resolution, gainBits)
    val in = new Bundle {
        val iptr_A = Input(DspComplex(SInt(resolution.W), SInt(resolution.W)))
    }
    val out = new Bundle {
        val Z = Output(DspComplex(SInt(resolution.W), SInt(resolution.W)))
    }
}

class cic_universal(config: cicConfig) extends Module {
    val io = IO(new cic_universalIO(resolution=config.resolution, gainBits=config.gainBits))
    val data_reso = config.resolution
    val calc_reso = config.resolution * 2

    // clkdiv_n
    val clkdiv_n = withReset(io.control.reset_clk) {Module(new clkdiv_n(n = 8))}

    clkdiv_n.io.control.en   := io.control.cic_en_clkdiv
    clkdiv_n.io.control.Ndiv := io.control.Ndiv

    // Integrators
    val integ = Module(new Integ(config=config))

    integ.io.control.convmode   := io.control.convmode
    integ.io.control.scale      := io.control.scale

    // Comb
    val comb = withClock(clkdiv_n.io.out.clkpn.asClock) {
        Module(new Comb(config=config))
    }

    comb.io.control.convmode    := io.control.convmode
    comb.io.control.scale       := io.control.scale

    when (io.control.convmode.asBool) { 
        integ.io.in.iptr_A.real := io.in.iptr_A.real 
        integ.io.in.iptr_A.imag := io.in.iptr_A.imag 

        comb.io.in.iptr_A.real := integ.io.out.Z.real 
        comb.io.in.iptr_A.imag := integ.io.out.Z.imag

        io.out.Z.real := comb.io.out.Z.real 
        io.out.Z.imag := comb.io.out.Z.imag 
    } .otherwise {      
        comb.io.in.iptr_A.real := io.in.iptr_A.real 
        comb.io.in.iptr_A.imag := io.in.iptr_A.imag 

        integ.io.in.iptr_A.real := comb.io.out.Z.real 
        integ.io.in.iptr_A.imag := comb.io.out.Z.imag 

        io.out.Z.real := integ.io.out.Z.real
        io.out.Z.imag := integ.io.out.Z.imag
    }
}

class clkdiv_nCTRL() extends Bundle {
    val en = Input(Bool())    
    val Ndiv = Input(UInt(8.W))
}

class clkdiv_nIO() extends Bundle {
    val control = new clkdiv_nCTRL()
    val out = new Bundle {
        val clkpn = Output(Bool())
    }
}

class clkdiv_n (n: Int = 2) extends Module {
    val io = IO(new clkdiv_nIO())
    val en = Wire(Bool()) 
    val r_Ndiv = RegInit(0.U.asTypeOf(io.control.Ndiv))

    en := io.control.en
    r_Ndiv := io.control.Ndiv

    val stateregister = RegInit(false.B)
    val count = RegInit(0.U(n.W))

    when (en) {
        when (count === r_Ndiv - 1.U) {
            count := 0.U
            stateregister := true.B
        } .otherwise {
            count := count + 1.U(1.W)
            stateregister := false.B
        }
    }

    when (r_Ndiv - 1.U === 0.U) {
        io.out.clkpn := clock.asUInt
    } .otherwise {
        io.out.clkpn := RegNext(stateregister)
    }
}

class CombIntegCTRL(val resolution : Int, val gainBits: Int) extends Bundle {
    val convmode = Input(UInt(1.W))
    val scale = Input(UInt(gainBits.W))
}

class CombIO(resolution: Int, gainBits: Int) extends Bundle {
    val control = new CombIntegCTRL(resolution, gainBits)
    val in = new Bundle {
        val iptr_A = Input(DspComplex(SInt(resolution.W), SInt(resolution.W)))
    }
    val out = new Bundle {
        val Z = Output(DspComplex(SInt(resolution.W), SInt(resolution.W)))
    }
}

class Comb(config: cicConfig) extends Module {
    val data_reso = config.resolution
    val calc_reso = config.resolution * 2
    val io = IO(new CombIO(resolution=calc_reso, gainBits=config.gainBits))


    val slowregs = RegInit(VecInit(Seq.fill(config.order + 1)(DspComplex.wire(0.S(calc_reso.W), 0.S(calc_reso.W)))))
    val minusregs = RegInit(VecInit(Seq.fill(config.order + 1)(DspComplex.wire(0.S(calc_reso.W), 0.S(calc_reso.W)))))

    for (i <- 0 to config.order) {
        if (i <= 0) {
            when (io.control.convmode.asBool) {
                slowregs(i).real := io.in.iptr_A.real << io.control.scale
                slowregs(i).imag := io.in.iptr_A.imag << io.control.scale 
            } .otherwise {   
                slowregs(i).real := RegNext(io.in.iptr_A.real)
                slowregs(i).imag := RegNext(io.in.iptr_A.imag) 
            }
            minusregs(i) := slowregs(i)
        } else {
            slowregs(i).real := slowregs(i - 1).real - minusregs(i - 1).real
            slowregs(i).imag := slowregs(i - 1).imag - minusregs(i - 1).imag
            minusregs(i) := slowregs(i)
        }
    }

    when (io.control.convmode.asBool) {
        io.out.Z.real := slowregs(config.order).real(calc_reso - 1, calc_reso - data_reso).asSInt
        io.out.Z.imag := slowregs(config.order).imag(calc_reso - 1, calc_reso - data_reso).asSInt
    } .otherwise {   
        io.out.Z.real := slowregs(config.order).real
        io.out.Z.imag := slowregs(config.order).imag
    }
}

class IntegIO(resolution: Int, gainBits: Int) extends Bundle {
    val control = new CombIntegCTRL(resolution, gainBits)
    val in = new Bundle {
        val iptr_A = Input(DspComplex(SInt(resolution.W), SInt(resolution.W)))
    }
    val out = new Bundle {
        val Z = Output(DspComplex(SInt(resolution.W), SInt(resolution.W)))
    }
}

class Integ(config: cicConfig) extends Module {
    val data_reso = config.resolution
    val calc_reso = config.resolution * 2

    val io = IO(new IntegIO(resolution=calc_reso, gainBits=config.gainBits))

    //Integrators
    val integregs = RegInit(VecInit(Seq.fill(config.order + 1)(DspComplex.wire(0.S(calc_reso.W), 0.S(calc_reso.W)))))
    for (i <- 0 to config.order) {
        if (i <= 0) {
            when (io.control.convmode.asBool) {
                integregs(i).real := io.in.iptr_A.real
                integregs(i).imag := io.in.iptr_A.imag
            } .otherwise {   
                integregs(i).real := io.in.iptr_A.real << io.control.scale
                integregs(i).imag := io.in.iptr_A.imag << io.control.scale
            }
        } else {
            integregs(i).real := integregs(i - 1).real + integregs(i).real
            integregs(i).imag := integregs(i - 1).imag + integregs(i).imag
        }
    }

    when (io.control.convmode.asBool) {
        io.out.Z.real := integregs(config.order).real
        io.out.Z.imag := integregs(config.order).imag
    } .otherwise {   
        io.out.Z.real := RegNext(integregs(config.order).real(calc_reso - 1, calc_reso - data_reso).asSInt)
        io.out.Z.imag := RegNext(integregs(config.order).imag(calc_reso - 1, calc_reso - data_reso).asSInt)
    }
}



/** Generates verilog or sv*/
object cic_universal extends App with OptionParser {
  // Parse command-line arguments
  val (options, arguments) = getopts(default_opts, args.toList)
  printopts(options, arguments)

  val config_file = options("config_file")
  val target_dir = options("td")
  var cic_config: Option[cicConfig] = None
  cicConfig.loadFromFile(config_file) match {
    case Left(config) => {
      cic_config = Some(config)
    }
    case Right(err) => {
      System.err.println(s"\nCould not load FIR configuration from file:\n${err.msg}")
      System.exit(-1)
    }
  }

  // Generate verilog
  val annos = Seq(ChiselGeneratorAnnotation(() => new cic_universal(config=cic_config.get)))
  //(new ChiselStage).execute(arguments.toArray, annos)
  val sysverilog = (new ChiselStage).emitSystemVerilog(
    new cic_universal(config=cic_config.get),
     
    //args
    Array("--target-dir", target_dir))
}



/** Module-specific command-line option parser */
trait OptionParser {
  // Module specific command-line option flags
  val available_opts: List[String] = List(
      "-config_file",
      "-td"
  )

  // Default values for the command-line options
  val default_opts : Map[String, String] = Map(
    "config_file"->"cic-config.yml",
    "td"->"verilog/"
  )

  /** Recursively parse option flags from command line args
   * @param options Map of command line option names to their respective values.
   * @param arguments List of arguments to parse.
   * @return a tuple whose first element is the map of parsed options to their values 
   *         and the second element is the list of arguments that don't take any values.
   */
  def getopts(options: Map[String, String], arguments: List[String]) : (Map[String, String], List[String]) = {
    val usage = s"""
      |Usage: ${this.getClass.getName.replace("$","")} [-<option> <argument>]
      |
      | Options
      |     -config_file        [String]  : Generator YAML configuration file name. Default "fir-config.yml".
      |     -td                 [String]  : Target dir for building. Default "verilog/".
      |     -h                            : Show this help message.
      """.stripMargin

    // Parse next elements in argument list
    arguments match {
      case "-h" :: tail => {
        println(usage)
        sys.exit()
      }
      case option :: value :: tail if available_opts contains option => {
        val (newopts, newargs) = getopts(
            options ++ Map(option.replace("-","") -> value), tail
        )
        (newopts, newargs)
      }
      case argument :: tail => {
        val (newopts, newargs) = getopts(options, tail)
        (newopts, argument.toString +: newargs)
      }
      case Nil => (options, arguments)
    }
  }

  /** Print parsed options and arguments to stdout */
  def printopts(options: Map[String, String], arguments: List[String]) = {
    println("\nCommand line options:")
    options.nonEmpty match {
      case true => for ((k,v) <- options) {
        println(s"  $k = $v")
      }
      case _ => println("  None")
    }
    println("\nCommand line arguments:")
    arguments.nonEmpty match {
      case true => for (arg <- arguments) {
        println(s"  $arg")
      }
      case _ => println("  None")
    }
  }
}

