package subsystem.rme

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLBundleA
import freechips.rocketchip.regmapper._
import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy.BufferParams.flow





case class CtrlUnitColExtractorIO(inMaxID: Int, outMaxID: Int, nExtractionDesc : Int = 16) extends Bundle {
    val data = UInt(512.W) // take in an entire cache line
    val position = UInt(log2Ceil(64).W)
    val extractionDescriptors = Vec(nExtractionDesc, new ExtractionDescriptor(4))
    val extractionDescriptorsValid = Vec(nExtractionDesc, Bool())
    val descriptorIn = new RequestDescriptor(inMaxID, outMaxID)
}


case class ColumnExtractorIO(inMaxID:Int, outmaxID : Int, dataRegWidth : Int, nExtractionDesc: Int = 16, minDataSize: Int = 4) extends Bundle {

    //val DescriptorIn = Input(RequestDescriptor(inMaxID, outmaxID))
   // val DataSizeOut = Output(UInt(7.W)) // size in bytes
    val CtrlUnit = Flipped(DecoupledIO(new CtrlUnitColExtractorIO(inMaxID, outmaxID)))
    val Packer = DecoupledIO(PackerColExtractIO(inMaxID, outmaxID, dataRegWidth, minDataSize, nExtractionDesc))

    
}




class ColumnExtractor(params: RelMemParams, inMaxID : Int, outmaxID : Int, nExtractionDesc: Int = 16) extends Module {
    /*
        We will shift in a cache line and extract the needed parts 
    */


    val beatWidth = 8
    val dataRegWidth = (math.pow(2, params.maxDataSize+1)).toInt * beatWidth // this should give us the extra byte we need to extract excesses



    val io = IO(new ColumnExtractorIO(inMaxID, outmaxID, dataRegWidth))
    io.Packer.valid := false.B
    io.Packer.bits := 0.U.asTypeOf(new PackerColExtractIO(inMaxID, outmaxID, dataRegWidth, 4, nExtractionDesc))

    val tmpLine = RegInit(0.U(512.W))
    //val tmpDescriptor = Reg(new RequestDescriptor(inMaxID, outmaxID))
    val tmpWire = WireInit(0.U(dataRegWidth.W))
    val hasValidLine = RegInit(false.B)
    val descriptors = RegInit(
    VecInit(Seq.fill(nExtractionDesc)(
        0.U.asTypeOf(new ExtractionDescriptor(4))
    )))

    val descriptorsValid = RegInit(
        VecInit(Seq.fill(nExtractionDesc)(
        false.B)
    ))


    val descriptorCount = RegInit(0.U(log2Ceil(nExtractionDesc+1).W))
    val descriptor = Reg(new RequestDescriptor(inMaxID, outmaxID))

    when(io.CtrlUnit.fire) {
        tmpLine         := io.CtrlUnit.bits.data
        descriptors     := io.CtrlUnit.bits.extractionDescriptors
        descriptorsValid := io.CtrlUnit.bits.extractionDescriptorsValid
        descriptor      := io.CtrlUnit.bits.descriptorIn
    }
    // We should be able to extract everything at once.
    // And just mask when >= descriptorCount
    //def ActiveDescriptor() : ExtractionDescriptor = {
    //    descriptors(descriptorCount-1.U)
    //}

    when(io.CtrlUnit.fire) {
        SynthesizePrintf(
            "[Colxtractor] CTRL FIRE inputValidMask=0x%x\n",
            io.CtrlUnit.bits.extractionDescriptorsValid.asUInt,
        )

    }

    when(io.Packer.fire) {

    //    SynthesizePrintf(
    //    "[ColExtractor] PACKER FIRE regMask=0x%x outputMask=0x%x size=%d\n",
    //    descriptorsValid.asUInt,
    //    io.Packer.bits.dataInValid.asUInt,
    //    io.Packer.bits.dataSize
    //)
}


    /*
        We can extract everything in parallel
    */

    val validPacker = RegInit(false.B)
    validPacker :=  Mux(io.CtrlUnit.fire, true.B, Mux(io.Packer.fire, false.B, validPacker))
    io.Packer.valid := validPacker
    io.CtrlUnit.ready := !validPacker
    (0 until nExtractionDesc).foreach { i =>
        val edescriptor = descriptors(i)
        val result = Wire(UInt(64.W)) // max = 8 bytes
        result := 0.U

        val byteOffset = Wire(UInt(7.W))
        byteOffset := edescriptor.start
        val dataSize = descriptor.size

        val shifted = tmpLine >> (byteOffset << 3)




        switch(dataSize) {
            is(0.U) { 
                io.Packer.bits.dataVecIn(i) := shifted(7, 0)
                io.Packer.bits.dataInValid(i) := descriptorsValid(i) 
            
            }      // 1 byte
            is(1.U) { 
                
                io.Packer.bits.dataVecIn(i) := shifted(15, 0) 
                io.Packer.bits.dataInValid(i) := descriptorsValid(i)
            }     // 2 bytes
            is(2.U) { 
                io.Packer.bits.dataVecIn(i) := shifted(31, 0)
                io.Packer.bits.dataInValid(i) := descriptorsValid(i)
            }     // 4 bytes
            is(3.U) { 
                io.Packer.bits.dataVecIn(i) := shifted(63, 0)
                io.Packer.bits.dataInValid(i) := descriptorsValid(i)
            }     // 8 bytes
            is(4.U) {           
                    if (i < 4)
                    {                                          // 16 bytes
                      io.Packer.bits.dataVecIn(i*2) := shifted(63, 0)
                      io.Packer.bits.dataVecIn(i*2+1) := shifted(127, 64)
                      io.Packer.bits.dataInValid(i*2) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(i*2+1) := descriptorsValid(i)
                    }
            }
            is(5.U) {            // 32 bytes
                    if (i < 2)
                    {
                      io.Packer.bits.dataVecIn(i*4)     := shifted(63, 0)
                      io.Packer.bits.dataVecIn(i*4+1)   := shifted(127, 64)
                      io.Packer.bits.dataVecIn(i*4+2)   := shifted(191, 128)
                      io.Packer.bits.dataVecIn(i*4+3)   := shifted(255, 192)
                      io.Packer.bits.dataInValid(i*4) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(i*4+1) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(i*4+2) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(i*4+3) := descriptorsValid(i)
                    }                                         
            }
            is(6.U) {       
                                                              // 64 bytes
                    if (i  == 0)
                    {
                      io.Packer.bits.dataVecIn(0)   := shifted(63, 0)
                      io.Packer.bits.dataVecIn(1)   := shifted(127, 64)
                      io.Packer.bits.dataVecIn(2)   := shifted(191, 128)
                      io.Packer.bits.dataVecIn(3)   := shifted(255, 192)
                      io.Packer.bits.dataVecIn(4)   := shifted(319, 256)
                      io.Packer.bits.dataVecIn(5)   := shifted(383, 320)
                      io.Packer.bits.dataVecIn(6)   := shifted(447, 384)
                      io.Packer.bits.dataVecIn(7)   := shifted(511, 448)
                      io.Packer.bits.dataInValid(0) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(1) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(2) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(3) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(4) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(5) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(6) := descriptorsValid(i)
                      io.Packer.bits.dataInValid(7) := descriptorsValid(i)
                    }
            }  
        }



        when (io.Packer.fire)
        {
           // SynthesizePrintf("(ColExtract%d) data 0x%x\n", i.U, result)
        }
    

    }




    // we split up larger data sizes larger than 8
    io.Packer.bits.dataSize :=
        Mux(descriptor.size > 3.U, 3.U, descriptor.size) 
        

    io.Packer.bits.descriptorIn  := descriptor
   
}