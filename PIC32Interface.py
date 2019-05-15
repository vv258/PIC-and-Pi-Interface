import serial
import binascii
import time

ComBaudRate     = 115200
StartOfTransmit = 0xF0
EndOfTransmit   = 0xD7

Handshake       = 0x1A
ReadInput       = 0x2A
WriteInput      = 0x2B
DACSetA         = 0x3A
DACSetB         = 0x3B
CheckBuf        = 0x4A
SetSampFreq     = 0x4B
StartADC        = 0x4C
SetPWMPer       = 0x5A
StartPWM1       = 0x5B
StartPWM2       = 0x5C
ReadBuf         = 0x6A
WriteBuf        = 0x6B
ConfigDACA      = 0x3C
ConfigDACB      = 0x3D
ConfigDACAB     = 0x3E
StartDAC        = 0x3F
StopDAC         = 0x39


ser = serial.Serial( "/dev/ttyAMA0", 
    baudrate=ComBaudRate, 
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    #timeout=5, # IMPORTANT, can be lower or higher
   )
def CheckSum(Command):
    CheckSumValue=0x0
    for i in range(0,len(Command)):
        CheckSumValue=CheckSumValue+Command[i]
    return CheckSumValue

def SendCommand(Command):
    TransArray=[StartOfTransmit]
    CmdLength=len(Command)
    for i in range(0,CmdLength):
        TransArray.append(Command[i])
    TransArray.append(CheckSum(Command))
    TransArray.append(EndOfTransmit)
    for cmd_byte in  TransArray:
        hex_byte = ("{0:02x}".format(cmd_byte))
     #   print(hex_byte)
        ser.write(bytearray.fromhex(hex_byte))


            
def WriteBuffer(BufNum, Data):
    DataLength=len(Data)
    DataLengthMSB,DataLengthLSB=divmod(DataLength,32)
    Command = [WriteBuf,BufNum,DataLengthMSB,DataLengthLSB]
    SendCommand(Command)
    #time.sleep(0.1)
    data_bytes=ser.read(3)

    for i in range(0,DataLength):
        hex_byte = ("{0:02x}".format(Data[i]))
       # time.sleep(0.04)
      #  print(hex_byte)
        ser.write(bytearray.fromhex(hex_byte))
        if((i%8)==0):
            ser.read(1)
            


                        
def ReadBuffer(BufNum, DataLength):
    DataLengthMSB,DataLengthLSB=divmod(DataLength,32)
    Command = [ReadBuf,BufNum,DataLengthMSB,DataLengthLSB]
    Data = list()
    SendCommand(Command)
    data_bytes=ser.read(3)

    data_bytes=ser.read(DataLength)
    data_bytes_hex=data_bytes.encode('hex')
    resp_bytes = bytearray.fromhex(data_bytes_hex)
    for data_byte in resp_bytes:
        Data.append(data_byte)    

    return Data

def ReadBuffer2(BufNum, DataLength):
    DataLengthMSB,DataLengthLSB=divmod(2*DataLength,32)
    Command = [ReadBuf,BufNum,DataLengthMSB,DataLengthLSB]
    Data = list()
    Data2 = list()

    SendCommand(Command)
    data_bytes=ser.read(3)
    data_bytes=ser.read(2*DataLength)
    data_bytes_hex=data_bytes.encode('hex')
    resp_bytes = bytearray.fromhex(data_bytes_hex)
    for data_byte in resp_bytes:
        Data.append(data_byte)
    for i in range(0,DataLength-1):
        Data2.append(Data[2*i]+Data[2*i+1]*256)
    return Data2


def SetPWMPeriod(Period,unit):
    PeriodMSB,PeriodLSB=divmod(Period,32)
    Command =[SetPWMPer,PeriodMSB,PeriodLSB,unit]
    SendCommand(Command)
    data_bytes=ser.read(3)



def EnablePWM1(OnTime,unit):
    OntimeMSB,OnTimeLSB=divmod(OnTime,32)
    Command =[StartPWM1,OntimeMSB,OnTimeLSB,unit]
    SendCommand(Command)
    data_bytes=ser.read(3)


    
def EnablePWM2(OnTime,unit):
    OntimeMSB,OnTimeLSB=divmod(OnTime,32)
    Command =[StartPWM2,OntimeMSB,OnTimeLSB,unit]
    SendCommand(Command)
    data_bytes=ser.read(3)


    
def SetDACA(DacVal):
    DacValMSB,DacValLSB=divmod(DacVal,64)
    Command =[DACSetA,DacValMSB,DacValLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)


    
def SetDACB(DacVal):
    DacValMSB,DacValLSB=divmod(DacVal,64)
    Command =[DACSetB,DacValMSB,DacValLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)


        
def ConfigureDACA(BufNum, Mode):
    Command =[ConfigDACA,BufNum,Mode]
    SendCommand(Command)
    data_bytes=ser.read(3)

def ConfigureDACB(BufNum, Mode):
    Command =[ConfigDACB,BufNum,Mode]
    SendCommand(Command)
    data_bytes=ser.read(3)
 
    
def ConfigureDACAB(BufNumA,BufNumB, Mode):
    BufNum=BufNumA*4+BufNumB
    Command =[ConfigDACAB,BufNum,Mode]
    SendCommand(Command)
    data_bytes=ser.read(3)

    
def StartDACOutput(SampleFreq, Samples):
    SamplesMSB,SamplesLSB=divmod(Samples,32)
    Command =[StartDAC,SampleFreq,SamplesMSB,SamplesLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)

    
def StopDACOutput():
    Command =[StopDAC]
    SendCommand(Command)
    data_bytes=ser.read(3)

    
def CheckBufferStatus():
    Command = [CheckBuf]
    SendCommand(Command)
    data_bytes=ser.read(4)
    data_bytes_hex=data_bytes.encode('hex')
    resp_bytes = bytearray.fromhex(data_bytes_hex)
    status=resp_bytes[2];
    return status


def _StartADC_( Channel,Buffer ):
    ChannelAndBuffer =  Channel*4 + Buffer
    Command = [StartADC,ChannelAndBuffer]
    SendCommand(Command)
    data_bytes=ser.read(3)



def _SetSampleFreq_( PrescalerSetting, SampleVal ):
    SampleValMSB,SampleValLSB = divmod( SampleVal, 64 )
    Command = [ SetSampFreq, PrescalerSetting, SampleValMSB,SampleValLSB ]
    SendCommand(Command)
    data_bytes=ser.read(3)


def WriteGPIO(data):
    DataValMSB,DataValLSB = divmod( data, 16 )
    Command = [ WriteInput, DataValMSB, DataValLSB ]
    SendCommand(Command)
    data_bytes=ser.read(3)



def ReadGPIO():
    Command = [ ReadInput ]
    SendCommand(Command)
    data_bytes=ser.read(5)
    data_bytes_hex=data_bytes.encode('hex')
    gpio_bytes = bytearray.fromhex(data_bytes_hex)
    ReadVal=gpio_bytes[2]*16+gpio_bytes[3];

    return ReadVal


