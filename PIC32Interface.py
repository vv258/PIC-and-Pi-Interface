import serial
import binascii
import time

ComBaudRate     = 9600
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



ser = serial.Serial( "/dev/tty.SLAB_USBtoUART", ComBaudRate )

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
        time.sleep(0.04)
        print(hex_byte)
        ser.write(bytearray.fromhex(hex_byte))
            
def WriteBuffer(BufNum, Data):
    DataLength=len(Data)
    DataLengthMSB,DataLengthLSB=divmod(DataLength,32)
    Command = [WriteBuf,BufNum,DataLengthMSB,DataLengthLSB]
    SendCommand(Command)
    #time.sleep(0.1)

    for i in range(0,DataLength):
        hex_byte = ("{0:02x}".format(Data[i]))
        time.sleep(0.04)
        print(hex_byte)
        ser.write(bytearray.fromhex(hex_byte))
    
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

                        
def ReadBuffer(BufNum, DataLength):
    DataLengthMSB,DataLengthLSB=divmod(DataLength,32)
    Command = [ReadBuf,BufNum,DataLengthMSB,DataLengthLSB]
    Data = list()
    SendCommand(Command)
    data_bytes=ser.read(DataLength)
    data_bytes_hex=data_bytes.encode('hex')
    resp_bytes = bytearray.fromhex(data_bytes_hex)
    for data_byte in resp_bytes:
        Data.append(data_byte)    
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)
    return Data

def ReadBuffer2(BufNum, DataLength):
    DataLengthMSB,DataLengthLSB=divmod(2*DataLength,32)
    Command = [ReadBuf,BufNum,DataLengthMSB,DataLengthLSB]
    Data = list()
    Data2 = list()

    SendCommand(Command)
    data_bytes=ser.read(2*DataLength)
    data_bytes_hex=data_bytes.encode('hex')
    resp_bytes = bytearray.fromhex(data_bytes_hex)
    for data_byte in resp_bytes:
        Data.append(data_byte)
    for i in range(0,DataLength-1):
        Data2.append(Data[2*i]+Data[2*i+1]*256)
    return Data2
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

def SetPWMPeriod(Period):
    PeriodMSB,PeriodLSB=divmod(Period,32)
    Command =[SetPWMPer,PeriodMSB,PeriodLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)


def EnablePWM1(OnTime):
    OntimeMSB,OnTimeLSB=divmod(OnTime,32)
    Command =[StartPWM1,OntimeMSB,OnTimeLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

    
def EnablePWM2(OnTime):
    OntimeMSB,OnTimeLSB=divmod(OnTime,32)
    Command =[StartPWM2,OntimeMSB,OnTimeLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

    
def SetDACA(DacVal):
    DacValMSB,DacValLSB=divmod(DacVal,64)
    Command =[DACSetA,DacValMSB,DacValLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

    
def SetDACB(DacVal):
    DacValMSB,DacValLSB=divmod(DacVal,64)
    Command =[DACSetB,DacValMSB,DacValLSB]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

    
def CheckBufferStatus():
    Command = [CheckBuf]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)


def _StartADC_( Channel,Buffer ):
    ChannelAndBuffer =  Channel*4 + Buffer
    Command = [StartADC,ChannelAndBuffer]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)


def _SetSampleFreq_( PrescalerSetting, SampleVal ):
    SampleValMSB,SampleValLSB = divmod( SampleVal, 64 )
    Command = [ SetSampFreq, PrescalerSetting, SampleValMSB,SampleValLSB ]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)

def WriteGPIO(data):
    DataValMSB,DataValLSB = divmod( data, 16 )
    Command = [ WriteInput, DataValMSB, DataValLSB ]
    SendCommand(Command)
    data_bytes=ser.read(3)
    print("resp")
    print(data_bytes)


def ReadGPIO():
    Command = [ ReadInput ]
    SendCommand(Command)
    data_bytes=ser.read(5)
    data_bytes_hex=data_bytes.encode('hex')
    gpio_bytes = bytearray.fromhex(data_bytes_hex)
    ReadVal=gpio_bytes[2]*16+gpio_bytes[3];
    print("resp")
    print(data_bytes)
    return ReadVal
