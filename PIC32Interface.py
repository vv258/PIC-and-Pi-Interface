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
        time.sleep(0.1)
        ser.write(bytearray.fromhex(hex_byte))
            
def WriteBuffer(BufNum, Data):
    DataLength=len(Data)
    DataLengthMSB,DataLengthLSB=divmod(DataLength,32)
    Command = [WriteBuf,BufNum,DataLengthMSB,DataLengthLSB]
    SendCommand(Command)

    for i in range(0,DataLength):
        hex_byte = ("{0:02x}".format(Data[i]))
        time.sleep(0.1)
        ser.write(bytearray.fromhex(hex_byte))
        
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
    return Data

