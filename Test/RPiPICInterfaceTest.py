import PIC32Interface
import time
sleeptime=0.4
WriteData= [0xA,0xB,0xC,0xD,0xE]
PIC32Interface.WriteBuffer(0,WriteData)
time.sleep(sleeptime)
WriteData= [0x1,0x2,0x3,0x4,0x6]
PIC32Interface.WriteBuffer(1,WriteData)
time.sleep(sleeptime)
WriteData= [0x1,0x3,0x5,0x7,0x5]
PIC32Interface.WriteBuffer(2,WriteData)
time.sleep(sleeptime)
WriteData= [0x4,0x2,0x1,0x3,0x4]
PIC32Interface.WriteBuffer(3,WriteData)
time.sleep(sleeptime)
data =list()
for i in range(1,255):
    data.append(i)
ReadData=PIC32Interface.ReadBuffer(0,5)
print(ReadData)
ReadData=PIC32Interface.ReadBuffer(1,4)
print(ReadData)
ReadData=PIC32Interface.ReadBuffer(2,4)
print(ReadData)
ReadData=PIC32Interface.ReadBuffer(3,4)
print(ReadData)
data =list()
for i in range(1,6):
    data.append(1)
PIC32Interface.WriteBuffer(0,data)
time.sleep(sleeptime)
ReadData=PIC32Interface.ReadBuffer(0,i)
print(ReadData)