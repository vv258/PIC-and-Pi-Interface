import sys
sys.path.append('../')
import PIC32Interface
import time
sleeptime=0.4
data =list()
for i in range(1,128):
    data.append(i)
PIC32Interface.WriteBuffer(0,data)
ReadData=PIC32Interface.ReadBuffer(0,i)
print(ReadData)