import sys
sys.path.append('../')
import PIC32Interface
import time
import matplotlib.pyplot as plt

#sleeptime=0.4
#data =list()
#for i in range(1,128):
#    data.append(i)
#PIC32Interface.WriteBuffer(0,data)
#ReadData=PIC32Interface.ReadBuffer(0,i)
#print(ReadData)

ReadData=PIC32Interface.ReadGPIO()
print(ReadData)

PIC32Interface.WriteGPIO(0)

#ReadData=PIC32Interface.ReadBuffer(0,200)

#print(ReadData)
PIC32Interface._SetSampleFreq_( 1, 200 )
#time.sleep(10)

PIC32Interface._StartADC_( 1,3 )
time.sleep(1)
while(not((PIC32Interface.CheckBufferStatus())and 0x08) ):
    pass
ReadData=PIC32Interface.ReadBuffer2(3,200)
#print(ReadData)

plotdata=[3.3*(float(x)/float(1024)) for x in ReadData]
t=range(0,len(ReadData))
#print t
plottime=[float(x) / 10 for x in t]
#print plottime
plt.plot(plottime,plotdata)
plt.show()
#PIC32Interface.ConfigureDACAB(0,0,1)

#PIC32Interface.StartDACOutput(10,200)
print(ReadData)