import sys
sys.path.append('../')
import PIC32Interface
import time
import math

PIC32Interface.SetDACA(3003)
PIC32Interface.SetDACB(3000)


data1 =list()
for j in range(0,16):
    for i in range(0,16):
        data1.append(i*16)
        data1.append(j)
        
for j in range(15,-1,-1):
    for i in range(15,-1,-1):
        data1.append(i*16)
        data1.append(j)
#print(data1)

PIC32Interface.WriteBuffer(0,data1)
time.sleep(1)
#ReadData=PIC32Interface.ReadBuffer(0,512)
#print(ReadData)
data3 =list()
data2 =list()
      
#for j in range(15,-1,-1):
#    for i in range(15,-1,-1):
#        data2.append(i*16)
#        data2.append(j)
#
#for j in range(0,16):
#    for i in range(0,16):
#        data2.append(i*16)
#        data2.append(j)

#sin_table[i] = (int)(2047*sin((float)i*6.283/(float)sine_table_size));
for i in range(0,512):
    data3.append(2047*math.sin(i*2*math.pi/512)+2047)

for i in range(0,512):
    MSB,LSB=divmod(int(data3[i]),256)
    data2.append(LSB)
    data2.append(MSB)        




#print(data2)

PIC32Interface.WriteBuffer(1,data2)
time.sleep(1)

#ReadData=PIC32Interface.ReadBuffer(1,512)
#print(ReadData)
#time.sleep(1)
#
#print(ReadData)
PIC32Interface.ConfigureDACAB(0,1,1)

PIC32Interface.StartDACOutput(10,512)