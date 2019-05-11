import sys
sys.path.append('../')
import PIC32Interface
import time


PIC32Interface.WriteGPIO(2)

ReadData=PIC32Interface.ReadGPIO()
print(ReadData)


