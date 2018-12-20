import PIC32Interface
import time

PIC32Interface.SetPWMPeriod(100)
#time.sleep(0.1)
PIC32Interface.EnablePWM1(20)
PIC32Interface.EnablePWM2(50)

PIC32Interface.SetDACA(2000)
PIC32Interface.SetDACB(2000)
