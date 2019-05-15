

#ifndef CONFIG_H
#define	CONFIG_H
#define _SUPPRESS_PLIB_WARNING 
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include <plib.h>
// serial stuff
#include <stdio.h>
#include <string.h>
#include "port_expanderpicandpi.h"
//=============================================================
// 60 MHz
#pragma config FNOSC = FRCPLL, POSCMOD = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_2  //40 MHz
#pragma config FPBDIV = DIV_1 // PB 40 MHz
#pragma config FWDTEN = OFF,  JTAGEN = OFF
#pragma config FSOSCEN = OFF  //PINS 11 and 12 to secondary oscillator!
#pragma config DEBUG = OFF   // RB4 and RB5
//==============================================================

// set up clock parameters
// system cpu clock
#define sys_clock 40000000

// sys_clock/FPBDIV
#define pb_clock sys_clock/1 // divide by one in this case

#endif	/* CONFIG_H */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
// threading library
#include "pt_cornell_1_2_3.h"
// yup, the expander


////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// need for sin function
#include <math.h>

//**********define the frequencies and declare variables*****
#define	SYS_FREQ 40000000
#define ISR_FREQ 1000

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

char recv_char;

enum SerialState {ACTIVE_WAIT_FOR_SOT,ACTIVE_PARSE,ACTIVE_CHECKSUM,ACTIVE_WAIT_FOR_EOT};
enum SerialState RecvState;
enum ErrorStatus {CheckSumERR, InvalidCmdERR, NoERR };
enum ErrorStatus RecvStatus;

#define ComBaudRate     115200
#define DebugBaudRate   9600
#define StartOfTransmit 0xF0
#define EndOfTransmit   0xD7

#define Handshake       0x1A
#define ReadInput       0x2A
#define WriteInput      0x2B
#define DACSetA         0x3A
#define DACSetB         0x3B
#define ConfigDACA      0x3C
#define ConfigDACB      0x3D
#define ConfigDACAB     0x3E
#define StartDAC        0x3F
#define StopDAC         0x39
#define CheckBuf        0x4A
#define SetSampFreq     0x4B
#define StartADC        0x4C
#define SetPWMPer       0x5A
#define StartPWM1       0x5B
#define StartPWM2       0x5C
#define ReadBuf         0x6A
#define WriteBuf        0x6B
#define ByteAck         0x3A

static unsigned int sys_time_seconds =0;
static char cRecvData[10];
static char cTranData[10];
static char cbuffer[50];
static unsigned char cRecvChar;
static int iCheckSumValid=0;
static unsigned char GPIOIN;
static int iPWMScale=0;
static int timer1_count=0;
int iSampleCh[4],iSampleChBuf[4];
int iTotalNumOfADCSamples=0, iNumOfADCSamples[4];
int iTotalNumOfDACSamples=0;
static char tempbuf[2048];
static char iBuffer[4][2048];
short iDACBuf[2048];
int iDACmode=0;
char ReceiveBuf[256];
void debug_msg( char* print_buffer){

    int iNumSendChars = 0;
    while (print_buffer[iNumSendChars] != '\0'){
        while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, print_buffer[iNumSendChars]);
        iNumSendChars++;
    }
    
    iNumSendChars=0;
    while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, '\n');
        

}


void vExec_Read_Input(char *cRecvData){
   // tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Read Input",ReadInput);
    debug_msg(cbuffer);
    GPIOIN = readPE(GPIOZ);
    cTranData[0]= (0xF0 & GPIOIN)>>4;
    cTranData[1]= 0x0F & GPIOIN;

}

void vExec_Write_Input(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
   // tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Write Input",WriteInput);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Write Bits B7.B6.B5.B4 if 1",cMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  Write Bits B3.B2.B1.B0 if 1",cLSBbits);
    debug_msg(cbuffer);
    writePE(GPIOY, cRecvData[0]<<4 | cRecvData[1]);

    
}

void vExec_DAC_SetA(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
    int iDACValue;
   // tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set DAC Channel A",DACSetA);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  DAC MSB Bits B11.B10.B9.B8.B7.B6",cMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  DAC LSB Bits B5.B4.B3.B2.B1.B0",cLSBbits);
    debug_msg(cbuffer);
    iDACValue=((cMSBbits & 0x3f)<<6)|(cLSBbits & 0x3f);
    sprintf(cbuffer,"Set DAC A to %d", iDACValue);
    debug_msg(cbuffer);
    mPORTAClearBits(BIT_3); // start transaction
        delay_ms(100);
    WriteSPI2( DAC_config_chan_A | (iDACValue));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
                            // CS high
    mPORTASetBits(BIT_3); // end transaction
}
                        
void vExec_DAC_SetB(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
    int iDACValue;
   // tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set DAC Channel B",DACSetB);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  DAC MSB Bits B11.B10.B9.B8.B7.B6",cMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  DAC LSB Bits B5.B4.B3.B2.B1.B0",cLSBbits);
    debug_msg(cbuffer);
    iDACValue=((cMSBbits & 0x3f)<<6)|(cLSBbits & 0x3f);
    sprintf(cbuffer,"Set DAC B to %d", iDACValue);
    debug_msg(cbuffer);
    mPORTAClearBits(BIT_3); // start transaction
    
    delay_ms(100);
    WriteSPI2( DAC_config_chan_B | (iDACValue));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
                            // CS high
    mPORTASetBits(BIT_3); // end transaction
}
    
void vExec_Check_Buf(){
  //  tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Check Buffer Status",CheckBuf);
    debug_msg(cbuffer);
     cTranData[0]= (!iSampleCh[0])+((!iSampleCh[1])<<1)+((!iSampleCh[2])<<2)+((!iSampleCh[3])<<3);
}
                        
void vExec_Set_Samp_Freq(char *cRecvData){
    char cSampleFreq= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
  //  tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  Sample Frequency",SetSampFreq);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Sample Frequency %d Khz",cSampleFreq,cSampleFreq);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    debug_msg(cbuffer);
    iTotalNumOfADCSamples=((cSampleMSBbits & 0x3f)<<6)|(cSampleLSBbits & 0x3f);
    sprintf(cbuffer,"Set %dKhz Sample frequency and acquire %d samples", cSampleFreq,iTotalNumOfADCSamples);
    debug_msg(cbuffer);
   int timer_count=(sys_clock/(1000*cSampleFreq));
  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1,timer_count );
         ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2); 
    mT1ClearIntFlag(); // and clear the interrupt flag 
    
}
                        
void vExec_Start_ADC(char *cRecvData){
    char cChannel= cRecvData[0]>>2;
    char cBuffer=cRecvData[0]&0x03;
   // tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set Start ADC",StartADC);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Channel Number %d Buffer Numer %d",cRecvData[0],cChannel,cBuffer);
    debug_msg(cbuffer);
    iSampleCh[cChannel]=1;
    iSampleChBuf[cChannel]=cBuffer;
    iNumOfADCSamples[cChannel]=0;
 

}
                        
void vExec_Read_Buf(char *cRecvData){
    char cBufferNum= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    int iNumOfSamples;
    int iByteCount=0;
   // tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Read Buffer",ReadBuf);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Buffer Number %d",cBufferNum,cBufferNum);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    debug_msg(cbuffer);
    iNumOfSamples=((cSampleMSBbits & 0x1f)<<5)|(cSampleLSBbits & 0x1f);
    sprintf(cbuffer,"Read %d Samples from Buffer Number %d", iNumOfSamples,cBufferNum);
    debug_msg(cbuffer);
    

     for(iByteCount=0;iByteCount<iNumOfSamples;iByteCount++){
        while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, iBuffer[cBufferNum][iByteCount]);
      //UARTSendDataByte(UART2, tempbuf[iByteCount]);

    
     }
}

void vExec_Write_Buf(char *cRecvData){
    char cBufferNum= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    int iNumOfSamples;
    int iByteCount=0;
  //  tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Write Buffer",WriteBuf);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Buffer Number %d",cBufferNum,cBufferNum);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    debug_msg(cbuffer);
    iNumOfSamples=((cSampleMSBbits )<<5)|(cSampleLSBbits & 0x1f);
    sprintf(cbuffer,"Write %d Samples to Buffer Number %d", iNumOfSamples,cBufferNum);
    debug_msg(cbuffer);
   
    for(iByteCount=0;iByteCount<iNumOfSamples;iByteCount++){
            while(!UARTReceivedDataIsAvailable(UART2));
            iBuffer[cBufferNum][iByteCount]=UARTGetDataByte(UART2);
            if((iByteCount%8)==0)
                UARTSendDataByte(UART2, ByteAck);
       }

   

}

void vExec_Set_PWM_Per(char *cRecvData){
    char cPeriodMSBbits= cRecvData[0];
    char cPeriodLSBbits =cRecvData[1];
    char cPeriodunit =cRecvData[2];

     int iPeriod;
 //   tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  PWM Period",SetPWMPer);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Period MSB B11.B10.B9.B8.B7.B6",cPeriodMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  Period LSB Bits B5.B4.B3.B2.B1.B0",cPeriodLSBbits);
    debug_msg(cbuffer);
   sprintf(cbuffer,"Byte3  %02X  Unit",cPeriodunit);
    debug_msg(cbuffer);
    iPeriod=(((cPeriodMSBbits & 0x1f)<<5)|cPeriodLSBbits);
        iPeriod=iPeriod*(pow(10,cPeriodunit));
    sprintf(cbuffer,"Set PWM Period to %d us",iPeriod);
    debug_msg(cbuffer);
  //  delay_ms(100);
    CloseTimer2();
    if((iPeriod*40)<pow(2,16)){
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, iPeriod*40);
    iPWMScale=0;
    }
    
    else if((iPeriod*40)<pow(2,17)){
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_2, (iPeriod*40)>>1);
        iPWMScale=1;
    }
    else if((iPeriod*40)<pow(2,18)){
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_4, (iPeriod*40)>>2);
        iPWMScale=2;
    }
    else if((iPeriod*40)<pow(2,19)){
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_8, (iPeriod*40)>>3);
        iPWMScale=3;
    }
    else if((iPeriod*40)<pow(2,20)){
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_16, (iPeriod*40)>>4);
        iPWMScale=4;
    }
   // ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2); 
 //   mT2ClearIntFlag(); // and clear the interrupt flag 
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , 0, iPeriod); // 
    OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , 0, iPeriod); // 


} 
    


void vExec_Start_PWM1(char *cRecvData){
    char cOnTime1MSBbits= cRecvData[0];
    char cOnTime1LSBbits =cRecvData[1];
        char cPeriodunit =cRecvData[2];

    int iPWM1ONPeriod;
  //  tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  PWM 1",StartPWM1);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  On Time 1 MSB B11.B10.B9.B8.B7.B6",cOnTime1MSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  On Time 1 LSB Bits B5.B4.B3.B2.B1.B0",cOnTime1LSBbits);
    debug_msg(cbuffer);
       sprintf(cbuffer,"Byte3  %02X  Unit ",cPeriodunit);
    debug_msg(cbuffer);
    iPWM1ONPeriod=(((cOnTime1MSBbits & 0x1f)<<5)|cOnTime1LSBbits);
        iPWM1ONPeriod=((iPWM1ONPeriod*40)>>iPWMScale)*pow(10,cPeriodunit);
    sprintf(cbuffer,"Set PWM Period to %d us",iPWM1ONPeriod);
    debug_msg(cbuffer);
    SetDCOC1PWM(iPWM1ONPeriod);

}
    
void vExec_Start_PWM2(char *cRecvData){
    char cOnTime2MSBbits= cRecvData[0];
    char cOnTime2LSBbits =cRecvData[1];
    char cPeriodunit =cRecvData[2];
    int iPWM2ONPeriod;
 //   tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  PWM 2",StartPWM2);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  On Time 1 MSB B11.B10.B9.B8.B7.B6",cOnTime2MSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  On Time 1 LSB B5.B4.B3.B2.B1.B0",cOnTime2LSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte3  %02X  Unit",cPeriodunit);
    debug_msg(cbuffer);
    iPWM2ONPeriod=(((cOnTime2MSBbits & 0x1f)<<5)|cOnTime2LSBbits);
    iPWM2ONPeriod=((iPWM2ONPeriod*40)>>iPWMScale)*pow(10,cPeriodunit);
    sprintf(cbuffer,"Set PWM Period to %d us",iPWM2ONPeriod);
    debug_msg(cbuffer);
    SetDCOC4PWM(iPWM2ONPeriod); 
}



void vExec_DAC_ConfigA(char *cRecvData){
    char cBufferNum= cRecvData[0];
    iDACmode=cRecvData[1];
    sprintf(cbuffer,"Command    %02X    Set Config DACA",ConfigDACA);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Source Buffer Number %d",cRecvData[0],cBufferNum);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  DAC mode %d",cRecvData[1],cRecvData[1]);
    debug_msg(cbuffer);
    int *pDAC1= &(iBuffer[cBufferNum][0]);
    int i=0;
    CloseTimer3();
    DmaChnDisable(DMA_CHANNEL3);
    for(i=0;i<1024;i++){
        iDACBuf[i]=DAC_config_chan_A|*pDAC1;
        pDAC1++;
        
    }
}

void vExec_DAC_ConfigB(char *cRecvData){
    char cBufferNum= cRecvData[0];
    iDACmode=cRecvData[1];
    sprintf(cbuffer,"Command    %02X    Set Config DACB",ConfigDACB);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Source Buffer Number %d",cRecvData[0],cBufferNum);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  DAC mode %d",cRecvData[1],cRecvData[1]);
    debug_msg(cbuffer);
    CloseTimer3();
    DmaChnDisable(DMA_CHANNEL3);
    int *pDAC2= &(iBuffer[cBufferNum][0]);
    int i=0;
    for(i=0;i<1024;i++){
        iDACBuf[i]=DAC_config_chan_B|*pDAC2;
        pDAC2++;
        
    }
}  
void vExec_DAC_ConfigAB(char *cRecvData){
    char cBufferA= cRecvData[0]>>2;
    char cBufferB=cRecvData[0]&0x3;    
    iDACmode=cRecvData[1]+2;
    sprintf(cbuffer,"Command    %02X    Set Config DAC A and B",ConfigDACAB);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Source Buffer NumberA %d Buffer NumberB %d",cRecvData[0],cBufferA,cBufferB);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  DAC mode %d",cRecvData[1],cRecvData[1]);
    debug_msg(cbuffer);
    CloseTimer3();
    DmaChnDisable(DMA_CHANNEL3);
    short *pDAC1= &(iBuffer[cBufferA][0]);
    short *pDAC2= &(iBuffer[cBufferB][0]);
    int i=0;
    short *pDAC3;
    pDAC3=pDAC1;
    
    for(i=0;i<2048;i+=2){
        iDACBuf[i]=DAC_config_chan_A|*pDAC1;
        iDACBuf[i+1]=DAC_config_chan_B|*pDAC2;
        pDAC1++; 
        pDAC2++;
        
    }
    
  
}   
void vExec_DAC_Start(char *cRecvData){
    char cSampleFreq= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
  //  tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  Sample Frequency",StartDAC);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte1  %02X  Sample Frequency %d Khz",cSampleFreq,cSampleFreq);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    debug_msg(cbuffer);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    debug_msg(cbuffer);
    iTotalNumOfDACSamples=((cSampleMSBbits & 0x3f)<<5)|(cSampleLSBbits & 0x1f);
    sprintf(cbuffer,"Set %dKhz Sample frequency and generate %d samples", cSampleFreq,iTotalNumOfDACSamples);
    debug_msg(cbuffer);
    
    switch(iDACmode){
        case 0:     OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, SYS_FREQ/(cSampleFreq<<10));
                    DmaChnOpen(DMA_CHANNEL3, 0, DMA_OPEN_DEFAULT);
                    DmaChnSetTxfer(DMA_CHANNEL3,(short *) iDACBuf, &SPI2BUF, iTotalNumOfDACSamples*2, 2, 2 );
	                
                    break;
        case 1:     OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1,SYS_FREQ/(cSampleFreq<<10) );
                    DmaChnOpen(DMA_CHANNEL3, 0, DMA_OPEN_AUTO);
                    DmaChnSetTxfer(DMA_CHANNEL3, (short *)iDACBuf, &SPI2BUF, iTotalNumOfDACSamples*2, 2, 2 );
                    break;
        case 2:     OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1,SYS_FREQ/(cSampleFreq<<11) );
                    DmaChnOpen(DMA_CHANNEL3, 0, DMA_OPEN_DEFAULT);
                    DmaChnSetTxfer(DMA_CHANNEL3, (short *)iDACBuf, &SPI2BUF, iTotalNumOfDACSamples*4, 2, 2 );
                    break;
        case 3:     OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, SYS_FREQ/(cSampleFreq<<11));
                    DmaChnOpen(DMA_CHANNEL3, 0, DMA_OPEN_AUTO);
                    DmaChnSetTxfer(DMA_CHANNEL3, (short *)iDACBuf, &SPI2BUF, iTotalNumOfDACSamples*4, 2, 2 );
                    break;
    }
 
    DmaChnSetEventControl(DMA_CHANNEL3, DMA_EV_START_IRQ(_TIMER_3_IRQ));
    DmaChnEnable(DMA_CHANNEL3);
 //   ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2); 
   // mT3ClearIntFlag(); // and clear the interrupt flag 
    
}
    
void vExec_DAC_Stop(){
    sprintf(cbuffer,"Command    %02X    Stop DAC",StopDAC);
    debug_msg(cbuffer);
    CloseTimer3();
    DmaChnDisable(DMA_CHANNEL3);
}

int CheckSum(char *cRecvData, int iNumOfBytes, char iSum){
   
    return 1;
    
}

void vSendErrorMsg(char RecvStatus){
       
}




int iNumOfDataBytes(unsigned char cRecvChar){
     switch(cRecvChar){
                        
                        
                        case(ReadInput):    return 0;
                        
                        case(WriteInput):   return 2;
                        
                        case(DACSetA):      return 2;
                        
                        case(DACSetB):      return 2;
                        
                        case(ConfigDACA):   return 2;
                        
                        case(ConfigDACB):   return 2;
                        
                        case(ConfigDACAB):  return 2;
                        
                        case(StartDAC):     return 3;
                        
                        case(StopDAC):      return 0;
                        
                        case(CheckBuf):     return 0;
                        
                        case(SetSampFreq):  return 3;
                        
                        case(StartADC):     return 1;
                    
                        case(SetPWMPer):    return 3;
             
                        case(StartPWM1):    return 3;
             
                        case(StartPWM2):    return 3;
                        
                        case(ReadBuf):      return 3;
                        
                        case(WriteBuf):     return 3;

                        default:            return -1;
        }
    
}


int iNumOfRespBytes(unsigned char cRecvChar){
     switch(cRecvChar){
                        
                        
                        case(ReadInput):    return 2;
                        
                        case(WriteInput):   return 0;
                        
                        case(DACSetA):      return 0;
                        
                        case(DACSetB):      return 0;
                        
                        case(ConfigDACA):   return 0;
                        
                        case(ConfigDACB):   return 0;
                        
                        case(ConfigDACAB):  return 0;
                        
                        case(StartDAC):     return 0;
                        
                        case(StopDAC):      return 0;
                        
                        case(CheckBuf):     return 1;
                        
                        case(SetSampFreq):  return 0;
                        
                        case(StartADC):     return 0;
                    
                        case(SetPWMPer):    return 0;
             
                        case(StartPWM1):    return 0;
             
                        case(StartPWM2):    return 0;
                        
                        case(ReadBuf):      return 0;
                        
                        case(WriteBuf):     return 0;

                        default:            return -1;
        }
    
}

void vSendRespMsg(unsigned char cCommand,char *cTansData)
{   
    
    while(!UARTTransmitterIsReady(UART2));
    UARTSendDataByte(UART2,StartOfTransmit);
    
    while(!UARTTransmitterIsReady(UART2));
    UARTSendDataByte(UART2,cCommand);
    int iByte,iRespBytes;
    iRespBytes=iNumOfRespBytes(cCommand);
    for(iByte=0;iByte<iRespBytes;iByte++){
        while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2,cTansData[iByte]);
    }
    
    while(!UARTTransmitterIsReady(UART2));
    UARTSendDataByte(UART2,EndOfTransmit);
}
 void vExecuteCommand(unsigned char cCommand,char *cRecvData){
     switch(cCommand){
                        
                        case(ReadInput):    vExec_Read_Input(cRecvData);     break;
                        
                        case(WriteInput):   vExec_Write_Input(cRecvData);    break;
                        
                        case(DACSetA):      vExec_DAC_SetA(cRecvData);       break;
                        
                        case(DACSetB):      vExec_DAC_SetB(cRecvData);       break;
 
                        case(ConfigDACA):   vExec_DAC_ConfigA(cRecvData);    break;
                        
                        case(ConfigDACB):   vExec_DAC_ConfigB(cRecvData);    break;
                        
                        case(ConfigDACAB):  vExec_DAC_ConfigAB(cRecvData);   break;
                        
                        case(StartDAC):     vExec_DAC_Start(cRecvData);      break;
                        
                        case(StopDAC):      vExec_DAC_Stop(cRecvData);       break;

                        case(CheckBuf):     vExec_Check_Buf();               break;
                        
                        case(SetSampFreq):  vExec_Set_Samp_Freq(cRecvData);  break;
                        
                        case(StartADC):     vExec_Start_ADC(cRecvData);      break;
                        
                        case(SetPWMPer):    vExec_Set_PWM_Per(cRecvData);    break;
                        
                        case(StartPWM1):    vExec_Start_PWM1(cRecvData);     break;
             
                        case(StartPWM2):    vExec_Start_PWM2(cRecvData);     break;
                        
                        case(ReadBuf):      vExec_Read_Buf(cRecvData);       break;
                        
                        case(WriteBuf):     vExec_Write_Buf(cRecvData);      break;

        }

 }



void __ISR(_TIMER_1_VECTOR, ipl3) Timer1Handler(void){
mT1ClearIntFlag();  
mPORTBToggleBits(BIT_4);// 
 //AcquireADC10();
int iChannelCount, iSampleCount;

    for(iChannelCount=0;iChannelCount<4;iChannelCount++){
        if(iNumOfADCSamples[iChannelCount]==iTotalNumOfADCSamples)
            iSampleCh[iChannelCount]=0;
        else if(iSampleCh[iChannelCount]){
            iSampleCount=(iNumOfADCSamples[iChannelCount])++;
            short *buffer_ptr=(short *)(iBuffer[iSampleChBuf[iChannelCount]]);
             unsigned short adc_val=ReadADC10(iChannelCount);
            *(buffer_ptr+iSampleCount)=adc_val;
 
        }
        
    }

}

static struct pt pt_uart_receive, pt_timer;


static int iNumOfRecvBytes=0;
static int iByteCount=0;
static unsigned char cCommand;


static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
   
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;

    PT_END(pt);

}

void PIN_MANAGER_Initialize(void)
{
//PPSInput(	3	,	IC1	    ,	RPA2	);
PPSInput(	3	,	U1RX	,	RPA4	);
PPSInput(	2	,	U2RX	,	RPB5	);
//PPSInput(	3	,	SDI2	,	RPB13	);
//PPSInput(	4	,	IC2	    ,	RPA3	);
PPSOutput(	1	,	RPB7	,	U1TX	);
PPSOutput(	1	,	RPB3	,	OC1	    );
PPSOutput(	4	,	RPB9	,	SS2	    );
PPSOutput(	4	,	RPB10	,	U2TX	);
PPSOutput(	3	,	RPB2	,	OC4	    );
PPSOutput(	2	,	RPB11	,	SDO2	); 
    
//PPSInput(	2	,	SDI1	,	RPB8	);
//PPSOutput(	3	,	RPA2	,	SDO1	); 



//SCK2 -> RB15
//SCK1 -> RB14
//SS1 -> RB13


}

void Variable_Initialize(void){
    iSampleCh[0]=0;
    iSampleCh[1]=0;
    iSampleCh[2]=0;
    iSampleCh[3]=0;
    iNumOfADCSamples[0]=0;
    iNumOfADCSamples[1]=0;
    iNumOfADCSamples[2]=0;
    iNumOfADCSamples[3]=0;
    RecvState= ACTIVE_WAIT_FOR_SOT;  
 
}

// handler for the DMA channel 1 interrupt
void __ISR(_DMA1_VECTOR, IPL5SOFT) DmaHandler1(void)
{
    DmaChnDisable(DMA_CHANNEL1);
	INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL1));	// release the interrupt in the INT controller, we're servicing int

    int iByte=0;

    do{
            cRecvChar=ReceiveBuf[iByte++]; 
            if(RecvState==ACTIVE_WAIT_FOR_SOT && cRecvChar== 0xF0){
            RecvState=ACTIVE_PARSE;
            iCheckSumValid=0;
            RecvStatus=NoERR;
        }
        
        else if (RecvState==ACTIVE_WAIT_FOR_EOT && cRecvChar== EndOfTransmit){
                if(RecvStatus==NoERR){
                    if(cCommand==ReadBuf || cCommand==WriteBuf){
                          vSendRespMsg(cCommand,cTranData);
                          vExecuteCommand(cCommand,cRecvData);

                    }
                    else{
                        vExecuteCommand(cCommand,cRecvData);
                        vSendRespMsg(cCommand,cTranData);
                    }
                    }
                else
                    vSendErrorMsg(RecvStatus);
            
                RecvState=ACTIVE_WAIT_FOR_SOT;
        }
        
        else if (RecvState==ACTIVE_CHECKSUM){
                if(!CheckSum(cRecvData,iNumOfRecvBytes,cRecvChar))
                        RecvStatus=CheckSumERR;
               RecvState=ACTIVE_WAIT_FOR_EOT;
                }
        
        else if(RecvState==ACTIVE_PARSE){
                    cCommand=cRecvChar;
                    iNumOfRecvBytes=iNumOfDataBytes(cCommand);
                
                if(iNumOfRecvBytes ==-1)
                            RecvStatus=InvalidCmdERR;
                            
                else{
                    
                    for(iByteCount=0;iByteCount<iNumOfRecvBytes;iByteCount++){
                            cRecvData[iByteCount]=ReceiveBuf[iByte++]; 
                        }
                }
                RecvState=ACTIVE_CHECKSUM;
        } 
        
        
    }while(cRecvChar!= EndOfTransmit);
    
	DmaChnClrEvFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);
    INTEnable(INT_SOURCE_DMA(DMA_CHANNEL1), INT_ENABLED);		// enable the chn interrupt in the INT controller

	// enable the chn
	DmaChnEnable(DMA_CHANNEL1);
}


void main(){

  PT_setup();
  PIN_MANAGER_Initialize();
  Variable_Initialize();
    
  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
     //=====setup  UART 1 to debug========
  
  UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART1, pb_clock, DebugBaudRate);
  UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX)); 
  
    
    //=====setup  UART 2 to communicate with RPi========
  
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, pb_clock, ComBaudRate);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
   
  ANSELA =0;
  ANSELB =0; 
 // the ADC ///////////////////////////////////////

	// ---- configure and enable the ADC ----
 
	// ensure the ADC is off before setting the configuration
	CloseADC10();
 
	// define setup parameters for OpenADC10 
	//                 Turn module on | ouput in integer | trigger mode auto | enable autosample
	#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
 
	// ADC ref external	| disable offset test	| enable scan mode | perform 4 samples | use dual buffers | use only mux A
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF
 
	//               	use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
 
	//  set inputs to analog
	#define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA| ENABLE_AN3_ANA
 
	// only scan An0 to An3
	#define PARAM5  SKIP_SCAN_AN4 |SKIP_SCAN_AN5 |SKIP_SCAN_AN6 |SKIP_SCAN_AN7 | \
                 	SKIP_SCAN_AN8 |SKIP_SCAN_AN9 |SKIP_SCAN_AN10 |SKIP_SCAN_AN11 | \
                 	SKIP_SCAN_AN12 |SKIP_SCAN_AN13 |SKIP_SCAN_AN14 |SKIP_SCAN_AN15
   	
	// set negative reference to Vref for Mux A
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF );
 
	// open the ADC	
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );

EnableADC10(); // Enable the ADC
SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL, 4);    

//GPIO INIT
mPORTBSetPinsDigitalOut(BIT_4);
mPORTBSetBits(BIT_4);
 
//Port Expander
  initPE();
  // Outputs
  mPortYSetPinsOut(BIT_7 | BIT_6 | BIT_5 | BIT_4 |BIT_3 | BIT_2 |BIT_1 | BIT_0 );
  // Inputs
  mPortZSetPinsIn(BIT_7 | BIT_6 | BIT_5 | BIT_4 |BIT_3 | BIT_2 |BIT_1 | BIT_0 );
  // Input pull up resistors
  mPortZEnablePullUp(BIT_7 | BIT_6 | BIT_5 | BIT_4 |BIT_3 | BIT_2 |BIT_1 | BIT_0) ;

    
    
  PT_INIT(&pt_timer);
  PT_INIT(&pt_uart_receive);
  // init the display
  //tft_init_hw();
  //tft_begin();
  //tft_fillScreen(ILI9340_BLACK);
  sprintf(cbuffer,"Welcome to PIC & Pi Project Debug Window\n");
    debug_msg(cbuffer);
  char print_buffer[50]="Welcome to PIC & Pi Project Command Window\n";
  int iNumSendChars = 0;
    while (print_buffer[iNumSendChars] != '\0'){
        while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, print_buffer[iNumSendChars]);
        iNumSendChars++;
    }
    
    iNumSendChars=0;
    while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, '\n');
    
 
    DmaChnOpen(DMA_CHANNEL1, DMA_CHN_PRI2, DMA_OPEN_MATCH);

	DmaChnSetMatchPattern(DMA_CHANNEL1, EndOfTransmit);	// set \r as ending character

	// set the events: we want the UART2 rx interrupt to start our transfer
	// also we want to enable the pattern match: transfer stops upon detection of EOT
	DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(_UART2_RX_IRQ));

	// set the transfer source and dest addresses, source and dest sizes and the cell size
	DmaChnSetTxfer(DMA_CHANNEL1, (void*)&U2RXREG, ReceiveBuf, 1, 256, 1);

	DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);		// enable the transfer done interrupt: pattern match or all the characters transferred

	// enable system wide multi vectored interrupts
	INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
	INTEnableInterrupts();

	INTSetVectorPriority(INT_VECTOR_DMA(DMA_CHANNEL1), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(DMA_CHANNEL1), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority

	INTEnable(INT_SOURCE_DMA(DMA_CHANNEL1), INT_ENABLED);		// enable the chn interrupt in the INT controller

	// enable the chn
	DmaChnEnable(DMA_CHANNEL1);
    
  
while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));

  }
    
}
