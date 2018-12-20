

#ifndef CONFIG_H
#define	CONFIG_H
#define _SUPPRESS_PLIB_WARNING 
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include <plib.h>
// serial stuff
#include <stdio.h>

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

#define ComBaudRate     9600
#define DebugBaudRate   9600
#define StartOfTransmit 0xF0
#define EndOfTransmit   0xD7

#define Handshake       0x1A
#define ReadInput       0x2A
#define WriteInput      0x2B
#define DACSetA         0x3A
#define DACSetB         0x3B
#define CheckBuf        0x4A
#define SetSampFreq     0x4B
#define StartADC        0x4C
#define SetPWMPer       0x5A
#define StartPWM1       0x5B
#define StartPWM2       0x5C
#define ReadBuf         0x6A
#define WriteBuf        0x6B

static unsigned int sys_time_seconds =0;
static char cRecvData[10];
static char cTranData[10];
static char cbuffer[20];
static char cRecvChar;
static int iCheckSumValid=0;

int iSampleCh[4],iSampleChBuf[4];
int iTotalNumOfADCSamples=0, iNumOfADCSamples[4];

int iBuffer[4][1024];

void printLine(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 10 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 8, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(1);
    tft_writeString(print_buffer);
    int iNumSendChars = 0;
    while (print_buffer[num_send_chars] != 0){
        while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, print_buffer[iNumSendChars]);
        num_send_chars++;
    }
    while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, '\n');
}


void vExec_Read_Input(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Read Input",ReadInput);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Read Bits B7.B6.B5.B4 if 1",cMSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Read Bits B3.B2.B1.B0 if 1",cLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);

}

void vExec_Write_Input(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Write Input",WriteInput);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Write Bits B7.B6.B5.B4 if 1",cMSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Write Bits B3.B2.B1.B0 if 1",cLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);    
}

void vExec_DAC_SetA(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
    int iDACValue;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set DAC Channel A",DACSetA);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  DAC MSB Bits B11.B10.B9.B8.B7.B6",cMSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  DAC LSB Bits B5.B4.B3.B2.B1.B0",cLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iDACValue=((cMSBbits && 0x3f)<<6)|(cLSBbits && 0x3f);
    sprintf(cbuffer,"Set DAC A to %d", iDACValue);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    mPORTBClearBits(BIT_11); // start transaction
    WriteSPI2( DAC_config_chan_A | (iDACValue));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
                            // CS high
    mPORTBSetBits(BIT_11); // end transaction
}
                        
void vExec_DAC_SetB(char *cRecvData){
    char cMSBbits =cRecvData[0];
    char cLSBbits =cRecvData[1];
    int iDACValue;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set DAC Channel B",DACSetB);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  DAC MSB Bits B11.B10.B9.B8.B7.B6",cMSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  DAC LSB Bits B5.B4.B3.B2.B1.B0",cLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iDACValue=((cMSBbits && 0x3f)<<6)|(cLSBbits && 0x3f);
    sprintf(cbuffer,"Set DAC B to %d", iDACValue);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    mPORTBClearBits(BIT_11); // start transaction
    WriteSPI2( DAC_config_chan_B | (iDACValue));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
                            // CS high
    mPORTBSetBits(BIT_11); // end transaction
}
    
void vExec_Check_Buf(){
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Check Buffer Status",CheckBuf);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    
}
                        
void vExec_Set_Samp_Freq(char *cRecvData){
    char cSampleFreq= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  Sample Frequency",SetSampFreq);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Sample Frequency %d Khz",cSampleFreq,cSampleFreq);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iTotalNumOfADCSamples=((cSampleMSBbits && 0x3f)<<6)|(cSampleLSBbits && 0x3f);
    sprintf(cbuffer,"Set %dKhz Sample frequency and acquire &d samples", cSampleFreq,iTotalNumOfADCSamples);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, cSampleFreq<<10);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2); 
    mT1ClearIntFlag(); // and clear the interrupt flag 
    
}
                        
void vExec_Start_ADC(char *cRecvData){
    char cChannel= cRecvData[0]>>2;
    char cBuffer=cRecvData[0]&0xFC;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set Start ADC",StartADC);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Channel Number %d Buffer Numer %d",cRecvData,cChannel,cBuffer);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Start ACquisition in Channel no %d",cChannel,cChannel);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iSampleCh[cChannel]=1;
    iSampleChBuf[cChannel]=cBuffer;

}
                        
void vExec_Read_Buf(char *cRecvData){
    char cBufferNum= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    int iNumOfSamples;
    int iByteCount=0;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Read Buffer",ReadBuf);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Buffer Number %d",cBufferNum,cBufferNum);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iNumOfSamples=((cSampleMSBbits && 0x3f)<<6)|(cSampleLSBbits && 0x3f);
    sprintf(cbuffer,"Read %d Samples from Buffer Number &d", iNumOfSamples,cBufferNum);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
     for(iByteCount=0;iByteCount<iNumOfSamples;iByteCount++){
        while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART1, iBuffer[cBufferNum][iByteCount]);
    }
}

void vExec_Write_Buf(char *cRecvData){
    char cBufferNum= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    int iNumOfSamples;
    int iByteCount=0;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Write Buffer",WriteBuf);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Buffer Number %d",cBufferNum,cBufferNum);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iNumOfSamples=((cSampleMSBbits && 0x3f)<<6)|(cSampleLSBbits && 0x3f);
    sprintf(cbuffer,"Write %d Samples to Buffer Number &d", iNumOfSamples,cBufferNum);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    
   
    for(iByteCount=0;iByteCount<iNumOfSamples;iByteCount++){
            while(!UARTReceivedDataIsAvailable(UART2));
            iBuffer[cBufferNum][iByteCount]=iUARTGetDataByte(UART2);
    }
}

void vExec_Set_PWM_Per(char *cRecvData){
    char cPeriodMSBbits= cRecvData[0];
    char cPeriodMLSBbits =cRecvData[1];
    char cPeriodLSBbits =cRecvData[2];
    int iPeriod;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  PWM Period",SetPWMPer);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Period MSB B15.B14.B13.B12.B11.B10",cPeriodMSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Period MSB Bits B11.B10.B9.B8.B7.B6",cPeriodMLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  Period LSB Bits B5.B4.B3.B2.B1.B0",cPeriodLSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iPeriod=((cPeriodMSBbits && 0x3f)<<10)|((cPeriodMLSBbits && 0x1f)<<5)|cPeriodLSBbits;
    sprintf(cbuffer,"Set PWM Period to %d",iPeriod);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, iPeriod);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2); 
    mT2ClearIntFlag(); // and clear the interrupt flag 
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , iPeriod, iPeriod); // 
    OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , iPeriod, iPeriod); // 


} 
    


void vExec_Start_PWM1(char *cRecvData){
    char cOnTime1MSBbits= cRecvData[0];
    char cOnTime1MLSBbits =cRecvData[1];
    char cOnTime1LSBbits =cRecvData[2];
    int iPWM1ONPeriod;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  PWM 1",StartPWM1);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  On Time 1 MSB B15.B14.B13.B12.B11.B10",cOnTime1MSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  On Time 1 MSB Bits B11.B10.B9.B8.B7.B6",cOnTime1MLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  On Time 1 LSB Bits B5.B4.B3.B2.B1.B0",cOnTime1LSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iPWM1ONPeriod=((cOnTime1MSBbits && 0x3f)<<10)|((cOnTime1MLSBbits && 0x1f)<<5)|cOnTime1LSBbits;
    sprintf(cbuffer,"Set PWM Period to %d",iPWM1ONPeriod);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    SetDCOC1PWM(iPWM1ONPeriod);

}
    
void vExec_Start_PWM2(char *cRecvData){
    char cOnTime2MSBbits= cRecvData[0];
    char cOnTime2MLSBbits =cRecvData[1];
    char cOnTime2LSBbits =cRecvData[2];
    int iPWM2ONPeriod;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  PWM 1",StartPWM2);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  On Time 1 MSB B15.B14.B13.B12.B11.B10",cOnTime2MSBbits);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  On Time 1 MSB Bits B11.B10.B9.B8.B7.B6",cOnTime2MLSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  On Time 1 LSB Bits B5.B4.B3.B2.B1.B0",cOnTime2LSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iPWM2ONPeriod=((cOnTime2MSBbits && 0x3f)<<10)|((cOnTime2MLSBbits && 0x1f)<<5)|cOnTime2LSBbits;
    sprintf(cbuffer,"Set PWM Period to %d",iPWM2ONPeriod);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    SetDCOC4PWM(iPWM2ONPeriod); 
}

int CheckSum(char *cRecvData, int iNumOfBytes, char iSum){
   
    iCheckSumValid=1; 
    
}

void vSendErrorMsg(char RecvStatus){
       
}


void vSendRespMsg(char cCommand,char *cTansData)
{
     
}

int iNumOfDataBytes(char cRecvChar){
     switch(cRecvChar){
                        
                        
                        case(ReadInput):    return 2;
                        
                        case(WriteInput):   return 2;
                        
                        case(DACSetA):      return 2;
                        
                        case(DACSetB):      return 2;
                        
                        case(CheckBuf):     return 1;
                        
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

 void vExecuteCommand(char cCommand,char *cRecvData){
     switch(cCommand){
                        
                        case(ReadInput):    vExec_Read_Input(cRecvData);     break;
                        
                        case(WriteInput):   vExec_Write_Input(cRecvData);    break;
                        
                        case(DACSetA):      vExec_DAC_SetA(cRecvData);       break;
                        
                        case(DACSetB):      vExec_DAC_SetB(cRecvData);       break;
                        
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
mT1ClearIntFlag();                                 // clear the interrupt flag
int iChannelCount;
    for(iChannelCount=0;iChannelCount<4;iChannelCount++){
        if(iNumOfADCSamples[iChannelCount]==iTotalNumOfADCSamples)
            iSampleCh[iChannelCount]=0;
        else if(iSampleCh[iChannelCount]){
            iBuffer[iSampleChBuf][iNumOfADCSamples[iChannelCount]++]=ReadADC10(iChannelCount);
        }
        
    }
}

static struct pt pt_uart_receive, pt_timer;


static int iNumOfRecvBytes=0;
static int iByteCount=0;
static char cCommand;

static PT_THREAD (protothread_uart_receive(struct pt *pt))
{
     PT_BEGIN(pt);
   
    while(1){
        
        PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
        cRecvChar = UARTGetDataByte(UART2);
        
        if(RecvState==ACTIVE_WAIT_FOR_SOT && recv_char== StartOfTransmit){
            RecvState=ACTIVE_PARSE;
            iCheckSumValid=0;
            RecvStatus=NoERR;
        }
        
        else if (RecvState==ACTIVE_WAIT_FOR_EOT && recv_char== EndOfTransmit){
                if(RecvStatus==NoERR){
                    vExecuteCommand(cCommand,cRecvData);
                    vSendRespMsg(cCommand,cTranData);
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
                            PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
                            cRecvData[iByteCount]=UARTGetDataByte(UART2);
                        }
                }
                RecvState=ACTIVE_CHECKSUM;
        }
  
    }
      
    PT_END(pt);

    
}


static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
   
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
    
   
   PT_END(pt);

}

void PIN_MANAGER_Initialize(void)
{
PPSInput(	3	,	IC1	    ,	RPA2	);
PPSInput(	3	,	U1RX	,	RPA4	);
PPSInput(	2	,	U2RX	,	RPB5	);
PPSInput(	3	,	SDI2	,	RPB13	);
PPSInput(	4	,	IC2	    ,	RPA3	);
PPSOutput(	1	,	RPB7	,	U1TX	);
PPSOutput(	1	,	RPB3	,	OC1	    );
PPSOutput(	4	,	RPB14	,	SS2	    );
PPSOutput(	4	,	RPB10	,	U2TX	);
PPSOutput(	3	,	RPB2	,	OC4	    );
PPSOutput(	2	,	RPB11	,	SDO2	); 
    
}

void Variable_Initialize(void){
    iSampleCh[0]=0;
    iSampleCh[1]=0;
    iSampleCh[2]=0;
    iSampleCh[3]=0;
    iNumOfSamples[0]=0;
    iNumOfSamples[1]=0;
    iNumOfSamples[2]=0;
    iNumOfSamples[3]=0;
    RecvState= ACTIVE_WAIT_FOR_SOT;  
 
}


void main(){

  PT_setup();
 // PIN_MANAGER_Initialize();
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
  
 // the ADC ///////////////////////////////////////
// configure and enable the ADC
CloseADC10(); // ensure the ADC is off before setting the configuration

// define setup parameters for OpenADC10
// Turn module on | ouput in integer | trigger mode auto | enable autosample
// ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
// ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
// ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
#define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

// define setup parameters for OpenADC10
// ADC ref external  | disable offset test | disable scan mode | do 2 sample | use single buf | alternate mode on
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_ON
        //
// Define setup parameters for OpenADC10
// use peripherial bus clock | set sample time | set ADC clock divider
// ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
// ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
// SLOW it down a little
#define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

// define setup parameters for OpenADC10
// set AN11 and  as analog inputs
#define PARAM4 ENABLE_AN0_ANA | ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN3_ANA// 

// define setup parameters for OpenADC10
// do not assign channels to scan
#define PARAM5 SKIP_SCAN_ALL //|SKIP_SCAN_AN5 //SKIP_SCAN_AN1 |SKIP_SCAN_AN5  //SKIP_SCAN_ALL
 
// // configure to sample AN5 and AN1 on MUX A and B
SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );
    
OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

EnableADC10(); // Enable the ADC
    
    
    
  PT_INIT(&pt_timer);
  PT_INIT(&pt_uart_receive);
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  sprintf(cbuffer,"Welcome to PIC & Pi Project\n");
  printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);

while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_uart_receive(&pt_uart_receive));

  }
    
}
