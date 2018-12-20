////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_2_3.h"
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
    int iNumOfSamples;
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set  Sample Frequency",SetSampFreq);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Sample Frequency %d Khz",cSampleFreq,cSampleFreq);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte2  %02X  Num of Samples MSB Bits B11.B10.B9.B8.B7.B6",cSampleMSBbits);
    printLine(4, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte3  %02X  Num of Samples LSB Bits B5.B4.B3.B2.B1.B0",cSampleLSBbits);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
    iNumOfSamples=((cSampleMSBbits && 0x3f)<<6)|(cSampleLSBbits && 0x3f);
    sprintf(cbuffer,"Set %dKhz Sample frequency and acquire &d samples", cSampleFreq,iNumOfSamples);
    printLine(8, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
}
                        
void vExec_Start_ADC(char *cRecvData){
    char cChannel= cRecvData[0];
    tft_fillScreen(ILI9340_BLACK);
    sprintf(cbuffer,"Command    %02X    Set Start ADC",StartADC);
    printLine(0, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Byte1  %02X  Channel Number %d ",cChannel,cChannel);
    printLine(2, cbuffer, ILI9340_WHITE, ILI9340_BLUE);
    sprintf(cbuffer,"Start ACquisition in Channel no %d",cChannel,cChannel);
    printLine(6, cbuffer, ILI9340_WHITE, ILI9340_BLUE); 
}
                        
void vExec_Read_Buf(char *cRecvData){
    char cBufferNum= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    int iNumOfSamples;
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
}
void vExec_Write_Buf(char *cRecvData){
    char cBufferNum= cRecvData[0];
    char cSampleMSBbits =cRecvData[1];
    char cSampleLSBbits =cRecvData[2];
    int iNumOfSamples;
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
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x001F;
    TRISB = 0xE323;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPUA = 0x0000;
    CNPUB = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELA = 0x0003;
    ANSELB = 0x000E;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    SYSKEY = 0x12345678;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;    
    CFGCONbits.IOLOCK = 0;

    RPB7Rbits.RPB7R = 0x0001;   //RB7->UART1:U1TX;
    RPB3Rbits.RPB3R = 0x0005;   //RB3->OC1:OC1;
    IC1Rbits.IC1R = 0x0000;   //RA2->IC1:IC1;
    U1RXRbits.U1RXR = 0x0002;   //RA4->UART1:U1RX;
    U2RXRbits.U2RXR = 0x0001;   //RB5->UART2:U2RX;
    SDI2Rbits.SDI2R = 0x0003;   //RB13->SPI2:SDI2;
    SS2Rbits.SS2R = 0x0001;   //RB14->SPI2:SS2;
    RPB10Rbits.RPB10R = 0x0002;   //RB10->UART2:U2TX;
    IC2Rbits.IC2R = 0x0000;   //RA3->IC2:IC2;
    RPB2Rbits.RPB2R = 0x0005;   //RB2->OC4:OC4;
    RPB11Rbits.RPB11R = 0x0004;   //RB11->SPI2:SDO2;

    CFGCONbits.IOLOCK = 1; // lock   PPS
    SYSKEY = 0x00000000; 
    
}

void Variable_Initialize(void){
     RecvState= ACTIVE_WAIT_FOR_SOT;  
 
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

 //     PT_SCHEDULE(protothread_sound(&pt_sound));   
  }
    
}
