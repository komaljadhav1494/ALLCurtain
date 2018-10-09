/*
 * File:   main.c
 * Author: Alfaone
 *MOdule Name: Opal
 * description: single curtain only 2 switches
 * Created on 9 October, 2018, 12:01 PM
 * In opal frame:::::::::;
 * (mT1100)/(mT0000)
 * (mT1000)/(mT0000)
 */


#include <stdio.h>
#include <stdlib.h>
#include <pic16f1526.h>
// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
// Since we have used 16 MHz crystal
#define _XTAL_FREQ 16000000  

// Pin MACROS
#define OUTPUT_RELAY1 RF1
#define OUTPUT_RELAY2 RF0



#define OUTPUT_RELAY_DIR_1 TRISFbits.TRISF1
#define OUTPUT_RELAY_DIR_2 TRISFbits.TRISF0




/*
 * Extra Periferals Direction and PORT
 */
//#define ZCD_CCP9_DIR TRISEbits.TRISE3
// USART Directions
#define USART_1_TRANSMIT_OUTPUT_DIR TRISCbits.TRISC6
#define USART_1_RECIEVE_INPUT_DIR TRISCbits.TRISC7

#define USART_2_TRANSMIT_OUTPUT_DIR TRISGbits.TRISG1
#define USART_2_RECIEVE_INPUT_DIR TRISGbits.TRISG2

#define RECIEVED_DATA_LENGTH (16*2)
#define TOTAL_NUMBER_OF_SWITCH (8*2)


#define TOUCHPANEL_DATA_LENGTH (8*2)
#define TRUE 1
#define FALSE 0

#define CHAR_TRUE '1'
#define CHAR_FALSE '0'






// ALL error Definitions
/* 
 * #define WRONG_DATA_RECIEVED_ERROR_CODE ERRX
 * #define RECIVING_OVERRUN_ERROR EROV
 * #define RECEIVING_DATA_LOST_IN_MAIN ERLS
 */
/* DATA USED IN MANUAL  STARTS HERE*/
unsigned int M1;unsigned int M2;unsigned int M3;unsigned int M4;unsigned int M5;unsigned int M6;unsigned int M7;unsigned int M8;
 volatile int curtFlag1=0;
volatile int curtFlag2=0;
volatile int curtFlag3=0;
volatile int curtFlag4=0;
volatile int curtFlag5=0;
volatile int curtFlag6=0;
volatile int curtFlag7=0;
volatile int curtFlag8=0;
volatile int timerCounter1=0;
volatile int timerCounter2=0;
volatile int timerCounter3=0;
volatile int timerCounter4=0;
volatile int panel=1;
volatile int flag=0;
volatile int enter=1;
volatile int uart_1 = 0;
volatile int uart_2 = 0;
int time_completed=0;
#define ON 1
#define OFF 0
#define CHAR_OFF '0'
#define CHAR_ON '1'
//#define DEBUG        
/* DATA USED IN MANUAL END HERE*/
#define REQUIRED_CURTAIN_DELAY_IN_SECONDS 30000 
#define CURTAIN_COUNTER (REQUIRED_CURTAIN_DELAY_IN_SECONDS / 125)



unsigned char ErrorNames[5]="####";
unsigned char IsrResponse[5]="####";
unsigned char MainResponse[5]="####";
unsigned char UART2_response[9]="########";
volatile int mainReceivedDataPosition = 0;
volatile int mainDataReceived = 0;
unsigned char mainReceivedDataBuffer[RECIEVED_DATA_LENGTH]; 
unsigned char tempReceivedDataBuffer[RECIEVED_DATA_LENGTH-8];
unsigned char parentalLockBuffer[TOTAL_NUMBER_OF_SWITCH]="000000000000000";
unsigned char copy_parentalLockBuffer[TOTAL_NUMBER_OF_SWITCH]="000000000000000";
unsigned char currentStateBuffer[(TOTAL_NUMBER_OF_SWITCH*4)+2]="#";



int touchpanelReceivedataPosition = 0; 
int touchPanelDataReceived = FALSE;
unsigned char touchpanleReceivedDatabuffer[TOUCHPANEL_DATA_LENGTH]="#";
unsigned char tempReceiveTouchpanelDataBuffer[TOUCHPANEL_DATA_LENGTH-8]="#";


#define TouchMatikBoardAddress 'm'

unsigned int M1;unsigned int M2;unsigned int M3;unsigned int M4;unsigned int M5;

int start_PWM_Generation_in_ISR_FLAG=FALSE;
char levelofDimmer_MSB='0',levelofDimmer_LSB='0';

void errorsISR(char* errNum);
void errorsMain(char* errNum);
void IsrFeedback(char* IsrResponse);
void MainFeedback(char* MainResponse);
//void sendFeedback_TO_UART2_pointer(char* send_data);
void sendAcknowledgment(char* currentStateBuffer);
void sendFeedback_TO_Gateway(char sw_status, char Switch_Num);
//void sendFeedback_TO_Touch_ISR(char Switch_Num_1s);
void sendFeedback_TO_Touch_Main(char Switch_state_MSB, char Switch_state_LSB);

void clearAllPorts();
void pinINIT_extra();
void GPIO_pin_Initialize();

void AllInterruptEnable();
void EUSART_Initialize();
void EUSART2_Initialize();

void TMR3_Initialize();
void TMR1_Initialize();
void TMR2_Initialize();
void TMR5_Initialize();
void CCP9_Initialize();
void allPeripheralInit();

void copyReceivedDataBuffer();
void copyTouchpanelReceiveDataBuffer();
void applianceControl(char switchMSB, char switchLSB, char switchSTATE, char dimmerSpeedMSB, char dimmerSpeedLSB, char parentalControl, char finalFrameState);

void actiontouchPanel(char Switch_Num_MSB, char Switch_Num_LSB );//, char speeds
void sendFeedback_TO_Touch_ISR(char Switch_state_MSB,char Switch_state_LSB);
void sendFeedback_TO_UART2_pointer_ISR(char* send_data)
{
       int Tx_count=0;
      while(Tx_count!=8)
     {
        while (!TX2STAbits.TRMT);
         TX2REG = *send_data;
         *send_data++;
        Tx_count++;
     }
}
#include"include.h"
interrupt void isr(){
   

    // ************************************* XbEE UART INTERRUPT *********************************************** //
    if(RC1IF){        
        if(RC1STAbits.OERR){    // If over run error, then reset the receiver
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            
            ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='O';      ErrorNames[3]='V';
            errorsISR(ErrorNames); 
        } 
       if(RC1STAbits.FERR){    // If over run error, then reset the receiver
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            
            ErrorNames[0]='F';      ErrorNames[1]='E';      ErrorNames[2]='R';      ErrorNames[3]='R';
            errorsISR(ErrorNames); 
        } 
        mainReceivedDataBuffer[mainReceivedDataPosition]=RC1REG;
        #ifdef DEBUG
        TX1REG=mainReceivedDataBuffer[mainReceivedDataPosition];
        #endif
        if(mainReceivedDataBuffer[0]=='%'){
            mainReceivedDataPosition++;
            if(mainReceivedDataPosition>15){
                mainDataReceived=TRUE;
                mainReceivedDataPosition=0;                
                RC1IF=0;                
            }
        }
        else{
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            mainReceivedDataPosition=0; // Reinitiate buffer counter
            
            ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='R';      ErrorNames[3]='X';
            errorsISR(ErrorNames);            
        }
        uart_1=1;
    }// End of RC1IF
     
////////     /**************************************TOUCH_PANEL INTERRUPT*******************************************/
     if(RC2IF){    
         
         TX2STAbits.TXEN = 1;

       // Serial Port Enabled
        RC2STAbits.SPEN = 1;
        if(RC2STAbits.OERR){    // If over run error, then reset the receiver
            RC2STAbits.CREN = 0; // countinuous Recieve Disable
            RC2STAbits.CREN = 1; // countinuous Recieve Enable
            
            ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='O';      ErrorNames[3]='V';
            errorsISR(ErrorNames); 
        }   
        
           if(RC2STAbits.FERR){    // if frame lost//caused by mismatch of baud rate
            RC2STAbits.CREN = 0; // countinuous Recieve Disable
            RC2STAbits.CREN = 1; // countinuous Recieve Enable
            
            ErrorNames[0]='F';      ErrorNames[1]='E';      ErrorNames[2]='R';      ErrorNames[3]='R';
            errorsISR(ErrorNames); 
        } 
        
        touchpanleReceivedDatabuffer[touchpanelReceivedataPosition] = RC2REG;
        if(touchpanleReceivedDatabuffer[0] == '(')
        {
            touchpanelReceivedataPosition++;
            if(touchpanelReceivedataPosition > 7)
            {
                touchPanelDataReceived = TRUE;
                touchpanelReceivedataPosition=0;
                 RC2IF = 0;
            }
        }
        else
        {
            RC2STAbits.CREN = 0; // countinuous Recieve Disable
            RC2STAbits.CREN = 1; // countinuous Recieve Enable
            touchpanelReceivedataPosition=0; // Reinitiate buffer counter
            
            ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='R';      ErrorNames[3]='T';
            errorsISR(ErrorNames);            
        }
        uart_2=1;
    }//End of RC2IF
         
         
  //**************TIMER1*********************************
        if(PIR1bits.TMR1IF==1 && PIE1bits.TMR1IE==1 ) {
              PIR1bits.TMR1IF=0;
         
             if(timerCounter1 == 120 )
              {
                 time_completed=1;
              }
              else if(curtFlag1)
              {
                 timerCounter1=timerCounter1+1;
                    TMR1H=0x0B;        TMR1L=0xDC;        T1CONbits.TMR1ON = 1;
                    enter=2;
              }
      
      
    }//end of timer1
  
    

}




/*
 * Alfaone Main code starts here
 * 
 */
int main() {

        __delay_ms(2000); //this delay is very important for relays otherwise 
                          //as the program starts all relays will turn on automatically
        
    GPIO_pin_Initialize();
    allPeripheralInit();
  //  AllInterruptEnable();
 
    TX1REG='N';__delay_ms(1);
    TX1REG='N';__delay_ms(1);
    TX1REG='N';__delay_ms(1);
    TX1REG='N';__delay_ms(1);
  OUTPUT_RELAY1 = 0;
  OUTPUT_RELAY2 = 0;

 
    while(1){
        
        //Check time completion
        if(time_completed == TRUE)
        {
            time_completed=0;
             T1CONbits.TMR1ON=0;
                  timerCounter1=0;
                   curtFlag1=0;
                   sendFeedback_TO_Touch_ISR('0','0');
                    OUTPUT_RELAY1=0;
                    __delay_ms(1);
                      curtFlag1=0; curtFlag5=0;
                    IsrResponse[0]='G'; IsrResponse[1]='0'; IsrResponse[2]='0'; IsrResponse[3]='2';
                    IsrFeedback(IsrResponse);
                    sendFeedback_TO_Touch_ISR('0','0');
                    OUTPUT_RELAY2=0;  __delay_ms(1); 
                    IsrResponse[0]='G'; IsrResponse[1]='0'; IsrResponse[2]='0'; IsrResponse[3]='1';
                    IsrFeedback(IsrResponse);     __delay_ms(1);
        }
         ///STARTING OF MOBILE APP DATA RECEIVE
        if(mainDataReceived==TRUE ){
            uart_1=0;
            mainDataReceived=FALSE;
            int start_flag = 0;
            int end_flag = 0;
            if(mainReceivedDataBuffer[0]=='%' && mainReceivedDataBuffer[1]=='%' && mainReceivedDataBuffer[14]=='@' && mainReceivedDataBuffer[15]=='@'){
                if(mainReceivedDataBuffer[0] == '%' && mainReceivedDataBuffer[1]=='%' && start_flag == 0)
                {
                    end_flag = 1;
                }
                if(mainReceivedDataBuffer[14]=='@' && mainReceivedDataBuffer[15]=='@' && end_flag ==1)
                {
                    copyReceivedDataBuffer();
                                 start_flag = 0;
                                   end_flag = 0;
                }
                
                
                
                
                
                applianceControl(tempReceivedDataBuffer[0],
                        tempReceivedDataBuffer[1],
                        tempReceivedDataBuffer[2],
                        tempReceivedDataBuffer[3],
                        tempReceivedDataBuffer[4],
                        tempReceivedDataBuffer[5],
                        tempReceivedDataBuffer[6]);
                                
            }   // End of all buffer data check
            else{
                ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='L';      ErrorNames[3]='S';
                errorsMain(ErrorNames);
                RC1STAbits.SPEN=0;  // Serial port disabled 
                RC1STAbits.CREN = 0; // countinuous Recieve Disable                
                for(int dataBufferCounter = 0; dataBufferCounter< 15; dataBufferCounter++)
                {
                    mainReceivedDataBuffer[dataBufferCounter] = '#'; // clean received data buffer
                }
                RC1STAbits.CREN = 1; // countinuous Recieve Enable
                RC1STAbits.SPEN=1;  // Serial port enabled (configures RXx/DTx and TXx/CKx pins as serial port pins)
            }
        } // End of mainDataReceived condition
        
        ///STARTING OF TOUCHPANEL DATA RECEIVE
        if(touchPanelDataReceived == TRUE)
        {
          //  TX1REG = 'R';
            uart_2=0;
            touchPanelDataReceived = FALSE;
            int start_flag = 0;
            int end_flag = 0;
            if(touchpanleReceivedDatabuffer[0] == '(' && touchpanleReceivedDatabuffer[7] == ')')
            {
                
                if(touchpanleReceivedDatabuffer[0] == '('  && start_flag == 0)
                {
                    end_flag =1;

                }
                if(touchpanleReceivedDatabuffer[7] == ')' && end_flag ==1)
                {
                copyTouchpanelReceiveDataBuffer();
                if(tempReceiveTouchpanelDataBuffer[0] != '@'){
                   actiontouchPanel(tempReceiveTouchpanelDataBuffer[1],tempReceiveTouchpanelDataBuffer[2]); //,tempReceiveTouchpanelDataBuffer[2]
                    start_flag = 0;
                    end_flag = 0;
                }
                                
                }
               
            }
                else
                {
                ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='L';      ErrorNames[3]='S';
                errorsMain(ErrorNames);
                RC2STAbits.SPEN = 0;  // Serial port disabled  
                RC2STAbits.CREN = 0; // countinuous Recieve Disable                
                for(int dataBufferCounter = 0; dataBufferCounter< 8; dataBufferCounter++)
                {
                    touchpanleReceivedDatabuffer[dataBufferCounter] = '#'; // clean received data buffer
                }
                RC2STAbits.CREN = 1; // countinuous Recieve Enable
                RC2STAbits.SPEN=1;  // Serial port enabled (configures RXx/DTx and TXx/CKx pins as serial port pins)
            }
            
        }
        
       
}
}




        
    


/*
 * All input output pin initialization
 */
void GPIO_pin_Initialize(){
    clearAllPorts();
    pinINIT_extra();

    
    OUTPUT_RELAY_DIR_1 = 0;
    OUTPUT_RELAY_DIR_2 = 0;
 
    USART_1_TRANSMIT_OUTPUT_DIR = 0;
    USART_1_RECIEVE_INPUT_DIR = 1;
    
    USART_2_TRANSMIT_OUTPUT_DIR = 0;
    USART_2_TRANSMIT_OUTPUT_DIR = 1;
    
    clearAllPorts();
}

/*
 * ALL Peripheral Initialization
 */
void allPeripheralInit(){
    EUSART_Initialize();
    EUSART2_Initialize();
    TMR1_Initialize();
    TMR3_Initialize();

}

/*
 * USART Control Registers initialization
 */
void EUSART_Initialize(){
    PIE1bits.RC1IE = 0;
    PIE1bits.TX1IE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE enabled; ABDEN disabled;
    BAUD1CON = 0x0A;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    RC1STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
    TX1STA = 0x24;

    // Baud Rate = 9600; SP1BRGL 12;
    //SPBRGL = 0x0C;
    //SPBRGL = 0x19;                  // SP1BRGL is 25 (hex value=0x19) for 9600 baud on 16 MHz crystal frequency
    SP1BRGL = 0xA0;                  // SYNC =0 ; BRGH = 1 ; BRG16=1;
    // Baud Rate = 9600; SP1BRGH 1;
    SP1BRGH = 0x01;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

    // enable receive interrupt
    PIE1bits.RC1IE = 1;                    // handled into INTERRUPT_Initialize()
  
    // Transmit Enabled
    TX1STAbits.TXEN = 1;

    // Serial Port Enabled
    RC1STAbits.SPEN = 1;
}
void EUSART2_Initialize()
{
    PIE4bits.RC2IE = 0;
    PIE4bits.TX2IE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE enabled; ABDEN disabled;
    BAUD2CON = 0x0A;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    RC2STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
    TX2STA = 0x24;

    // Baud Rate = 9600; SP1BRGL 12;
    SP2BRGL = 0xA0;                  // SYNC =0 ; BRGH = 1 ; BRG16=1;
    // Baud Rate = 9600; SP1BRGH 1;
    SP2BRGH = 0x01;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

    // enable receive interrupt  
    PIE4bits.RC2IE = 1; // handled into INTERRUPT_Initialize()
   // PIE4bits.TX2IE = 0;
    // Transmit Enabled
    TX2STAbits.TXEN = 1;

    // Serial Port Enabled
    RC2STAbits.SPEN = 1;
}
void TMR1_Initialize(void)
{
   //TxCKPS<1:0>: Timer1/3/5 Input Clock Prescale Select bits
//11 =1:8 Prescale valuet
    T1CON = 0x30;

    //T1GSS T1G; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled;
    T1GCON = 0x00;

        //TMR1H 29;
    TMR1H = 0x00;

    //TMR1L 112;
    TMR1L = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR1IF = 0;

    // Enabling TMR1 interrupt.
    PIE1bits.TMR1IE = 1;

    // Start TMR1
   // T1CONbits.TMR1ON = 1;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

}

void TMR3_Initialize(void)
{

     //Set the Timer to the options selected in the GUI

    //T5CKPS 1:8; T5OSCEN disabled; nT5SYNC synchronize; TMR5CS FOSC/4; TMR5ON off;
    T3CON = 0x30;

    //T5GSS T5G; TMR5GE disabled; T5GTM disabled; T5GPOL low; T5GGO_nDONE done; T5GSPM disabled;
    T3GCON = 0x00;

    //TMR5H 123;
    TMR3H = 0x00;

    //TMR5L 48;
    TMR3L = 0x00;

    // Clearing IF flag.
    PIR3bits.TMR3IF = 0;   
   
    // Enabling TMR5 interrupt.
    PIE3bits.TMR3IE = 1;
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

}


void errorsISR(char* errNum){
    int Tx_count=0;
  	while(Tx_count!=4)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *errNum;
 		*errNum++;
        Tx_count++;
 	}
}
void errorsMain(char* errNum){
   int Tx_count=0;
  	while(Tx_count!=4)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *errNum;
 		*errNum++;
        Tx_count++;
 	}
}
void IsrFeedback(char* IsrResponse)
{
      int Tx_count=0;
      while(Tx_count!=4)
     {
        while (!TX1STAbits.TRMT);
//        TX1REG='S';
         TX1REG = *IsrResponse;
         *IsrResponse++;
        Tx_count++;
     }
}
void MainFeedback(char* Mainresponse)
{
      int Tx_count=0;
      while(Tx_count!=4)
     {
        while (!TX1STAbits.TRMT);
//        TX1REG='S';
         TX1REG = *Mainresponse;
         *Mainresponse++;
        Tx_count++;
     }
}

void sendAcknowledgment(char* currentStateBuffer){
  int Tx_count=0;
  	while(Tx_count!=4)
 	{ 
        while (!TX1STAbits.TRMT);
//        TX1REG='S';
 		TX1REG = *currentStateBuffer;
 		*currentStateBuffer++;
        Tx_count++;
 	}
}
void sendFeedback_TO_Gateway(char sw_status, char Switch_Num){
    __delay_ms(5);      TX1REG = 'G';
    __delay_ms(1);      TX1REG = sw_status;
    __delay_ms(1);      TX1REG = '0';
    __delay_ms(1);      TX1REG = Switch_Num;
}
void sendFeedback_TO_Touch_ISR(char Switch_state_MSB,char Switch_state_LSB){     
        __delay_ms(5);
        TX2REG = '(' ;
        __delay_ms(1);
        TX2REG = TouchMatikBoardAddress ;//touchmatik address
        __delay_ms(1);
        TX2REG ='T' ;
        __delay_ms(1);
        TX2REG=Switch_state_MSB;
        __delay_ms(1);
        TX2REG=Switch_state_LSB;
        __delay_ms(1);
        TX2REG='0';
        __delay_ms(1);
        TX2REG='0';
        __delay_ms(1);
        TX2REG=')';
        __delay_ms(1);
}
void sendFeedback_TO_Touch_Main(char Switch_state_MSB,char Switch_state_LSB){
  
        __delay_ms(5);
        TX2REG = '(' ;
        __delay_ms(1);
        TX2REG = TouchMatikBoardAddress ;//touchmatik address
        __delay_ms(1);
        TX2REG ='T' ;
        __delay_ms(1);
        TX2REG=Switch_state_MSB;
        __delay_ms(1);
        TX2REG=Switch_state_LSB;
        __delay_ms(1);
        TX2REG='0';
        __delay_ms(1);
        TX2REG='0';
        __delay_ms(1);
        TX2REG=')';
}
void copyReceivedDataBuffer(){
    int dataBufferCounter=2;
    for(dataBufferCounter=2;dataBufferCounter<9;dataBufferCounter++){
        tempReceivedDataBuffer[dataBufferCounter-2]=mainReceivedDataBuffer[dataBufferCounter]; // copy data buffer from main
        mainReceivedDataBuffer[dataBufferCounter]='#';  // clean data buffer
    }
}
void copyTouchpanelReceiveDataBuffer()
{
     int dataBufferCounter=2;
     for(dataBufferCounter=2; dataBufferCounter<5;dataBufferCounter++)
     {
         tempReceiveTouchpanelDataBuffer[dataBufferCounter-2] = touchpanleReceivedDatabuffer[dataBufferCounter];
         touchpanleReceivedDatabuffer[dataBufferCounter] = "#";
     }
}
/*
 * AANALOG and PULL up REGISTERS related initialization
 */
void pinINIT_extra(){
    ANSELG=0x00;    WPUG = 0;
    
    ANSELF=0x00; 
    
    ANSELE=0x00;    WPUE=0x00;
    
    ANSELD=0x00;    WPUD=0x00;
    
    ANSELB=0x00;    WPUB=0x00;
    
    ANSELA=0x00;     
} 

/*
 * always clear all the ports before initialization
 */
void clearAllPorts()
{
  //  TX1REG='C';
  OUTPUT_RELAY1 = 0;
  OUTPUT_RELAY2 = 0;
}
