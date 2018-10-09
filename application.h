void applianceControl(char charSwitchMSB, char charSwitchLSB, char charSwitchSTATE, char chDimmerSpeedMSB, char chDimmerSpeedLSB,
        char charParentalControl, char charFinalFrameState){

    //define used variables and initilize it with zero
    int integerSwitchNumber = 0;
    int integerSwitchState = 0;
    int integerSpeed = 0;
    int currentStateBufferPositions=0;
   // TX1REG = charParentalControl;
    // Get switch Number in Integer format 
    //define all used character data types and initlize it with "#"
    char switchNumberStringBuffer[2]="#";
    char dimmerSpeedStringBuffer[2]="#";
    
    switchNumberStringBuffer[0]=charSwitchMSB;
    switchNumberStringBuffer[1]=charSwitchLSB;    
    integerSwitchNumber = atoi(switchNumberStringBuffer);//convert string into integer
    
    // Get switch State in Integer Format
    
    integerSwitchState = charSwitchSTATE-'0';
    
    // Get speed of Fan or level of dimmer    
    dimmerSpeedStringBuffer[0]=chDimmerSpeedMSB;
    dimmerSpeedStringBuffer[1]=chDimmerSpeedLSB;    
    integerSpeed = atoi(dimmerSpeedStringBuffer);
    
    // save Parental lock state of each switch into parental lock buffer
//    int integerParentalControl=charParentalControl-'0';
    parentalLockBuffer[integerSwitchNumber] = charParentalControl;
   
   
    copy_parentalLockBuffer[integerSwitchNumber]=parentalLockBuffer[integerSwitchNumber];
  //   TX1REG = parentalLockBuffer[integerSwitchNumber]; //ok same
  //   TX1REG = copy_parentalLockBuffer[integerSwitchNumber];
    
    
    // ACKNOWLEDGMENT data Format :->> (Gateway+SwitchState+SwitchMSB+SwitchLSB)
    
    currentStateBufferPositions = ((1+4*(integerSwitchNumber))-5);
   
    
    currentStateBufferPositions-=3;     // since we have come forward by 3 address in current state buffer

    
    switch(integerSwitchNumber){
        case 1:
        {
            switch(charSwitchSTATE) {
                case '1':  
                    
                {   
                        sendFeedback_TO_Touch_Main('1','1');
                            OUTPUT_RELAY1=ON;__delay_ms(2); 
                            MainResponse[0]='G'; MainResponse[1]='1'; MainResponse[2]='0'; MainResponse[3]='1'; MainFeedback(MainResponse);
                           OUTPUT_RELAY2=OFF;__delay_ms(2);
                         MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='2'; MainFeedback(MainResponse);
                          curtFlag1=1;
                          timerCounter1=0;
                          TMR1H=0x0B;//500ms delay
                          TMR1L=0xDC;
                          PIR1bits.TMR1IF=0;       
                          T1CONbits.TMR1ON=1;
                }break;
                case '0':
                {
                            sendFeedback_TO_Touch_Main('0','0');
                          OUTPUT_RELAY1=OFF;__delay_ms(2); 
                            MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='1'; MainFeedback(MainResponse);
                           OUTPUT_RELAY2=OFF;__delay_ms(2);
                         MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='2'; MainFeedback(MainResponse);    
                          T1CONbits.TMR1ON=0;
                }break;
                default:
                break;
            }    
        }
            break;
        case 2:
            {
               switch(charSwitchSTATE){
                   case '1':
                   {
                        sendFeedback_TO_Touch_Main('1','0');
                                 OUTPUT_RELAY1=OFF;__delay_ms(1); 
                              MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='1'; MainFeedback(MainResponse);
                              OUTPUT_RELAY2=ON;__delay_ms(1);
                              MainResponse[0]='G'; MainResponse[1]='1'; MainResponse[2]='0'; MainResponse[3]='2'; MainFeedback(MainResponse);

                               curtFlag1=1;
                               timerCounter1=0;
                               TMR1H=0x0B;//500ms delay
                               TMR1L=0xDC;
                               PIR1bits.TMR1IF=0;       
                               T1CONbits.TMR1ON=1;
                   }break; 
                   case '0':
                   {
                            sendFeedback_TO_Touch_Main('0','0');
                               OUTPUT_RELAY1=OFF;__delay_ms(1); 
                              MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='1'; MainFeedback(MainResponse);
                              OUTPUT_RELAY2=OFF;__delay_ms(1);
                              MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='2'; MainFeedback(MainResponse);      
                               T1CONbits.TMR1ON=0;
                   }break;
                   default:
                       break;
               }      
            break;
            }
       
        default:
            break;
        }
    
}