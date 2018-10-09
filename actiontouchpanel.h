

void actiontouchPanel(char Switch_Num_MSB, char Switch_Num_LSB) //, char speeds
{

        M1=ON;    M2=ON;      M3=ON;    M4=ON;    M5=ON;    M6=ON;  M7=ON;  M8=ON;


   
  
    switch(Switch_Num_MSB) {
                
            case '1':{
                        switch(Switch_Num_LSB){
                            case '1':{
                                     if(M1 == ON && copy_parentalLockBuffer[1] == CHAR_OFF ){
             
                                                if(panel==1){
                                                 __delay_ms(5);  
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
                                                }
                                                 M1 = OFF;
                                                 panel=1;
                                     }
                                            
                                     }break;
                            case  '0':
                            {
                                if(M2 == ON && copy_parentalLockBuffer[2] == CHAR_OFF ){
                                             
                                                      if(panel==1){
                                                         __delay_ms(5);  
                                                       OUTPUT_RELAY1=OFF;__delay_ms(2); 
                                                       MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='1'; MainFeedback(MainResponse);
                                                       OUTPUT_RELAY2=ON;__delay_ms(2);
                                                       MainResponse[0]='G'; MainResponse[1]='1'; MainResponse[2]='0'; MainResponse[3]='2'; MainFeedback(MainResponse);
                                                        curtFlag1=1;
                                                        timerCounter1=0;
                                                        TMR1H=0x0B;//500ms delay
                                                        TMR1L=0xDC;
                                                        PIR1bits.TMR1IF=0;       
                                                        T1CONbits.TMR1ON=1;
                                                      }
                                                      M2=OFF;
                                            }      
                            }break;
                            default:
                             break;
                        }
                }break;
           case '0':{
                    if(panel==1){
                     __delay_ms(5);  
                   OUTPUT_RELAY1=OFF;__delay_ms(2); 
                   MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='1'; MainFeedback(MainResponse);
                   OUTPUT_RELAY2=OFF;__delay_ms(2);
                   MainResponse[0]='G'; MainResponse[1]='0'; MainResponse[2]='0'; MainResponse[3]='2'; MainFeedback(MainResponse);       
                    T1CONbits.TMR1ON=0;
                    } 
                 }break;
        default:
            break;
          }
       }
       
  
