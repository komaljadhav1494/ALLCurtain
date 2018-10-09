# ALLCurtain


 header files used:
 * 1>>>actiontouchpanel.h>>>>>carrying all information about touchpanel
 * 2>>application.h>>>carrying all info aout apppplication control by app
 * 3>>include.h>>this is most important file because all files are included in "include.h"
 * i defined "include.h" header file just above the ISR so, we dont need to define and declare all variable inside the header files
 
#define OUTPUT_RELAY1 RF1
#define OUTPUT_RELAY2 RF0



#define OUTPUT_RELAY_DIR_1 TRISFbits.TRISF1
#define OUTPUT_RELAY_DIR_2 TRISFbits.TRISF0
