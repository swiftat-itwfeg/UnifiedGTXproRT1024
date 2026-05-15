#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "threadManager.h"
#include "fsl_debug_console.h"

/*! ****************************************************************************   
      \fn int main( void )                                                              

      \brief
         This function is the starting point for the unified scale application.
                  
      \author
          Aaron Swift
*******************************************************************************/ 
int main( void )
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    
    systemStartup();  
    while(1) {}
    return 0;
}