#include "idleTask.h"
#include "fsl_debug_console.h"
#include "sensors.h"
#include "lp5521.h"
#include "takeupMotor.h"
#include "virtualScope.h"
#include "developmentSettings.h"
#include "translator.h"

static TaskHandle_t     pHandle_                = NULL;
static bool             suspend_                = false;

static void idleTask( void *pvParameters );

BaseType_t createIdleTask( void )
{
    BaseType_t result;


    PRINTF("idleTask(): Starting...\r\n" );
    
     /* create printer task thread */
    result = xTaskCreate( idleTask,  "IdleTask", configMINIMAL_STACK_SIZE,
                                        NULL, idle_task_PRIORITY, &pHandle_ );
    return result;    
}


static void idleTask( void *pvParameters )
{
    PRINTF("idleTask(): Thread running...\r\n" );
    vTaskDelay( pdMS_TO_TICKS( 500 ) );
     
    while( !suspend_ ) {
        
        while(1)
        {
            static unsigned short printfThrottle = 0;
            printfThrottle++;
            if(printfThrottle >= 500) {
               //PRINTF("Media Reading: %d, outOfMedia?: %d\r\n", pollMediaCounts(),getOutOfMedia());
               //sendLowLabelMinMax(0, 0, 0, 0);
               printfThrottle = 0;  
            }
           
            //printStepsTakenTPHIntr();
            printFromInterrupt();
            printLLAverageTime();
            vTaskDelay( pdMS_TO_TICKS( 20 ) );
        }
    }
    vTaskSuspend(NULL);  
    
}