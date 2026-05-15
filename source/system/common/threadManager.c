#include "threadManager.h"
#include "queueManager.h"
#include "deviceProperties.h"
#include "systemTimer.h"
#include "sensors.h"
#include "hobartPrinterTask.h"
#include "hobartWeigherTask.h"
#include "averyWeigherTask.h"
#include "averyCutter.h"
#include "usbTask.h"
#include "idleTask.h"
#include "vendor.h"
#include "task.h"
#include "portable.h"
#include "fsl_clock.h"
#include "fsl_gpt.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "unitTest.h"
#include "fsl_debug_console.h"
#include "translator.h"
#include "takeupMotor.h"
#include "developmentSettings.h"
#include "w25x10cl.h"   
#include "serialFlash.h"



static TaskHandle_t     managerHandle_                          = NULL;
static QueueHandle_t    managerQHandle_                         = NULL;
static TaskHandle_t     managedTaskHandles_[LAST_TASK]          = NULL;
/* here is how this works each index in the array corelates to the ManagedThreads
   enmum in threadManager.h The masks determine what permissions correlate to each
   bit. So if (taskPermissions[WEIGHER] & TEMINATEMASK) then you can terminate
   the managedTaskHandles_[WEIGHER] task
*/
static uint8_t taskPermissions[LAST_TASK]               = { WEIGHERPERMISSIONS, 
                                                            PRINTERPERMISSIONS,
                                                            USBPERMISSIONS, 
                                                            TRANSLATORPERMISSIONS,
                                                            SENSORSPERMISSIONS, 
                                                            UNITTESTPERMISSIONS,
                                                            DOTWEARPERMISSIONS };
/* set to 1 to print out the task list at a 1 sec interval */
#ifdef PRINT_STATS 
TimerHandle_t statsTimer                =  NULL;
#endif

static bool suspend_                    = false;

volatile unsigned long  statCntr        = 0;
static PeripheralModel_t  model_        = GLOBAL_SCALE_UNKNOWN;
static int              totalHeap_      = 0;
static DEVICE_PROPERTIES_t dProperties;

extern Pr_Config config_;
extern PrStatusInfo                     currentStatus;
extern PrStatusInfo                     prevStatus;
extern volatile ADCManager              adcManager;

extern TaskHandle_t getDotWearHandle();
extern BaseType_t createDotWearTask( void );
/******************************************************************************/
/*!   \fn void systemStartup( void )

      \brief
        This function intializes kernel resources, starts up system resources 
        and determines system type to start peripheral threads.

      \author
          Aaron Swift
*******************************************************************************/
void systemStartup( void )
{
    /* initialize the serial flash interface */
    if( initializeSerialFlash() ) {
        PRINTF("systemStartup(): serial flash interface initialized\r\n");
        if( !initSerialFlashMutex() ) {
            PRINTF("systemStartup(): failed to initialize serial flash mutex!\r\n");
            /* force reboot of scale */
            assert( 0 );            
        }
    } else {
        PRINTF("systemStartup(): failed to initialize serial flash interface!\r\n");        
        /* force reboot of scale */
        assert( 0 );        
    }
    
    
    /* before accessing serial flash, we must first get a lock on it */
    getLockSerialFlash();
    /* access our device properties */
    if( !getDeviesProperties( &dProperties ) ) {
        PRINTF("systemStartup(): failed to obtain device properties!\r\n");
    }
    /* release the lock onthe serial flash */
    releaseLockSerialFlash();
    
    /* determine our model */
    model_ = getPeripheralModel( &dProperties );  

    /* get our initial heap size */
    totalHeap_ = (uint32_t)xPortGetFreeHeapSize();
    PRINTF("starting heap size: %d\r\n", totalHeap_);

    /* initialize our message queues */
    initializeQueueManager( model_ );
    PRINTF("heap after queue allocation: %d\r\n", (uint32_t)xPortGetFreeHeapSize());
    PRINTF("allocation size: %d\r\n", ( totalHeap_ - (uint32_t)xPortGetFreeHeapSize() ) );

    /* make sure we have our message queues */ 
    if( ( model_ == GLOBAL_SCALE_AV_XPRO ) ||
        ( model_ == GLOBAL_SCALE_AV_XONE ) || 
        ( model_ == GLOBAL_SCALE_HB_GT ) || 
        ( model_ == GLOBAL_SCALE_HB_DT ) ) {
        if( ( getUsbInPrQueueHandle() == NULL ) || 
            ( getUsbInWrQueueHandle() == NULL ) || 
            ( getUsbOutPrQueueHandle() == NULL ) || 
            ( getUsbOutWrQueueHandle() == NULL ) ||
            ( getWeigherQueueHandle() == NULL ) || 
            ( getPrinterQueueHandle() == NULL ) ) {
            PRINTF("systemStartup(): critical error: failed to create message queues!\r\n");               
            /* force reboot of scale */
            assert( 0 );                     
        }          
    } else if( ( model_ == GLOBAL_SCALE_AV_WEIGHER ) || 
               ( model_ == GLOBAL_SCALE_HB_WEIGHER ) ) {
         if( ( getUsbInWrQueueHandle() == NULL ) ||
             ( getUsbOutWrQueueHandle() == NULL ) ||
             ( getWeigherQueueHandle() == NULL ) ){
            PRINTF("systemStartup(): critical error: failed to create message queues!\r\n");               
            /* force reboot of scale */
            assert( 0 );                                     
         }              
    } else if ( ( model_ == GLOBAL_SCALE_AV_PRINTER ) ||
                ( model_ == GLOBAL_SCALE_HB_PRINTER ) ) {
         if( ( getUsbInPrQueueHandle() == NULL ) ||
             ( getUsbOutPrQueueHandle() == NULL ) ||
             ( getPrinterQueueHandle() == NULL ) ) {
            PRINTF("systemStartup(): critical error: failed to create message queues!\r\n");               
            /* force reboot of scale */
            assert( 0 );                                     
         }              
    }

    showSystemBanner();
        
    /* intialize timer resources*/
    PRINTF("systemStartup(): initialize timer resources\r\n");
    initTimers(); 
    if( createHeartBeatTimer() )
        startHeartBeatTimer();
     /* initialize and start rtc */
    initTimeAndDate();
    showDateTime();
    
    /* setup the sensors in auto mode */
    PRINTF("systemStartup(): initialize sensors interface\r\n");
    createSensorsTask( AD_AUTO );    // AD_AUTO AD_MANUAL

    HEADTYPE printheadType = getPrintHeadType();
    PrinterStyle style_ = RT_PRINTER_UNKNOWN;    
    if( printheadType == ROHM_80MM_650_OHM ) {
        PRINTF("80MM printhead detected\r\n");        
        style_ = RT_PRINTER_SERVICE_SCALE_80MM;
      
		/* set stepper current limit (a,b) */
        GPIO_WritePinOutput( GPIO3, 5U, true ); 
        GPIO_WritePinOutput( GPIO3, 7U, true ); 
      
    } else if( printheadType == ROHM_72MM_800_OHM ) {
        PRINTF("72MM printhead detected\r\n");
        style_ = RT_PRINTER_SERVICE_SCALE_72MM;
      
        /* set stepper current limit (a,b) */
        GPIO_WritePinOutput( GPIO3, 5U, false ); 
        GPIO_WritePinOutput( GPIO3, 7U, false );       
    }
    
    PRINTF( "spawning global tasks\r\n" ); 
    if( ( model_ == GLOBAL_SCALE_AV_XPRO ) || ( model_ == GLOBAL_SCALE_AV_XONE ) ) {
        PRINTF( "systemStartup(): spawning global Xpro / Xone tasks -- avery\r\n" ); 
        
        uS_Clock_init();
        if( createTranslator( dProperties.subClass, 
                              getWeigherQueueHandle(), getPrinterQueueHandle(),                              
                              getUsbInPrQueueHandle(), getUsbInWrQueueHandle(),
                              getUsbOutPrQueueHandle(), getUsbOutWrQueueHandle() ) == pdTRUE ) {                                    

            if( createUsbTask( dProperties.subClass, (QueueHandle_t)getUsbInPrQueueHandle(), 
                             (QueueHandle_t)getUsbInWrQueueHandle()) == pdTRUE ) {        
                if( createAveryWeigherTask( RT_WEIGHER_CS5530_30LBS, (QueueHandle_t)getAveryWeigherQueueHandle() ) ) {
                    if( createAveryPrinterTask( dProperties.subClass, (QueueHandle_t)getAveryPrinterQueueHandle() ) ) {
                        createIdleTask();                      
                        /* start created threads */
                        startSystemThreads();                                         
                    } else {          
                       PRINTF("systemStartup(): Falied to create Printer task!\r\n");  
                       /* force reboot of scale */
                      assert( 0 );            
                    }
                } else {
                    PRINTF("systemStartup(): Falied to create Weigher task!\r\n");        
                    /* force reboot of scale */
                    assert( 0 );                            
                }
            }
        }        
    } else if( ( model_ == GLOBAL_SCALE_HB_GT ) || ( model_ == GLOBAL_SCALE_HB_DT ) ) {
        if( createTranslator( dProperties.subClass, 
                              getWeigherQueueHandle(), getPrinterQueueHandle(),                              
                              getUsbInPrQueueHandle(), getUsbInWrQueueHandle(),
                              getUsbOutPrQueueHandle(), getUsbOutWrQueueHandle() ) == pdTRUE ) {                                    
        

            if( createUsbTask( dProperties.subClass, (QueueHandle_t)getUsbInPrQueueHandle(), 
                             ( QueueHandle_t)getUsbInWrQueueHandle()) == pdTRUE ) {        
                if( createHobartPrinterTask( style_, (QueueHandle_t)getPrinterQueueHandle() ) == pdTRUE ) {   
                    /* TO DO: move this inside printer task!*/
                    if( printheadType == KYOCERA753_OHM || printheadType == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
                        outOfMedia = checkForOutOfMediaHTPrinter();
                    } else {
                        outOfMedia = checkForOutOfMedia();
                        setLowLabelPeelingMinFromHost(0);
                        setLowLabelPeelingMaxFromHost(0);
                        setLowLabelStreamingMinFromHost(0);
                        setLowLabelStreamingMaxFromHost(0);
                        
                        resetLabelLowVars();
                    }

                    if( createHobartWeigherTask( RT_WEIGHER_CS5530_30LBS, (QueueHandle_t)getWeigherQueueHandle() ) == pdTRUE ) {                      
                        createIdleTask();                      
                        /* start created threads */
                        startSystemThreads();                             
                    } else {
                        PRINTF("systemStartup(): Failed to create Weigher task!\r\n");
                        /* force reboot of scale */
                        assert( 0 );                                    
                    }                        
                } else {
                    PRINTF("systemStartup(): Failed to create Printer task!\r\n");  
                    /* force reboot of scale */
                    assert( 0 );                                
                }                      

            }
        }     
    } else if( model_ == GLOBAL_SCALE_HB_WEIGHER ) {
        if( createTranslator( dProperties.subClass, 
                              getWeigherQueueHandle(), getPrinterQueueHandle(),                              
                              getUsbInPrQueueHandle(), getUsbInWrQueueHandle(),
                              getUsbOutPrQueueHandle(), getUsbOutWrQueueHandle() ) == pdTRUE ) {                                    
        

            if( createUsbTask( dProperties.subClass, (QueueHandle_t)getUsbInPrQueueHandle(), 
                             ( QueueHandle_t)getUsbInWrQueueHandle()) == pdTRUE ) {        
                if( createHobartWeigherTask( RT_WEIGHER_CS5530_30LBS, (QueueHandle_t)getWeigherQueueHandle() ) == pdTRUE ) {                      
                    createIdleTask();                      
                    /* start created threads */
                    startSystemThreads();                             
                } else {
                    PRINTF("systemStartup(): Falied to create Weigher task!\r\n");
                    /* force reboot of scale */
                    assert( 0 );                                
                }                        
            }
        }
    } else if( model_ == GLOBAL_SCALE_HB_PRINTER ) {
        PrinterStyle style_ = RT_PRINTER_SERVICE_SCALE_72MM;
        if( createTranslator( dProperties.subClass, 
                              getWeigherQueueHandle(), getPrinterQueueHandle(),                              
                              getUsbInPrQueueHandle(), getUsbInWrQueueHandle(),
                              getUsbOutPrQueueHandle(), getUsbOutWrQueueHandle() ) == pdTRUE ) {                                    
        

            if( createUsbTask( dProperties.subClass, (QueueHandle_t)getUsbInPrQueueHandle(), 
                             ( QueueHandle_t)getUsbInWrQueueHandle()) == pdTRUE ) {        
                if( createHobartPrinterTask( style_, (QueueHandle_t)getPrinterQueueHandle() ) == pdTRUE ) {                      
                    createIdleTask();                      
                    /* start created threads */
                    startSystemThreads();                             
                } else {
                    PRINTF("systemStartup(): Falied to create Weigher task!\r\n");
                    /* force reboot of scale */
                    assert( 0 );                                
                }                        
            }
        }      
    } else if( model_ == GLOBAL_SCALE_AV_WEIGHER ) {
        if( createTranslator( dProperties.subClass, 
                              getWeigherQueueHandle(), NULL,
                              NULL, getUsbInWrQueueHandle(),
                              NULL, getUsbOutWrQueueHandle() ) == pdTRUE ) {                                    
            if( createUsbTask( dProperties.subClass, (QueueHandle_t)NULL, 
                               (QueueHandle_t)getUsbInWrQueueHandle() ) == pdTRUE ) { 
                if( createAveryWeigherTask( RT_WEIGHER_CS5530_30LBS, (QueueHandle_t)getWeigherQueueHandle() ) == pdTRUE ) {                      
                    createIdleTask();                      
                    /* start created threads */
                    startSystemThreads();                             
                } else {
                    PRINTF("systemStartup(): Falied to create Weigher task!\r\n");
                    /* force reboot of scale */
                    assert( 0 );                                
                }                        
            }
        }      
    } else if( model_ == GLOBAL_SCALE_AV_PRINTER ) {
        if( createTranslator( dProperties.subClass,
                              NULL, getPrinterQueueHandle(),
                              getUsbInPrQueueHandle(), NULL,
                              getUsbOutPrQueueHandle(), NULL ) == pdTRUE ) {                                    
            if( createUsbTask( dProperties.subClass, (QueueHandle_t)getUsbInPrQueueHandle(), 
                               (QueueHandle_t)NULL ) == pdTRUE ) { 
                if( createAveryPrinterTask( dProperties.subClass, (QueueHandle_t)getPrinterQueueHandle() ) == pdTRUE ) {                      
                    createIdleTask();                      
                    /* start created threads */
                    startSystemThreads();                             
                } else {
                    PRINTF("systemStartup(): Falied to create Weigher task!\r\n");
                    /* force reboot of scale */
                    assert( 0 );                                
                }                        
            }
        }            
    } else {
        PRINTF( "critical error: unknown model type!\r\n" ); 
        /* force reboot of scale */
        assert( 0 );            
    }           
}

/******************************************************************************/
/*!   \fn BaseType_t createThreadManagerTask( void )

\brief
	This function creates a freeRTOS thread for the thread manager.

\author
	Eric Landes
*******************************************************************************/
BaseType_t createThreadManagerTask( void )
{
    BaseType_t result = xTaskCreate( threadManager,  "Thread Manager", configMINIMAL_STACK_SIZE,
                                  NULL, manager_task_PRIORITY, &managerHandle_ );
#ifdef PRINT_STATS
    statsTimer = xTimerCreate("Stat Timer", pdMS_TO_TICKS(1000), pdTRUE, (void *) 0, statsTimerCallback);
    xTimerStart( statsTimer, 0);
#endif
    return result;
}

/******************************************************************************/
/*!   \fn static void threadManager( void *pvParameters )

\brief
	This function is the thread manager run thread.

\author
	Eric Landes
*******************************************************************************/
static void threadManager( void *pvParameters )
{
    PRINTF("threadManager(): Thread running...\r\n" );
    while( !suspend_ ) {
        ManagerMsg MMsg;
        if( xQueueReceive(managerQHandle_, &MMsg, portMAX_DELAY  ) ) {
            ManagerMsg newMsg;
            memcpy( &newMsg, &MMsg, sizeof(ManagerMsg) );
            handleManagerMsg( &newMsg );
        } else {
            PRINTF("threadManager(): Failed to get manager message from queue!\r\n" );
        }
        taskYIELD();
    }
    vTaskSuspend(NULL);
}

/******************************************************************************/
/*!   \fn static void handleManagerMsg( void *pvParameters )

\brief
    handles messages for the thread manager task

\author
    Eric Landes

*******************************************************************************/
static void handleManagerMsg(ManagerMsg * msg)
{
    
    PRINTF("handleManagerMsg(): New Manager Message. Cmd: %d Task: %d\r\n",msg->cmd, msg->task);
    if( msg->cmd == LIST ) {
        PRINTF("ThreadManager statistics ***************************************\r\n");
        PRINTF("Task list ******************************************************\r\n");
        char buffer[ 40 * TASK_COUNT ] = {0};
        vTaskList( (char *)&buffer );
        PRINTF( buffer );
        PRINTF("****************************************************************\r\n");
        PRINTF("Heap ***********************************************************\r\n");
        int size = xPortGetFreeHeapSize();
        PRINTF("Total heap size: %d\r\n", totalHeap_ );
        PRINTF("Free heap: %d\r\n", size );
        PRINTF("****************************************************************\r\n");
    } else if(msg->task < LAST_TASK ) {
        switch(msg->cmd ) 
        {
            case SUSPEND:
            if( managedTaskHandles_[msg->task] != NULL ) {
                if( ( taskPermissions[msg->task] & SUSPEND_RESUMEMASK ) ) {
                    vTaskSuspend(managedTaskHandles_[msg->task]);
                    PRINTF("handleManagerMsg(): %s Task Suspended\r\n",
                    pcTaskGetName( managedTaskHandles_[msg->task]));
                } else {
                    PRINTF("handleManagerMsg(): Insuffecient privileges to suspend %s \r\n",
                            pcTaskGetName( managedTaskHandles_[msg->task]));
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was not initialized\r\n");
            }
            break;
            case RESUME:
            if( managedTaskHandles_[msg->task] != NULL ) {
                if( ( taskPermissions[msg->task] & SUSPEND_RESUMEMASK ) ) {
                    vTaskResume(managedTaskHandles_[msg->task]);
                    PRINTF("handleManagerMsg(): %s Task Resumed\r\n",
                    pcTaskGetName( managedTaskHandles_[msg->task]));
                } else {
                    PRINTF("handleManagerMsg(): Insuffecient privileges to resume %s \r\n",
                            pcTaskGetName( managedTaskHandles_[msg->task]));
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was not initialized\r\n");
            }
            break;
            case TERMINATE:
            if( managedTaskHandles_[msg->task] != NULL ) {
                if( ( taskPermissions[msg->task] & TEMINATEMASK ) ) {
                    PRINTF("handleManagerMsg(): %s Task Terminated\r\n",
                    pcTaskGetName( managedTaskHandles_[msg->task]));
                    vTaskDelete(managedTaskHandles_[msg->task]);
                    managedTaskHandles_[msg->task] = NULL;
                } else {
                    PRINTF("handleManagerMsg(): Insuffecient privileges to kill %s \r\n",
                            pcTaskGetName( managedTaskHandles_[msg->task]));
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was not initialized\r\n");
            }
            break;
            case CREATE:
            if( managedTaskHandles_[msg->task] == NULL ) {
                if( ( taskPermissions[msg->task] & CREATEMASK ) ) {
                    if( msg->task == T_SENSORS ) {
                        tm_createSensorTask();
                        PRINTF("handleManagerMsg(): %s Task Created\r\n",
                        pcTaskGetName( managedTaskHandles_[msg->task]));
                    }else if(msg->task == T_DOT_WEAR){
                        /* only allow dotwear if best model */
                        #if 1 //as of 7/2025, dont have a "BEST" model (in PCBA resistor config)
                        if( model_ == GLOBAL_SCALE_HB_GT ) {
                          tm_createDotWearTask();
                          PRINTF("handleManagerMsg(): %s Task Created\r\n",
                          pcTaskGetName( managedTaskHandles_[msg->task]));
                        }
                        #endif
                    }
                }else{
                    PRINTF("handleManagerMsg(): Insuffecient privileges to create task\r\n");
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was already created!\r\n");
                char buffer[40* TASK_COUNT] = {0};
                vTaskList((char *)&buffer);
                PRINTF(buffer);
            }
            break;
          case LIST:
            break;
        }
    } else {
        PRINTF("handleManagerMsg(): Invalid Task Handle Command!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn void updateTaskHandle(ManagedThreads task)

      \brief
        This function assigns the transmit queue handle.

      \author
        Aaron Swift
*******************************************************************************/
void updateTaskHandle(ManagedThreads task)
{
    switch (task)
    {
        case T_SENSORS:
            managedTaskHandles_[T_SENSORS] = getSensorsHandle();
            break;
        case T_WEIGHER: 
            managedTaskHandles_[T_WEIGHER] = getWeigherHandle();
            break;
        case T_PRINTER: 
            managedTaskHandles_[T_PRINTER] = getPrinterHandle();
            break;
        case T_USB: 
            managedTaskHandles_[T_USB] = getUsbHandle();
            break;
        case T_TRANSLATOR: 
            managedTaskHandles_[T_TRANSLATOR] = getTranslatorHandle();
            break;
        case T_CUTTER:
            managedTaskHandles_[T_CUTTER] = getCutterHandle();
            break;
        case T_VALUEMAX: 
            managedTaskHandles_[T_VALUEMAX] = getValeuMaxHandle();
            break;      
        case T_DOT_WEAR: 
            managedTaskHandles_[T_DOT_WEAR] = (TaskHandle_t)getDotWearHandle();
            break;
        case T_IDLE:
            managedTaskHandles_[T_IDLE] = getIdleHandle();
            break;          
        case T_UNIT_TEST: 
            managedTaskHandles_[T_UNIT_TEST] = getUnitTestHandle();
            break;
        default:
            PRINTF("handleManagerMsg(): Failed to update task handle!\r\n" );
        break;
    }
}

/******************************************************************************/
/*!   \fn assignThreadManagerMsgQueue( QueueHandle_t pQHandle )

      \brief
        This function assigns the transmit queue handle.

      \author
            Eric Landes
*******************************************************************************/
void assignThreadManagerMsgQueue( QueueHandle_t pQHandle )
{
    managerQHandle_ = pQHandle;
}

#if 0
/******************************************************************************/
/*!   \fn static void startUnitTest( void )

      \brief
        This function creates and starts unit test thread.

      \author
          Aaron Swift
*******************************************************************************/
void startUnitTest( void )
{
    if( createUnitTestTask() ) {
        PRINTF("startUnitTest(): Created Unit test thread.\r\n" );
        if( getCanQueueHandle() != NULL ) {
            assignMsgQueue( getTestQueueHandle() );
        }
        else {
            PRINTF("systemStartup(): CAN Queue handle NULL!\r\n" );
        }
    }
    else {
        PRINTF("systemStartup(): Failed to create Unit test thread!\r\n" );
    }
}
#endif

/******************************************************************************/
/*!   \fn static void startSystemThreads( void )

      \brief
        This function starts the kernel scheduler.

      \author
          Aaron Swift
*******************************************************************************/
static void startSystemThreads()
{
    /* once the threads are started the scheduler should not return unless
    we run out of heap space */
    vTaskStartScheduler();
    while( 1 ) {
        PRINTF("startSystemThreads(): Critical error out of heap space!\r\n");
    }
}

/******************************************************************************/
/*!   \fn unsigned short getSwitch2_ID_Pins(void)
        
      \brief  
        Read Switch 2 (S2 on schematic). This switch may be used for a variety
        of purposes (TBD)
      \author
        Tom Fink
*******************************************************************************/
unsigned short getSwitch2_ID_Pins(void)
{
    unsigned short s2_Pins = 0;
    /* read input pins to determine model type */
    if( GPIO_ReadPinInput( IDF1_GPIO, IDF1_PIN ) ) {
        s2_Pins  += 1;
    }
    if( GPIO_ReadPinInput( IDF2_GPIO, IDF2_PIN )) {
        s2_Pins  += 2;
    }

    //PRINTF("Switch 2 Pins# = %d\r\n", s2_Pins);

    return s2_Pins;    
}


/******************************************************************************/
/*!   \fn static PeripheralModel_t getPeripheralModel( void )

      \brief
        This function returns what type of device we are based on the device 
        properties.

      \author
          Aaron Swift
*******************************************************************************/
static PeripheralModel_t getPeripheralModel( DEVICE_PROPERTIES_t *pDProperties )
{
    PeripheralModel_t model = GLOBAL_SCALE_UNKNOWN;

    if( pDProperties->class == AVERY_WEIGHER_PRINTER ) {
        if( pDProperties->subClass == AV_XPRO ) {
            model = GLOBAL_SCALE_AV_XPRO;
            PRINTF("getPeripheralModel(): GLOBAL_SCALE_AV_XPRO\r\n");           
        } else if( pDProperties->subClass == AV_XONE ) {
            model = GLOBAL_SCALE_AV_XONE;
            PRINTF("getPeripheralModel(): GLOBAL_SCALE_AV_XONE\r\n");      
        } 
    } else if( pDProperties->class == AVERY_WEIGHER ) {
        model = GLOBAL_SCALE_AV_WEIGHER;
        PRINTF("getPeripheralModel(): GLOBAL_SCALE_AV_WEIGHER\r\n");           
    } else if( pDProperties->class == AVERY_PRINTER ) {
        model = GLOBAL_SCALE_AV_PRINTER;
        PRINTF("getPeripheralModel(): GLOBAL_SCALE_AV_PRINTER\r\n");           
    } else if( pDProperties->class == HOBART_WEIGHER_PRINTER ) {
        if( pDProperties->subClass == HB_GT ) {
            model = GLOBAL_SCALE_HB_GT;
            PRINTF("getPeripheralModel(): GLOBAL_SCALE_HB_GT\r\n");                 
        } else if( pDProperties->subClass == HB_DT ) {
            model = GLOBAL_SCALE_HB_DT;
            PRINTF("getPeripheralModel(): GLOBAL_SCALE_HB_DT\r\n");               
        }
    } else if( pDProperties->class == HOBART_WEIGHER ) {
        model = GLOBAL_SCALE_HB_WEIGHER;
        PRINTF("getPeripheralModel(): GLOBAL_SCALE_HB_WEIGHER\r\n");               
    } else if( pDProperties->class == HOBART_PRINTER ) { 
        model = GLOBAL_SCALE_HB_PRINTER;
        PRINTF("getPeripheralModel(): GLOBAL_SCALE_HB_PRINTER\r\n");               
    } else if( pDProperties->class == UN_WEIGHER_PRINTER ) { 
        PRINTF("getPeripheralModel(): Unsupported at this time!\r\n");    
    } else if( pDProperties->class == UN_WEIGHER ) { 
        PRINTF("getPeripheralModel(): Unsupported at this time!\r\n");    
    } else if( pDProperties->class == UN_PRINTER ) { 
        PRINTF("getPeripheralModel(): Unsupported at this time!\r\n");    
    }
    
    if( model == GLOBAL_SCALE_UNKNOWN ) {    
        PRINTF("getPeripheralModel(): Critical error:  Unknown model!\r\n");               
        PRINTF("getPeripheralModel(): pDProperties->class: %d\r\n", pDProperties->class);
        PRINTF("getPeripheralModel(): pDProperties->subClass: %d\r\n", pDProperties->subClass);         
    }
    return model;
}

/******************************************************************************/
/*!   \fn PeripheralModel_t getMyModel( void )

      \brief
        This function returns the peripheral model.

      \author
          Aaron Swift
*******************************************************************************/
PeripheralModel_t getMyModel( void )
{
    return model_;
}

/******************************************************************************/
/*!   \fn static void tm_createSensorTask()

      \brief
        Helper to allow the tread manager to create the sensors task.

      \author
          Eric Landes
*******************************************************************************/
static void tm_createSensorTask(){
    /* configuration management determine what are we? ( FPC, Printer, Weigher ) */
    if( createSensorsTask(false) ) {
    	managedTaskHandles_[T_SENSORS] = (TaskHandle_t)getSensorsHandle();
    } else {
        PRINTF("systemStartup(): Failed to create sensors thread!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn static void tm_createDotWearTask()

      \brief
        Helper to allow the thread manager to create the dot wear task.

      \author
          Eric Landes
*******************************************************************************/
static bool tm_createDotWearTask(){
    /* configuration management determine what are we? ( FPC, Printer, Weigher ) */
    if( createDotWearTask() ) {
    	managedTaskHandles_[T_DOT_WEAR] = (TaskHandle_t)getDotWearHandle();
        return true;
    } else {
        PRINTF("systemStartup(): Failed to create dot wear task!\r\n" );
        return false;
    }
}


#define TIMING_VALUE_uS 300   //k64 @96Mhz clk
/*! ****************************************************************************   
      \fn void delay_uS (unsigned int time)                                                              
 
      \author
          Aaron Swift
*******************************************************************************/ 
void delay_uS( unsigned int time )
{
    unsigned int i, j;
    
    if( time > 0 ) {
      /* the overhead of the function takes about 1uS */
      time--;
    }
    
    for( i = 0; i < time; i++ ) { 
      j = TIMING_VALUE_uS;
      while(j--){}
    }
}

static void showSystemBanner( void )
{
    PRINTF("systemStartup(): Unified Scale Controller RT1024DAG5A\r\n");
    PRINTF("device properties:\r\n");
    
    /* print what class of device we are */
    if( dProperties.class == AVERY_WEIGHER_PRINTER ) {
        PRINTF("device class: Avery Weigher / Printer\r\n");
    } else if( dProperties.class == HOBART_WEIGHER_PRINTER ) {
        PRINTF("device class: Hobart Weigher / Printer\r\n");
    } else if( dProperties.class == AVERY_WEIGHER ) {
        PRINTF("device class: Avery Weigher\r\n");
    } else if( dProperties.class == AVERY_PRINTER ) {
        PRINTF("device class: Avery Printer\r\n");
    } else if( dProperties.class == HOBART_WEIGHER ) {
        PRINTF("device class: Hobart Weigher\r\n");
    } else if( dProperties.class == HOBART_PRINTER ) {
        PRINTF("device class: Hobart Printer\r\n");
    } else if( dProperties.class == UN_WEIGHER_PRINTER ) {
        PRINTF("device class: Unified Weigher / Printer\r\n");
    } else if( dProperties.class == UN_WEIGHER ) {
        PRINTF("device class: Unified Weigher\r\n");
    } else if( dProperties.class == UN_PRINTER ) {
        PRINTF("device class: Unified Printer\r\n");
    } else {
        PRINTF("device class: Unknown! Unified firmware needs loaded\r\n");
    }
    /* print our device sub class */
    if( dProperties.subClass == AV_XPRO ) {
        PRINTF("device sub class: Avery Xpro scale\r\n");
    } else if( dProperties.subClass == AV_XONE ) {
        PRINTF("device sub class: Avery Xone scale\r\n");
    } else if( dProperties.subClass == AV_WEIGHER ) {
        PRINTF("device sub class: Avery Weigher only device\r\n");
    } else if( dProperties.subClass == AV_PRINTER ) {
        PRINTF("device sub class: Avery Printer only device\r\n");
    } else if( dProperties.subClass == HB_GT ) {
        PRINTF("device sub class: Hobart GT scale\r\n");      
    } else if( dProperties.subClass == HB_DT ) {
        PRINTF("device sub class: Hobart DT scale\r\n");            
    } else if( dProperties.subClass == HB_WEIGHER ) {
        PRINTF("device sub class: Hobart Weigher only device\r\n");
    } else if( dProperties.subClass == HB_PRINTER ) {
        PRINTF("device sub class: Hobart Printer only device\r\n");
    } else if( dProperties.subClass == UNIFIED_WEIGHER_PRINTER ) {
        PRINTF("device sub class: Unified scale\r\n");
    } else if( dProperties.subClass == UNIFIED_WEIGHER ) {
        PRINTF("device sub class: Unified Weigher only device\r\n");      
    } else if( dProperties.subClass == UNIFIED_PRINTER ) {
        PRINTF("device sub class: Unified Printer only device\r\n");    
    } else {
        PRINTF("device sub class: Unknown! Unified firmware needs loaded\r\n");    
    }
 
    /* print system info */
    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_PllSys);
    PRINTF( "System clk: %dMhz\r\n", ( freq / 1000000 ) );
    uint32_t clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd0 );
    PRINTF( "Pfd0 clk: %dMhz\r\n", ( clock / 1000000 ) );
    clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd1 );
    PRINTF( "Pfd1 clk: %dMhz\r\n", ( clock / 1000000 ) );
    clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd2 );
    PRINTF( "Pfd2 clk: %dMhz\r\n", ( clock / 1000000 ) );
    clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd3 );
    PRINTF( "Pfd3 clk: %dMhz\r\n", ( clock / 1000000 ) );    
    size_t space = xPortGetFreeHeapSize();
    PRINTF( "Heap allocation: %dK\r\n", ( space / 1000 ) );
    PRINTF( "\r\n" );  
    PRINTF( "\r\n" );  
}          
           
#ifdef PRINT_STATS
void statsTimerCallback( TimerHandle_t xTimer )
{
    ManagerMsg msg;
    msg.cmd = LIST;
    addThreadManagerMsg(&msg);
}
#endif

