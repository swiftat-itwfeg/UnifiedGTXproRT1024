#include "systemTimer.h"
#include "semphr.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"


static TimerHandle_t heartBeat_; 
static SemaphoreHandle_t xMutex_;
unsigned long sTick = 0;

/******************************* avery variables ******************************/
/******************************************************************************/
volatile bool                           usclock1IsrFlag         = false;
static uint32_t                         usclock1counter         = 0;
static uint32_t                         usclock1repeat          = 0;
static usclockCallback_t                usclock1handler         = NULL;
volatile bool                           usclock2IsrFlag         = false;
static uint32_t                         usclock2counter         = 0;
static uint32_t                         usclock2repeat          = 0;
static usclockCallback_t                usclock2handler         = NULL;
volatile bool                           usclock3IsrFlag         = false;
static uint32_t                         usclock3counter         = 0;
static uint32_t                         usclock3repeat          = 0;
static usclockCallback_t                usclock3handler         = NULL;

#define US_CLOCK_FROM_NOW(ticks)  \
  (ticks > 2 ) ? \
    (uint32_t)(GPT_GetCurrentTimerCount(GPT1) + ticks - 2U) : \
    (uint32_t)(GPT_GetCurrentTimerCount(GPT1) + 1)

/******************************************************************************/
/******************************************************************************/


/******************************************************************************/
/*!   \fn void initTimers( void )

      \brief
        This function initializes the systems heartbeat timer.
   
      \author
          Aaron Swift
*******************************************************************************/
void initTimers( void )
{
    heartBeat_ = NULL;
        
    /* create mutex for one shot timers */
    xMutex_ = xSemaphoreCreateMutex(); 
    if( !xMutex_ ) {     
      PRINTF("initTimers(): Failed to create mutex!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn bool createHeartBeatTimer( void )

      \brief
        This function creates a software timer for a heart beat indicator.
   
      \author
          Aaron Swift
*******************************************************************************/
bool createHeartBeatTimer( void )
{
    bool result = false;
    heartBeat_  = xTimerCreate( "heartBeat", (TickType_t)HEARTBEAT, pdTRUE, ( void * ) 0, heartBeatCallback );
    if( heartBeat_ != NULL ) {
        PRINTF("createHeartBeatTimer(): Timer created!\r\n" );
        result = true;
    }
    
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 0, };

    GPIO_PinInit( HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, &config );
    return result;
}

/******************************************************************************/
/*!   \fn bool startHeartBeatTimer( void )

      \brief
        This function starts the software timer.
   
      \author
          Aaron Swift
*******************************************************************************/
bool startHeartBeatTimer( void )
{
    bool result = false;
    if( xTimerStart( heartBeat_, 0 ) == pdPASS ) {
        PRINTF("startHeartBeatTimer(): Timer started!\r\n" );
        result = true;
    }
    return result;
}


/******************************************************************************/
/*!   \fn bool heartBeatCallback( void )

      \brief
        This function software timer callback function.
   
      \author
          Aaron Swift
*******************************************************************************/
void heartBeatCallback( TimerHandle_t heartBeat_ )
{

    /* the number of callbacks is stored in the timer object */
    sTick = ( uint32_t ) pvTimerGetTimerID( heartBeat_ );
    
    sTick++;
    if( sTick % 2 == 0 ) { 
        /* turn off heartbeat LED */
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true);
    } else {
        /* turn on heartbeat LED */
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
    }
    vTimerSetTimerID( heartBeat_, ( void * )sTick );
}

/******************************************************************************/
/*!   \fn unsigned long getSystemTick( void )

      \brief
        This function returns the system tick count.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long getSystemTick( void )
{
    return sTick;
}

/******************************************************************************/
/*!   \fn void uS_Clock_init( void )

      \brief
        This function initializes a microsecond timer using GPT1.

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock_init( void )
{
    gpt_config_t gptConfig;
    
    GPT_GetDefaultConfig( &gptConfig );
    
    gptConfig.enableFreeRun = true;    /* do not reset on compare */
    
    /* intialize the gpt module */
    GPT_Init( GPT1, &gptConfig );    
    GPT_SetClockDivider( GPT1, 32 );

    /* get gpt clock frequency */
    uint32_t freq = CLOCK_GetFreq( kCLOCK_PerClk );

    /* gpt frequency is 1.953Mhz = .512uSec */

    /* disable gpt output compare 1, 2 and 3 interrupts */
    GPT_DisableInterrupts( GPT1, 
                           kGPT_OutputCompare1InterruptEnable 
                         | kGPT_OutputCompare2InterruptEnable 
                         | kGPT_OutputCompare3InterruptEnable );

    GPT_ClearStatusFlags( GPT1, (gpt_status_flag_t)0 );
    
    /* reset channel 1 duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel1, 0 ); 
    GPT_SetOutputOperationMode( GPT1, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );

    /* reset channel 2 duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel2, 0 ); 
    GPT_SetOutputOperationMode( GPT1, kGPT_OutputCompare_Channel2, kGPT_OutputOperation_Disconnected );

    /* reset channel 3 duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel3, 0 ); 
    GPT_SetOutputOperationMode( GPT1, kGPT_OutputCompare_Channel3, kGPT_OutputOperation_Disconnected );

    /* enable interrupt */
    EnableIRQ( GPT1_IRQn );

    /* start Timer */
    GPT_StartTimer( GPT1 );   
}

uint32_t uS_Clock_get( void )
{
    return (uint32_t)(((uint64_t)GPT_GetCurrentTimerCount( GPT1 ) * US_CLOCK_USEC_MULTIPLY) / US_CLOCK_USEC_DIVIDE);
}

/******************************************************************************/
/*!   \fn void usleep(uint32_t usec)

      \brief
        This function use the first timer for suspend execution for microsecond 
        intervals.

      \param [in]    usec  Number of microseconds to wait

      \author
          Philippe Corbes
*******************************************************************************/
void uSleep( uint32_t usec )
{
    /* if required delay is <= 10mS, read the clock until end of sleep */
    if( usec <= 10000 ) {        
        uint32_t now = uS_Clock_get();
        uint32_t end = (now + usec) % (US_CLOCK_USEC_MAX + 1);
        /* wait till the time counter roll down */
        while( now > end ) { 
            now = uS_Clock_get(); 
        }
        /* normal loop */
        while( now < end ) { 
            now = uS_Clock_get();
        }
    } else {
        /* Create a timer and wait the end of timer */
        TimerHandle_t sleepTimer;
        bool sleepTimerFlag = false; 
        sleepTimer = xTimerCreate( "msSleep", 
                               (TickType_t)((usec / 1000) / portTICK_PERIOD_MS), 
                               pdFALSE, 
                               (void *) &sleepTimerFlag, 
                               usleepCallback );
        if( sleepTimer != NULL ) {
            if( xTimerStart( sleepTimer, 0 ) == pdPASS ) {
                while( !sleepTimerFlag ) {
                    taskYIELD();
                }
            }
            xTimerDelete( sleepTimer, 0 );
        }
    }  
}

/******************************************************************************/
/*!   \fn bool usleepTimerCallback( void )

      \brief
        This function software sleep timer callback function.
   
      \author
          Philippe Corbes
*******************************************************************************/
void usleepCallback( TimerHandle_t timerHandle )
{
    bool * flag = ( bool * ) pvTimerGetTimerID( timerHandle );
    *flag = true;  
}

/******************************************************************************/
/*!   \fn void uS_Clock1_start( uint32_t usec )

      \brief
        This function starts the first microsecond timer.

      \param [in]    usec  Number of microseconds before finish
                           The maximum time is 2.199.023.255 microseconds

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock1_start( uint32_t usec )
{
    usclock1counter = US_CLOCK_FROM_NOW( US_CLOCK_USEC_TO_TICKS( usec ) );
    usclock1IsrFlag = false;
    usclock1repeat = 0;

    /* set channel 1 for slt time duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel1, usclock1counter ); 

    /* enable gpt output compare1 interrupt */
    GPT_EnableInterrupts( GPT1, kGPT_OutputCompare1InterruptEnable );  
}

/******************************************************************************/
/*!   \fn void uS_Clock2_start( uint32_t usec )

      \brief
        This function start the second microsecond timer.

      \param [in]    usec  Number of microseconds before finish
                           The maximum time is 2.199.023.255 microseconds

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock2_start( uint32_t usec )
{
    usclock2counter = US_CLOCK_FROM_NOW(US_CLOCK_USEC_TO_TICKS(usec));
    usclock2IsrFlag = false;
    usclock2repeat = 0;

    /* set channel 2 for slt time duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel2, usclock2counter ); 

    /* enable gpt output compare2 interrupt */
    GPT_EnableInterrupts( GPT1, kGPT_OutputCompare2InterruptEnable );
}

/******************************************************************************/
/*!   \fn void uS_Clock3_start(uint32_t usec)

      \brief
        This function start the third microsecond timer.

      \param [in]    usec  Number of microseconds before finish
                           The maximum time is 2.199.023.255 microseconds

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock3_start(uint32_t usec)
{
    usclock3counter = US_CLOCK_FROM_NOW(US_CLOCK_USEC_TO_TICKS(usec));
    usclock3IsrFlag = false;
    usclock3repeat = 0;

    /* set channel 3 for slt time duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel3, usclock3counter ); 

    /* enable gpt output compare3 interrupt */
    GPT_EnableInterrupts( GPT1, kGPT_OutputCompare3InterruptEnable );
}

/******************************************************************************/
/*!   \fn void uS_Clock1_repeat( uint32_t usec )

      \brief
        This function starts the first microsecond timer and automatically repeat.

      \param [in]    usec  Number of microseconds before repeat
                           The maximum time is 2.199.023.255 microseconds

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock1_repeat( uint32_t usec )
{
    usclock1counter = US_CLOCK_FROM_NOW(US_CLOCK_USEC_TO_TICKS(usec));
    usclock1repeat = US_CLOCK_USEC_TO_TICKS(usec);
    usclock1IsrFlag = false;

    /* set channel 1 for slt time duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel1, usclock1counter ); 

    /* enable gpt output compare1 interrupt */
    GPT_EnableInterrupts( GPT1, kGPT_OutputCompare1InterruptEnable );  
}

/******************************************************************************/
/*!   \fn void uS_Clock2_repeat( uint32_t usec )

      \brief
        This function start the second microsecond timer and automatically repeat.

      \param [in]    usec  Number of microseconds before repeat
                           The maximum time is 2.199.023.255 microseconds

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock2_repeat( uint32_t usec )
{
    usclock2counter = US_CLOCK_FROM_NOW(US_CLOCK_USEC_TO_TICKS(usec));
    usclock2repeat = US_CLOCK_USEC_TO_TICKS(usec);
    usclock2IsrFlag = false;

    /* set channel 2 for slt time duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel2, usclock2counter ); 

    /* enable gpt output compare2 interrupt */
    GPT_EnableInterrupts( GPT1, kGPT_OutputCompare2InterruptEnable );
}

/******************************************************************************/
/*!   \fn void uS_Clock3_repeat(uint32_t usec)

      \brief
        This function start the third microsecond timer and automatically repeat.

      \param [in]    usec  Number of microseconds before repeat
                           The maximum time is 2.199.023.255 microseconds

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock3_repeat(uint32_t usec)
{
    usclock3counter = US_CLOCK_FROM_NOW(US_CLOCK_USEC_TO_TICKS(usec));
    usclock3repeat = US_CLOCK_USEC_TO_TICKS(usec);
    usclock3IsrFlag = false;

    /* set channel 3 for slt time duration */
    GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel3, usclock3counter ); 

    /* enable gpt output compare3 interrupt */
    GPT_EnableInterrupts( GPT1, kGPT_OutputCompare3InterruptEnable );
}

/******************************************************************************/
/*!   \fn void uS_Clock1_handler( usclockCallback_t handler )

      \brief
        This function set the first timer's callback function called on timer end.

      \param [in]    handler  Function address

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock1_handler( usclockCallback_t handler )
{
    usclock1handler = handler;  
}

/******************************************************************************/
/*!   \fn void uS_Clock2_handler( usclockCallback_t handler )

      \brief
        This function set the second timer's callback function called on timer end.

      \param [in]    handler  Fonction address

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock2_handler( usclockCallback_t handler )
{
    usclock2handler = handler;
}

/******************************************************************************/
/*!   \fn void uS_Clock3_handler( usclockCallback_t handler )

      \brief
        This function set the third timer's callback function called on timer end.

      \param [in]    handler  Fonction address

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock3_handler( usclockCallback_t handler )
{
    usclock3handler = handler;
}

/******************************************************************************/
/*!   \fn void uS_Clock1_stop( void )

      \brief
        This function stops the timer.

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock1_stop( void )
{
    /* disable gpt output compare1 interrupt */
    GPT_DisableInterrupts( GPT1, kGPT_OutputCompare1InterruptEnable );
    usclock1counter = 0;
    usclock1IsrFlag = true;
    usclock1repeat = 0;  
}

/******************************************************************************/
/*!   \fn void uS_Clock2_stop( void )

      \brief
        This function stop the second timer.

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock2_stop( void )
{
    /* disable gpt output compare2 interrupt */
    GPT_DisableInterrupts( GPT1, kGPT_OutputCompare2InterruptEnable );
    usclock2counter = 0;
    usclock2IsrFlag = true;
    usclock2repeat = 0;
}

/******************************************************************************/
/*!   \fn void uS_Clock3_stop(void)

      \brief
        This function stop the third timer.

      \author
          Philippe Corbes
*******************************************************************************/
void uS_Clock3_stop(void)
{
    /* disable gpt output compare3 interrupt */
    GPT_DisableInterrupts( GPT1, kGPT_OutputCompare3InterruptEnable );
    usclock3counter = 0;
    usclock3IsrFlag = true;
    usclock3repeat = 0;
}

void uSClockIsrHandler( void )
{
    if( kGPT_OutputCompare1Flag == GPT_GetStatusFlags( GPT1, kGPT_OutputCompare1Flag ) ) {
        /* clear interrupt flag.*/
        GPT_ClearStatusFlags( GPT1, kGPT_OutputCompare1Flag );
        if( usclock1counter && GPT_GetCurrentTimerCount(GPT1) >= usclock1counter ) {
            usclock1IsrFlag = true;
            if( usclock1repeat ) {
                /* set next interrupt timer time value*/
                GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel1, US_CLOCK_FROM_NOW(usclock1repeat )); 
                
                /* enable gpt output compare1 interrupt */
                GPT_EnableInterrupts( GPT1, kGPT_OutputCompare1InterruptEnable );
            } else {
                GPT_DisableInterrupts( GPT1, kGPT_OutputCompare1Flag );
                usclock1counter = 0;
            }

            if( usclock1handler != NULL ) {
                usclock1handler();
            }
        }
    }
    
    if( kGPT_OutputCompare2Flag == GPT_GetStatusFlags( GPT1, kGPT_OutputCompare2Flag ) ) {
        /* clear interrupt flag.*/
        GPT_ClearStatusFlags( GPT1, kGPT_OutputCompare2Flag ); 
        if( usclock2counter && GPT_GetCurrentTimerCount(GPT1) >= usclock2counter ) {
            usclock2IsrFlag = true;
            if( usclock2repeat ) {
                /* set next interrupt timer time value*/
                GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel2, US_CLOCK_FROM_NOW(usclock2repeat )); 

                /* enable gpt output compare1 interrupt */
                GPT_EnableInterrupts( GPT1, kGPT_OutputCompare2InterruptEnable );
            } else {
                GPT_DisableInterrupts( GPT1, kGPT_OutputCompare2Flag );
                usclock2counter = 0;
            }

            if( usclock2handler != NULL ) {
                usclock2handler();
            }
        }
    }

    if( kGPT_OutputCompare3Flag == GPT_GetStatusFlags( GPT1, kGPT_OutputCompare3Flag ) ) {
        /* clear interrupt flag.*/
        GPT_ClearStatusFlags( GPT1, kGPT_OutputCompare3Flag );      
        if( usclock3counter && GPT_GetCurrentTimerCount(GPT1) >= usclock3counter ) {
            usclock3IsrFlag = true;
            if( usclock3repeat ) {
                /* set next interrupt timer time value*/
                GPT_SetOutputCompareValue( GPT1, kGPT_OutputCompare_Channel3, US_CLOCK_FROM_NOW(usclock3repeat )); 

                /* enable gpt output compare1 interrupt */
                GPT_EnableInterrupts( GPT1, kGPT_OutputCompare3InterruptEnable );
            } else {
                GPT_DisableInterrupts( GPT1, kGPT_OutputCompare3Flag );
                usclock3counter = 0;
            }

            if( usclock3handler != NULL ) {
                usclock3handler();
            }
        }
    }    
    SDK_ISR_EXIT_BARRIER;    
}