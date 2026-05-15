#ifndef SYSTEMTIMER_H
#define SYSTEMTIMER_H
#include "FreeRTOS.h"
#include "timers.h"
#include "board.h"
#include <stdbool.h>


/* Timer callback functions execute in the context of the timer service task. 
    It is therefore essential that timer callback functions never attempt to block. 
    For example, a timer callback function must not call vTaskDelay(), 
    vTaskDelayUntil(), or specify a non zero block time when 
    accessing a queue or a semaphore. 
*/ 

/******************************* hobart defines *******************************/
/******************************************************************************/
#define HEARTBEAT       150


typedef struct
{
    int id;
    TimerHandle_t handle;
    TickType_t startTime;
    bool timeout;
}OneShot;

#define MAX_LIST_DEPTH  10
#define MUTEX_TIMEOUT   50      /*~  50mSec */
/******************************************************************************/
/******************************************************************************/

/******************************* avery defines *******************************/
/******************************************************************************/
#define uSClockIsrHandler GPT1_IRQHandler

#define US_CLOCK_USEC_MULTIPLY    64ull
#define US_CLOCK_USEC_DIVIDE      125ull

/* Minimum time to wait in microseconds */
#define US_CLOCK_USEC_MIN       (uint32_t)(2ull)

/* Maximum time to wait in microseconds */
#define US_CLOCK_USEC_MAX       (uint32_t)(0xffffffffull * US_CLOCK_USEC_MULTIPLY / US_CLOCK_USEC_DIVIDE)

/* Convert the microseconds to value stored in the OCR register */
#define US_CLOCK_USEC_TO_TICKS(usec) ((uint64_t)usec * US_CLOCK_USEC_DIVIDE / US_CLOCK_USEC_MULTIPLY)

/*! @brief uS_Clock interrupt callback function. */
typedef void (*usclockCallback_t)(void);

/******************************************************************************/
/******************************************************************************/


/************************** common public functions ***************************/
/******************************************************************************/
void initTimers( void );
bool createHeartBeatTimer( void );
bool startHeartBeatTimer( void );
void heartBeatCallback( TimerHandle_t heartBeat_ );
/******************************************************************************/
/******************************************************************************/


/************************** avery public functions ****************************/
/******************************************************************************/
void uS_Clock_init( void );
uint32_t uS_Clock_get( void );
void uSleep( uint32_t usec );
void usleepCallback( TimerHandle_t timerHandle );
void uS_Clock1_start( uint32_t usec );
void uS_Clock2_start( uint32_t usec );
void uS_Clock3_start( uint32_t usec );
void uS_Clock1_repeat( uint32_t usec );
void uS_Clock2_repeat( uint32_t usec );
void uS_Clock3_repeat( uint32_t usec );
void uS_Clock1_handler( usclockCallback_t handler );
void uS_Clock2_handler( usclockCallback_t handler );
void uS_Clock3_handler( usclockCallback_t handler );
void uS_Clock1_stop( void );
void uS_Clock2_stop( void );
void uS_Clock3_stop( void );
/******************************************************************************/
/******************************************************************************/
#endif
