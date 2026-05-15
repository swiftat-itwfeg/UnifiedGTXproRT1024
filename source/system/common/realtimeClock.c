#include "realtimeClock.h"
#include "semphr.h"
#include "fsl_snvs_hp.h"
#include "fsl_debug_console.h"
static SemaphoreHandle_t mutRTC         = NULL;
static SYSTEMTIME_t currTime_;

/* private */
static void timeAndDateDefault( void );
/******************************************************************************/
/*!   \fn void initTimeAndDate( void )

      \brief
        This function initializes our date and time from the realtime clock.

      \author
          Aaron Swift
*******************************************************************************/
bool initTimeAndDate( void )
{
    bool result = false;
    snvs_hp_rtc_config_t rtcConfig;
    snvs_hp_rtc_datetime_t cDateTime_;
    
    mutRTC = xSemaphoreCreateMutex();
    
    if( mutRTC != NULL ) {
        /* intialize rtc */
        SNVS_HP_RTC_GetDefaultConfig( &rtcConfig );
        SNVS_HP_RTC_Init( SNVS, &rtcConfig );

        /*start the rtc timer counter */
        SNVS_HP_RTC_StartTimer( SNVS );

        SNVS_HP_RTC_GetDatetime( SNVS, &cDateTime_ );
        
        /* update our system time */
        currTime_.mDay              = cDateTime_.day;
        currTime_.mMonth            = cDateTime_.month;
        currTime_.mYear             = cDateTime_.year;
        currTime_.mHour             = cDateTime_.hour;
        currTime_.mMinute           = cDateTime_.minute;
        currTime_.mSecond           = cDateTime_.second;
        currTime_.mDayOfWeek        = cDateTime_.day;

        if( cDateTime_.year < 2025) {
            PRINTF("initRTC(): realtime clock needs set!\r\n" );
            timeAndDateDefault();
        } else {
            result = true;
        }
    } else {
    	PRINTF("initRTC(): failed to create mutex!\r\n" );
    }
    return result;  
}

/******************************************************************************/
/*!   \fn void initTimeAndDate( void )

      \brief
        This function return true is real time clock is enabled.

      \author
          Aaron Swift
*******************************************************************************/
bool isRTCEnabled( void )
{
    return (SNVS->HPCR & SNVS_HPCR_RTC_EN_MASK) == SNVS_HPCR_RTC_EN_MASK;
}

/******************************************************************************/
/*!   \fn static void timeAndDateDefault( void )

      \brief
        This function sets the default values for date and time.

      \author
          Aaron Swift
*******************************************************************************/
static void timeAndDateDefault( void )
{
    snvs_hp_rtc_datetime_t cDateTime_;
    if( mutRTC != NULL ) {
        /* is resource available? */
        if( xSemaphoreTake( mutRTC, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {

            memset( (void *)&currTime_, 0, sizeof(SYSTEMTIME_t) );
            /* set our default date time */
            currTime_.mDay       = 1;
            currTime_.mMonth     = 1;
            currTime_.mYear      = atoi (&__DATE__[7]);	// Set to compile time year
            currTime_.mHour      = 0;
            currTime_.mMinute    = 0;
            currTime_.mSecond    = 1;
            currTime_.mDayOfWeek = 6;

            /* convert to rtc structure for write to rtc */
            cDateTime_.minute = currTime_.mMinute;
            cDateTime_.hour = currTime_.mHour;
            cDateTime_.day = currTime_.mDay;
            cDateTime_.month = currTime_.mMonth;
            cDateTime_.year = currTime_.mYear;

            /* set rtc time to default */
            SNVS_HP_RTC_SetDatetime( SNVS, &cDateTime_ );
        } else {
            PRINTF("timeAndDateDefault(): failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("timeAndDateDefault():  mutRTC == NULL!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn bool getSystemTime( SYSTEMTIME_t *pCurrTime )

      \brief
        This function gets the current date and time from the rtc.

      \author
          Aaron Swift
*******************************************************************************/
bool getSystemTime( SYSTEMTIME_t *pCurrTime )
{
    snvs_hp_rtc_datetime_t cDateTime_;
    /* retrieve the curent date time */
    SNVS_HP_RTC_GetDatetime( SNVS, &cDateTime_ );  
    
    /* copy */
    pCurrTime->mDay             = cDateTime_.day;
    pCurrTime->mMonth           = cDateTime_.month;
    pCurrTime->mYear            = cDateTime_.year;
    pCurrTime->mHour            = cDateTime_.hour;
    pCurrTime->mMinute          = cDateTime_.minute;
    pCurrTime->mSecond          = cDateTime_.second;
    
    /* refresh our current time */
    currTime_.mDay              = cDateTime_.day;
    currTime_.mMonth            = cDateTime_.month;
    currTime_.mYear             = cDateTime_.year;
    currTime_.mHour             = cDateTime_.hour;
    currTime_.mMinute           = cDateTime_.minute;
    currTime_.mSecond           = cDateTime_.second;           
    return true;
}

/******************************************************************************/
/*!   \fn bool setSystemTime( SYSTEMTIME_t *st )

      \brief
        This function sets the date and time of the rtc.

      \author
          Aaron Swift
*******************************************************************************/
bool setSystemTime( SYSTEMTIME_t *st )
{
    snvs_hp_rtc_datetime_t cDateTime_;
    
    if( mutRTC != NULL ) {
        /* is resource available? */
        if( xSemaphoreTake( mutRTC, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            cDateTime_.day      = st->mDay;
            cDateTime_.month    = st->mMonth;
            cDateTime_.year     = st->mYear;
            cDateTime_.hour     = st->mHour;
            cDateTime_.minute   = st->mMinute;
            cDateTime_.second   = st->mSecond;
              
            /* set rtc time to default */
            SNVS_HP_RTC_SetDatetime( SNVS, &cDateTime_ );
        }
    }   
}

void showDateTime( void )
{
    if( currTime_.mMonth == 1 )
        PRINTF("January ");  
    if( currTime_.mMonth == 2 )
        PRINTF("Febuary "); 
    if( currTime_.mMonth == 3 )
        PRINTF("March "); 
    if( currTime_.mMonth == 4 )
        PRINTF("April ");       
    if( currTime_.mMonth == 5 )
        PRINTF("May "); 
    if( currTime_.mMonth == 6 )
        PRINTF("June "); 
    if( currTime_.mMonth == 7 )
        PRINTF("July "); 
    if( currTime_.mMonth == 8 )
        PRINTF("August "); 
    if( currTime_.mMonth == 9 )
        PRINTF("September "); 
    if( currTime_.mMonth == 10 )
        PRINTF("October "); 
    if( currTime_.mMonth == 11 )
        PRINTF("November "); 
    if( currTime_.mMonth == 12 )
        PRINTF("December "); 
                    
    PRINTF("%d %d ", currTime_.mDay, currTime_.mYear );  
    if( currTime_.mHour <= 12 ) 
        PRINTF("%d:%d:%d:%d AM\r\n", currTime_.mHour, currTime_.mMinute, currTime_.mSecond, currTime_.mMilliseconds);  
    else 
        PRINTF("%d:%d:%d:%d PM\r\n", ( currTime_.mHour -  12 ), currTime_.mMinute, currTime_.mSecond, currTime_.mMilliseconds);  
  
}
