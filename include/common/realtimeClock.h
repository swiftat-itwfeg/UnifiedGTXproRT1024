#ifndef REALTIMECLCOCK_H
#define REALTIMECLCOCK_H
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"

typedef struct
{
        uint16_t mYear;				// Actual year i.e. 1998
        uint8_t	mMonth;				// January = 1
        uint8_t	mDayOfWeek;			// Sunday = 0
        uint8_t	mDay;				// Current day of month (1-31)
        uint8_t	mHour;				// The current hour
        uint8_t	mMinute;			// The current minute
        uint8_t	mSecond;			// The current second
        uint8_t	mMilliseconds;		        // The current milliseconds x 10 (1-99)
}SYSTEMTIME_t;

/* public */
bool initTimeAndDate( void );
bool getSystemTime( SYSTEMTIME_t *pCurrTime );
bool setSystemTime( SYSTEMTIME_t *st );
void showDateTime( void );
#endif
