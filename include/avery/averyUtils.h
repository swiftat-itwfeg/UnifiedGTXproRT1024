#ifndef AVERYUTILS_H
#define AVERYUTILS_H
#include <stdint.h>

/* macros */
/* is x in range between min & max - return true or false */
#define RANGE(x, min, max)	((x) <= (max) && (x) >= (min))		
/* restrict result to limits min & max */
#define LIMIT(x, min, max)	((RANGE(x, max, min)) ? (x) : (((x) > (max)) ? (max) : (min))) 
#define ABS(x)				(x >= 0 ? x : (-x))
/* multiplies by PI, but make sure you use big numbers your precision will be poor */
#define xPI(a)				(((a) * 3142 + 500)/1000)			
/* divides your num by PI, but make sure you use big numbers or your precision will be poor */
#define divPI(a)			(((a) * 1000 + 1571)/3142)			


/* public functions */
uint8_t getBoardRevsion( void );

#endif
