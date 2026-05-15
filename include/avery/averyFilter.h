#ifndef AVERYFILTER_H
#define AVERYFILTER_H
#include "stdint.h"

#define DEFAULT_FILTER_IDX 2
#define MAX_FILTER_IDX     3

/* The size of the filter control is dependant on the filter descriptor
 * and the largest array required will be for the filter with the most stages
 * Thus Filter_0_25Hz is made up of:
 *
 * None Filter Control = 4 * 12 = 48
 * Filter Control (FDIV & FSHFT ) = 4 * 8  = 32
 * Filter Control (FDEC) = 1 * 8  = 8
 * Filter Control (FEND) = 1 * 4  = 4

 * Making a total of 48 + 32 + 8 + 4 = 92 Bytes required in total
 * For a int32_t array this means we need 92/4 = 23 elements (Rounded gives 24)
 */
#define MAX_FILTER_CONTROL 24

/* The length of this array is gonverned by the total stage length for the
 * largest filter
 * Thus for Filter_0_25Hz, we require 60 + 45 + 82 + 61 = 248 elements
 */
#define MAX_FILTER_STORES 248

/* The length of this array is gonverned by the total stage length for the
 * largest filter
 * Thus for Filter_0_25Hz, we require 60 + 45 + 82 + 61 = 248 elements
 */
#define STD_FILTER_OPTION_MASK       0x07
#define SECOND_STAGE_FILTER_TRIGGER1 1500
#define SECOND_STAGE_FILTER_TRIGGER2 150
#define SECOND_STAGE_FILTER_LEN      8

enum
{
    FILT_ADC,
    FILT_INCX,
    FILT_INCY,
    MAX_FILTERS
};

/* filter opcodes */
#define FDEC  (int8_t) - 4
#define FSHFT (int8_t) - 3
#define FDIV  (int8_t) - 2
#define FEND  (int8_t) - 1


typedef struct
{
    int32_t filterControl[MAX_FILTERS][MAX_FILTER_CONTROL];     /* IIR filters for weight and tilt */                                                       
    int32_t filterStore[MAX_FILTERS][MAX_FILTER_STORES];    
}USBRAM;

void initFilter( uint8_t number, uint8_t type );
uint8_t averageFilter( uint8_t number, int32_t in, int32_t *pOut );
uint16_t getFilterOutputLimit( void );
#endif