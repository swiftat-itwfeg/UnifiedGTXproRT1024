#ifndef AVERYPRSTATUS_H
#define AVERYPRSTATUS_H
#include <stdint.h>
#include <stdbool.h>
/* status values that go in printStatus */
typedef enum
{
    SPS_POSND_LE_B4_PHEAD,		/* leading edge now positioned a backlash distance from phead */
    SPS_POSND_LE_AT_PHEAD,		/* leading edge now positioned at phead */
    SPS_POSND_TE_AT_PHEAD,		/* leading edge now positioned at phead */
    SPS_POSND_TE_AT_PEEL,		/* now positioned at peel bar */
    SPS_POSND_AT_TEAR,			/* now positioned at tear bar */
    SPS_POSND_LE_AT_SENSOR,		/* now positioned at sensor */
    SPS_POSND_TE_AT_SENSOR,		/* now positioned at sensor */
    SPS_LABEL_FEED_ERROR,		/* label feed err occurred */
    SPS_PAPER_OUT,			/* paper has run out */
    SPS_DATA_TRANSFER_ERROR,	        /* image data was not available when reqd */
    SPS_FINISHED,			/* finished printing */
}PrinterStates_t;
      
typedef struct
{
    bool paperOut; 								
    bool usbError;
    bool feedError;
    bool printerFault;
    bool cashDrawerOpen;
    bool overHeat;
    bool resetOccurred;
    bool commandFail;
    bool labelNotTaken;
    bool mode;
    bool cassetteDoorOpen;
    unsigned char percentMediaLeft;
    char temperature;
    unsigned char takeUpDiameter;
    unsigned char protocol[2];
}PrintStatus_t;

typedef struct
{
    unsigned char maxDotRes;
    unsigned char cassettePrintheadWidth[2]; 		
    unsigned char cassettePrintheadDotsPerMm; 		
}ExtraStatus_t;
      
void initPrinterState( void );
uint32_t getPrinterState( void );
void setPrinterState( PrinterStates_t state );
#endif
