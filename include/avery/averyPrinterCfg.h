#ifndef AVERYPRINTERCFG_H
#define AVERYPRINTERCFG_H
#include <stdbool.h>

#define TAKEN_SENSOR_THRESHOLD			0		
#define TK_UP_SENSOR_BIAS_MVOLTS		1200


#if 0   /* original */

typedef	struct
{

	


;

	unsigned char gMediaSensorBias;		// default = 0;


	long          gSetLabelGapLength;	// default = MM_TO_DOTS(0);
	unsigned char gPrinterId;			// just an id for this printer
	unsigned char gPaperOutThreshold;	// percentage at which we flash LEDs to warn of imminent paper out

} NV;



#endif
typedef struct
{
    unsigned short sensorToPheadDistance;       /* distance in tenth mm */
    unsigned char gapCalVal;                    /* default = 0 */
    unsigned char gapSensorBackingVal;          /* analog comparator when backing paper is detected */
    unsigned char gapSensorLabelVal;            /* analog comparator when backing and label paper is detected */    
    unsigned char mediaSensorCalVal;   
    unsigned char paperDetectVal;               /* percentage at which we flash LEDs to warn of imminent paper out */
    unsigned char minLabelEdgeDiff;             /* the min diff between max & min gap sense reading that constitutes an edge */
    unsigned char labelPrintDensity;
    unsigned char receiptPrintDensity;
    bool labelTakenEnabeld;                     /* default = true */
    unsigned char labelCalVal;
    unsigned long gTakenSensorThreshold;        /* deafault = */
    unsigned long gTakeUpRelaxedCounts;         /* counts read from the take up sensor ADC when under no tension */
    unsigned long gTakeupSpan;
    unsigned char gTakeupBias;
    unsigned char gPrinterId;
    long labelGapLength;                        /* length is in #lines */
}PRCfg_t;

#endif