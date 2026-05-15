/*
 * diagnostics.h
 *
 *  Created on: Jul 31, 2025
 *      Author: SWIFTAT
 */

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H
#include "averyProtocol.h"
#include <stdbool.h>
   
#define DIAGLEN				1000
#define MARKS_PER_SECTION		16
#define NUM_SECTIONS			7
#define NUM_CUTTER_SPEED_PROFILE_MARKS	( NUM_SECTIONS * MARKS_PER_SECTION )

#define LENGTH(a)			( sizeof( a ) / sizeof( a[0] ) )
   
typedef struct
{
	unsigned long setSpeed;
	long labelLength;
	long labelGapLength;
	bool continuousPrint;

	unsigned long gapSenseMin;
	unsigned long gapSenseMid;
	unsigned long gapSenseMax;
	unsigned long lookingForFirstEdge;
	long onLabelCount;
	long inGapCount;

	unsigned long mediaSenseMin;
	unsigned long mediaSenseMid;
	unsigned long mediaSenseMax;
	unsigned long mediaDiameter;
	long mediaAveLittleDips;
	long mediaAveLargeDips;

	long printStatus;

	long stepCount;				/* incremented every step */
	unsigned char gapSense[DIAGLEN];	/* gap sense/4 for last 1000 steps */
	unsigned char tuAngle[DIAGLEN];		/* tu sense/4 for last 1000 steps */
	unsigned char mmSpeed[DIAGLEN];		/* main motor speed (mm/s) for last 1000 steps */
	unsigned char tuSpeed[DIAGLEN];		/* take up motor speed (mm/s) for last 1000 steps */
	unsigned char mediaSense[DIAGLEN];	/* reading from media sensor/4 for last 1000 steps */
	unsigned char dotResistance[448];					/* dot resistance in 10 ohm increments */
	unsigned short cutterSpeedProfile[NUM_CUTTER_SPEED_PROFILE_MARKS];	/* main motor speed (mm/s) for last 1000 steps */
}DebugData_t;

typedef struct
{
	unsigned long elementSize;
	unsigned long numElements;
	char* name;
	unsigned char field;
}DIAG_DESC;

typedef enum
{
    DIAG_COUNT,
    DIAG_GET
}DIAD_ACTION;

typedef struct {
    char		buf[ 2 * PACKET_DATA_SIZE ];
    DIAD_ACTION	action;
    unsigned short	numBlks;
    unsigned short	blkNumReq;
    unsigned short	numBlk;
    unsigned char	line;
    unsigned long	loop;
}DIAG_CONTEXT;

typedef enum
{
    DIAG_SETSPEED,			/* speed set to (steps/sec) */
    DIAG_MMSPEED,			/* (steps/sec) */
    DIAG_TUSPEED,			/* (steps/sec) */

    DIAG_LABELLENGTH,			/* (steps) */
    DIAG_LABELGAPLENGTH,	        /* (steps) */
    DIAG_GAPLENGTH,			/* (steps) */
    DIAG_ONLABELCOUNT,			/* steps since beginning of label */
    DIAG_INGAPCOUNT,			/* steps since beginning of gap */
    DIAG_STEPCOUNT,			/* steps since power up */

    DIAG_CONTINUOUSPRINT,	        /* true or false */
    DIAG_TEMPERATURE,			/* return temperature (degrees) */
    DIAG_CASSETTEOPEN,			/* true/false */
    DIAG_PRINTSTATUS,

    DIAG_GAPSENSEMIN,			/* 0 - 255 */
    DIAG_GAPSENSEMID,			/* 0 - 255 */
    DIAG_GAPSENSEMAX,			/* 0 - 255 */
    DIAG_GAPSENSEBIAS,			/* (uAmps) */
    DIAG_MINLABELEDGEDIFF,	        /* 0 - 255 */
    DIAG_GAPSENSE,			/* 0 - 255 */

    DIAG_TUANGLE,			/* (degrees) */
    DIAG_TUDIAMETER,			/* (mm) */
    DIAG_TUBIASUAMPS,			/* (uAmps) */
    DIAG_TUBIASMVOLTS,			/* (mVolts) */
    DIAG_TUSPAN,			/* (raw adc) */

    DIAG_LBLTKNSENSE,			/* (raw adc) */
    DIAG_LBLTKNTHRESHOLD,		/* (raw adc) */
    DIAG_LBLTKNMODE,			/* ON or OFF (ie 1 or 2 respectively) */

    DIAG_MEDIASENSEBIAS,		/* (uAmps) */

    DIAG_PHEADDOTS,			/* (in 10 ohm increments) */

    DIAG_MEDIASENSEMIN,			/* (raw adc) */
    DIAG_MEDIASENSEMID,			/* (raw adc) */
    DIAG_MEDIASENSEMAX,			/* (raw adc) */
    DIAG_MEDIASENSE,			/* (raw adc) */
    DIAG_MEDIADIAMETER,			/* mm */
    DIAG_MEDIAAVELITTLEDIPS,	        /* steps */
    DIAG_MEDIAAVELARGEDIPS,		/* steps */

    DIAG_PCBREV,			/* revision of the PCB */
    DIAG_CUTTERSPEEDPROFILE	
} DIAG_ENUM;

void diagInit( DIAG_CONTEXT *pDiag );
bool diagIsFull( DIAG_CONTEXT *pDiag, bool full );
void diagCountBlks( DIAG_CONTEXT *pDiag );
unsigned short diagUpdateNumBlk( DIAG_CONTEXT *pDiag );
void diagBuildMessage( DIAG_CONTEXT *pDiag );
#endif
