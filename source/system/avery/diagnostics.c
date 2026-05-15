#include "diagnostics.h"
#include "averyPrinterCfg.h"
#include <stdio.h>
#include <string.h>
#include "printHead.h"
#include "vendor.h"

static          DebugData_t            debugData;
uint32_t        debugIdx               = 0;

extern PRCfg_t *pCfg;

const DIAG_DESC diagDescTbl[] = {
		/* type	no.				name					field */
		4,	1,				(char *)"setSpeed",			DIAG_SETSPEED,
		1,	DIAGLEN,		        (char *)"mmSpeed",			DIAG_MMSPEED,
		1,	DIAGLEN,		        (char *)"tuSpeed",			DIAG_TUSPEED,

		4,	1,				(char *)"labelLength",			DIAG_LABELLENGTH,		/* (steps) */
		4,	1,				(char *)"labelGapLength",		DIAG_LABELGAPLENGTH,	        /* (steps) */
		4,	1,				(char *)"setGapLength",	 		DIAG_GAPLENGTH,			/* (steps) */
		4,	1,				(char *)"onLabelCount",	 		DIAG_ONLABELCOUNT,
		4,	1,				(char *)"inGapCount",			DIAG_INGAPCOUNT,
		4,	1,				(char *)"stepCount",			DIAG_STEPCOUNT,

		4,	1,				(char *)"continuousPrint",		DIAG_CONTINUOUSPRINT,
		4,	1,				(char *)"temperature",			DIAG_TEMPERATURE,
		4,	1,				(char *)"cassetteOpen",			DIAG_CASSETTEOPEN,
		4,	1,				(char *)"printStatus",			DIAG_PRINTSTATUS,

		4,	1,				(char *)"gapSenseMin",			DIAG_GAPSENSEMIN,
		4,	1,				(char *)"gapSenseMid",			DIAG_GAPSENSEMID,
		4,	1,				(char *)"gapSenseMax",			DIAG_GAPSENSEMAX,
		4,	1,				(char *)"gapSenseBiasUAmps",	        DIAG_GAPSENSEBIAS,
		1,	1,				(char *)"gapSenseMinDiff",		DIAG_MINLABELEDGEDIFF,
		1, 	DIAGLEN,		        (char *)"gapSense",			DIAG_GAPSENSE,

		1,	DIAGLEN,		        (char *)"tuAngle",			DIAG_TUANGLE,

		4,	1,				(char *)"tuDiameter",			DIAG_TUDIAMETER,		/* (mm) */
		4,	1,				(char *)"tuBiasUAmps",	 		DIAG_TUBIASUAMPS,		/* (uAmps) */
		4,	1,				(char *)"tuBiasMVolts",		 	DIAG_TUBIASMVOLTS,		/* (mVolts) */
		4,	1,				(char *)"tuSpan",			DIAG_TUSPAN,		        /* (raw adc) */

		4,	1,				(char *)"lblTknSense",			DIAG_LBLTKNSENSE,		/* (raw adc) */
		4,	1,				(char *)"lblTknThreshold",		DIAG_LBLTKNTHRESHOLD,	        /* (raw adc) */
		4,	1,				(char *)"lblTknMode",			DIAG_LBLTKNMODE,		/* on or off */

		4,	1,				(char *)"mediaSenseMin",		DIAG_MEDIASENSEMIN,
		4,	1,				(char *)"mediaSenseMid",		DIAG_MEDIASENSEMID,
		4,	1,				(char *)"mediaSenseMax",		DIAG_MEDIASENSEMAX,
		4,	1,				(char *)"mediaSenseBias",		DIAG_MEDIASENSEBIAS,	        /* (uAmps) */
		1,	DIAGLEN,		        (char *)"mediaSense",			DIAG_MEDIASENSE,
		4,	1,				(char *)"mediaDiameter",		DIAG_MEDIADIAMETER,
		4,	1,				(char *)"mediaAveLittleDips",	        DIAG_MEDIAAVELITTLEDIPS,
		4,	1,				(char *)"mediaAveLargeDips",	        DIAG_MEDIAAVELARGEDIPS,

		1,	PRINTER_HEAD_SIZE_56MM,         (char *)"pheadDots",		        DIAG_PHEADDOTS, 	        /* in 10 ohm increments */

		1,	1,				(char *)"pcbRevision",			DIAG_PCBREV,

		1,	NUM_CUTTER_SPEED_PROFILE_MARKS, (char *)"cutterSpeedProfile",           DIAG_CUTTERSPEEDPROFILE,
};

DebugData_t *getDebugData( void ) { return &debugData; }

void diagInit( DIAG_CONTEXT *pDiag )
{
	pDiag->action = DIAG_GET;
	pDiag->blkNumReq =0;
	pDiag->numBlk = 0;
	pDiag->line = 0;
	pDiag->loop = 0;	/* start from the begining of the next array */
	memset( pDiag->buf, 0, sizeof( pDiag->buf ) );
}

bool diagIsFull( DIAG_CONTEXT *pDiag, bool full )
{
    if( pDiag->action == DIAG_COUNT ) {
        diagCountBlks( pDiag );
    }else if( !full ) {
        full = ( diagUpdateNumBlk( pDiag ) == pDiag->blkNumReq );
    }
    return full;
}

void diagCountBlks( DIAG_CONTEXT *pDiag )
{
    if( strlen( pDiag->buf ) > PACKET_DATA_SIZE ) {
        /* slide left the buffer */
        memmove( pDiag->buf, &pDiag->buf[PACKET_DATA_SIZE], sizeof( pDiag->buf ) - PACKET_DATA_SIZE );
        memset( &pDiag->buf[ sizeof( pDiag->buf ) - PACKET_DATA_SIZE ], 0, PACKET_DATA_SIZE );
        
        pDiag->numBlks++;
    }
}

unsigned short diagUpdateNumBlk( DIAG_CONTEXT *pDiag )
{
    unsigned short next = pDiag->numBlk + ( strlen(pDiag->buf) / PACKET_DATA_SIZE );

    if( next > pDiag->numBlk ) {
        if ( ( pDiag->blkNumReq > 1 ) && ( ( pDiag->blkNumReq - pDiag->numBlk ) > 1 ) ){
            diagCountBlks( pDiag );
        } else {
            pDiag->numBlk = next;
        }
    }
    return( pDiag->numBlk );
}

void diagBuildMessage( DIAG_CONTEXT *pDiag )
{
    bool full = false;
    //PRCfg_t *pCfg = getPrinterConfig();
    
    if( pDiag->blkNumReq == 1 ) {
        /* setup first block */
        strcpy( pDiag->buf, "<printerDiagnostics>\n" );
    }

    while( pDiag->line < LENGTH(diagDescTbl) && !full ) {
        unsigned long w = 0;
        unsigned char* p = 0;
        unsigned long* pU32 = 0;
        unsigned short* pU16 = 0;
        unsigned long numElements = diagDescTbl[pDiag->line].numElements;

        /* send PRINTHEAD_DOTS_56MM elements if 56mm printhead */
        if( diagDescTbl[pDiag->line].field == DIAG_PHEADDOTS && HEAD_DOTS_56MM == HEAD_DOTS_56MM ) {
            numElements = HEAD_DOTS_56MM;
        }

        /* find data to send */
        switch( diagDescTbl[pDiag->line].field )
        {
            case DIAG_SETSPEED:			w = debugData.setSpeed;					break; 	/* speed set to (steps/sec) */
            case DIAG_MMSPEED:			p = debugData.mmSpeed;					break; 	/* (steps/sec) */
            case DIAG_TUSPEED:			p = debugData.tuSpeed;					break; 	/* (steps/sec) */

            case DIAG_LABELLENGTH:		w = debugData.labelLength;				break; 	/* (steps) */
            case DIAG_LABELGAPLENGTH:	        w = debugData.labelGapLength;			        break; 	/* (steps) */
            case DIAG_ONLABELCOUNT:		w = debugData.onLabelCount;				break; 	/* steps since beginning of label */
            case DIAG_INGAPCOUNT:		w = debugData.inGapCount;				break; 	/* steps since beginning of gap */
            case DIAG_STEPCOUNT:		w = debugData.stepCount;				break; 	/* steps since power up */

            case DIAG_CONTINUOUSPRINT:	        w = debugData.continuousPrint;			        break; 	/* true or false */
#if 0       /* TO DO: finish */            
            case DIAG_TEMPERATURE:		w = getTempInDegC(); 					break;	/* return temperature (degrees) */
#else
            case DIAG_TEMPERATURE:		w = 22;
#endif            
            case DIAG_CASSETTEOPEN:		w = false;						break; 	/* X-one no way to detect */
            case DIAG_PRINTSTATUS:		w = debugData.printStatus;				break;

            case DIAG_GAPSENSEMIN:		w = debugData.gapSenseMin;				break; 	/* 0 - 255 */
            case DIAG_GAPSENSEMID:		w = debugData.gapSenseMid;				break; 	/* 0 - 255 */
            case DIAG_GAPSENSEMAX:		w = debugData.gapSenseMax;				break; 	/* 0 - 255 */
            case DIAG_GAPSENSEBIAS:		w = pCfg->gapCalVal;					break; 	/* %duty cycle */
            case DIAG_MINLABELEDGEDIFF: 	w = pCfg->minLabelEdgeDiff;			        break; 	/* 0 - 255 */
            case DIAG_GAPSENSE:			p = debugData.gapSense;					break; 	/* 0 - 255 */
            case DIAG_GAPLENGTH:		w = pCfg->labelGapLength;			        break; 	/* (steps) */
            /* x-one has no take-up sensor */
            case DIAG_TUANGLE:			p = debugData.tuAngle;					break; 	/* (degrees) */
            case DIAG_TUDIAMETER:		w = 0;						        break; 	/* (mm) */
            case DIAG_TUBIASUAMPS:		w = 0;							break; 	/* (uAmps) */
            case DIAG_TUBIASMVOLTS:		w = 0;							break; 	/* (mVolts) */
            case DIAG_TUSPAN:			w = 0;							break; 	/* (raw adc) */
#if 0       /* TO DO: finish */
            case DIAG_LBLTKNSENSE:		w = isLabelPresent();					break; 	/* true or false */
#else
            case DIAG_LBLTKNSENSE:		w = false;
#endif            
            case DIAG_LBLTKNTHRESHOLD:	        w = 0;							break; 	/* TO DO: finish */
            case DIAG_LBLTKNMODE:		w =(pCfg->labelTakenEnabeld==true)?1:0;                 break; 	/* return 1 or 0 for 1 (ie on) or 2 (ie off) */

            case DIAG_MEDIASENSEMIN:	        w = debugData.mediaSenseMin;			        break; /* raw adc */
            case DIAG_MEDIASENSEMID:	        w = debugData.mediaSenseMid;			        break; /* raw adc */
            case DIAG_MEDIASENSEMAX:	        w = debugData.mediaSenseMax;			        break; /* raw adc */
            case DIAG_MEDIASENSEBIAS:	        w = pCfg->mediaSensorCalVal * 100;		        break; /* (uAmps) */
            case DIAG_MEDIASENSE:		p = debugData.mediaSense;				break; /* raw adc */
            case DIAG_MEDIADIAMETER:	        w = debugData.mediaDiameter;			        break; /* mm */

            case DIAG_PHEADDOTS:		p = debugData.dotResistance;			        break; /* (in 10 ohm increments) */

            case DIAG_PCBREV:			w = getAssetNumber();			                break; /* pcb revision */

            case DIAG_CUTTERSPEEDPROFILE: pU16 = debugData.cutterSpeedProfile;	                        break;
        }

        /* multiple elements? */
        if (numElements > 1)
        {
            /* loop around converting values into comma separated strings */
            while( pDiag->loop < numElements && !full ) {
                int idx;
                char *d = &pDiag->buf[strlen(pDiag->buf)];

                /* where num elements is DIAGLEN, the buffer is indexed by debugIdx & will have finished
                   where ever that indicates at the end of print, so we shift the index so the last reading
                   ends up as the last reading in the data sent */
                if( numElements == DIAGLEN ) {
                    idx = ( pDiag->loop + debugIdx ) % DIAGLEN;
                } else {
                        /* just send idx 0 to numElements, in that order */
                        idx = pDiag->loop;
                }

                
                /* dump data */
                if( p != 0 ) {	                
                    if( pDiag->loop == 0 ) {
                        #if defined(DIAG_ARRAY_VALUES_PER_LINE) && (DIAG_ARRAY_VALUES_PER_LINE > 0)
                        sprintf(d, "<d name=\"%s\">\n%d", diagDescTbl[diag->line].name, p[idx]);
                    }
                    else if( ( diag->loop %DIAG_ARRAY_VALUES_PER_LINE ) == 0 )
                    {
                        sprintf(d, ",\n%d", p[idx]);
                        #else
			sprintf(d, "<d name=\"%s\"> %d", diagDescTbl[pDiag->line].name, p[idx]);
                        #endif
                    } else {
                        sprintf(d, ", %d", p[idx]);
                    }
                } else if( pU16 != 0 ) {	
                    /* dump 16bits data */				
                    if( pDiag->loop == 0 ) {
                        #if defined(DIAG_ARRAY_VALUES_PER_LINE) && (DIAG_ARRAY_VALUES_PER_LINE > 0)
                        sprintf(d, "<d name=\"%s\">\n%d", diagDescTbl[pDiag->line].name, pU16[idx]);
                    } else if( ( pDiag->loop %DIAG_ARRAY_VALUES_PER_LINE ) == 0 ) {
                        sprintf(d, ",\n%d", pU16[idx]);
                        #else
			sprintf(d, "<d name=\"%s\"> %d", diagDescTbl[pDiag->line].name, pU16[idx]);
                        #endif
                    } else {
                        sprintf(d, ", %d", pU16[idx]);
                    }
                } else if( pU32 != 0 ) {	
                    /* dump 32bits data */ 
                    if( pDiag->loop == 0 ) {
                        #if defined(DIAG_ARRAY_VALUES_PER_LINE) && (DIAG_ARRAY_VALUES_PER_LINE > 0)
			sprintf(d, "<d name=\"%s\">\n%d", diagDescTbl[pDiag->line].name, pU32[idx]);
                    } else if( ( diag->loop %DIAG_ARRAY_VALUES_PER_LINE ) == 0 ) {
                        sprintf(d, ",\n%d", pU32[idx]);
                        #else
                        sprintf(d, "<d name=\"%s\"> %d", diagDescTbl[pDiag->line].name, pU32[idx]);
                        #endif
                    } else {
                        sprintf(d, ", %d", pU32[idx]);
                    }
                }
                full = diagIsFull(pDiag, full);
                pDiag->loop++;
            }

            if( pDiag->loop == numElements /*&& !full*/ ) {
                //...put end marker in - nb we deliberately blat it over the trailing comma
                strcpy(&pDiag->buf[strlen(pDiag->buf)], "</d>\n");
                full = diagIsFull(pDiag, full);
                pDiag->line++;
                pDiag->loop = 0;	// Restart from the begining of the next array
            }
        } else {
            /* a single element */
            sprintf(&pDiag->buf[strlen(pDiag->buf)], "<d name=\"%s\">%d</d>\n", diagDescTbl[pDiag->line].name, w);
            full = diagIsFull(pDiag, full);
            pDiag->line++;
        }
    } 

    if( pDiag->line == LENGTH(diagDescTbl) && !full ) {
        if( pDiag->action == DIAG_COUNT ) {
            /* slide the buffer left to leave any data that overflows this block */
            diagIsFull( pDiag, full );
            /* if data has overflowed this block then add a penultimate block */
            if( strlen(pDiag->buf) > 0 ) {
                    pDiag->numBlks++;
            }
            /* add another block for last block "</printerDiagnostics>\n" */
            pDiag->numBlks++; 
        } else if (pDiag->blkNumReq == pDiag->numBlks) {
          /* DIAG_GET only */
          /* setup last block, which is always "</printerDiagnostics>\n" */
          strcpy(&pDiag->buf[strlen(pDiag->buf)], "</printerDiagnostics>\n");
        }
    }
}
