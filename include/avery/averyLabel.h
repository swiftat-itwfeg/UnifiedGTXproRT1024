#ifndef AVERYLABEL_H
#define AVERYLABEL_H
#include "averyProtocol.h"

/* label position definitions */
#define DOTS_PER_MM					8				                /* dots per mm - do not use this macro in the code.  Use those below */
#define DOTS_TO_MM(dots)				((dots) / DOTS_PER_MM)
#define MM_TO_DOTS(mm)					((mm) * DOTS_PER_MM)

#define NOMINAL_PHEAD_RESISTANCE	                80					        /* 800 ohms */
#define DEFAULT_SPEED					MM_TO_DOTS(DEFAULT_SPEED_MM)		        /* steps/sec */

#define MIN_SPEED					20					        /* steps/sec */
#define BRAKING_DISTANCE				10					        /* steps */
#define TU_BRAKING_DISTANCE				(BRAKING_DISTANCE/4)
#define TU_SPEED_DIF_CONST                              (DEFAULT_SPEED) 

#define ACCELERATION_CONSTANT			        30					        /* bigger numbers means slower acceleration (minimum 24, because divided by 3 and DOTS_PER_MM(=8) */
#define TU_ACCELERATION_CONSTANT		        (ACCELERATION_CONSTANT / 3)

/*need to take up slop when switching from backwind to print */
#define TAKEUP_BACKLASH_DIS				( MM_TO_DOTS( 7 ) )			        /* in mm */
#define STEPS_TO_SLACKEN_TAKE_UP		        ( 4200 / gTakeUpDiameter )		        /* 140 steps for empty or 70 for full.  Used to be 75 */

#define MAX_LABEL_LEN_MM				300					        /* len in mm */
#define MAX_LABEL_LEN					( MM_TO_DOTS( MAX_LABEL_LEN_MM ) )	        /* len in dots */
#define MIN_LABEL_LEN					( MM_TO_DOTS( 15 ) )			        /* Min 15mm!!!??? */
#define MAX_LABEL_GAP_LEN_MM			150						        /* len in mm */
#define MAX_LABEL_GAP_LEN				( MM_TO_DOTS( MAX_LABEL_GAP_LEN_MM ) )	        /* len in dots*/
#define LABEL_LEN_TOLERANCE				( MM_TO_DOTS( 10 ) )			        /* tolerance */
#define MAX_BITMAP_LINES				( MAX_LABEL_LEN )
#define MAX_IMAGE_SIZE					( MAX_BITMAP_LINES * BYTES_PER_BITMAPLINE )


/*macros private to label.c */
#define GAP_LEN_TOLERANCE				(MM_TO_DOTS(5))				        /* accept upto 5mm adrift */
#define SENSOR_TO_PEEL_DIS				(MM_TO_DOTS(55))			        /* in mm */

#define SENSOR_TO_PHEAD_DIS				(MM_TO_DOTS(42 - (int)gTakingUpBackingPaper))	/* in mm - NB to allow for pull through, we decrease this by 1mm, when taking up backing paper. */
#define SENSOR_TO_TEAR_DIS				(MM_TO_DOTS(gCutter == true ? 62 : (gLinerlessLabels == false ? 60 : 57))) /* in mm */
#define FEED_FOR_TEAR					(MM_TO_DOTS(4))				        /* extra bit of margin so that printed area is not torn */
#define PHEAD_TO_PEEL_DIS				(SENSOR_TO_PEEL_DIS - SENSOR_TO_PHEAD_DIS)	/* amount to back feed to position label under head from parked position */
#define PHEAD_TO_TEAR_DIS				(SENSOR_TO_TEAR_DIS - SENSOR_TO_PHEAD_DIS)	/* feed after print for cont */
#define PEEL_TO_TEAR_DIS				(SENSOR_TO_TEAR_DIS - SENSOR_TO_PEEL_DIS) 

#define MOTORS_HOLD_POWER_S				3						/* motor's power retain delay in second after label printed */

/* To prevent paper jams, ideally, don't want to backwind continuous paper without
 * backing paper more than 5mm from the front of the tear off bar.
 * However to prevent the start of labels/receipts scrunching up due to backlash
 * the paper must be backwound by TAKEUP_BACKLASH_DIS (7 mm).
 * Therefore must backwind by TAKEUP_BACKLASH_DIS.
 * The backind distance for continuous paper without backing paper calculated in
 * psplSetBackwind is RECEIPT_BACKWIND_DIS + TAKEUP_BACKLASH_DIS.
 * However if make RECEIPT_BACKWIND_DIS 0 then the if(backwindDis > 0) statement
 * will not be met and it will try to "backwind" the label forwards.
 * Not sure if zero backwindDis is a valid case so to be on the safe side
 * make RECEIPT_BACKWIND_DIS one dot. Don't think anyone will notice one dot
 * extra backwind. */
#define RECEIPT_BACKWIND_DIS			        1						/* reduced backwind to ensure paper doesn't catch behind peel bar */

/* For clamshell */
#define CLAMSHELL_SENSOR_TO_PHEAD_DIS	                (MM_TO_DOTS(44))				/* Originally 42 */
#define CLAMSHELL_PHEAD_TO_TEAR_DIS		        ( SENSOR_TO_TEAR_DIS - CLAMSHELL_SENSOR_TO_PHEAD_DIS )    /* feed after print for cont */

#define MAX_GAP						( gLabelGapLength + GAP_LEN_TOLERANCE )
#define MIN_GAP						((gLabelGapLength > GAP_LEN_TOLERANCE) ? gLabelGapLength - GAP_LEN_TOLERANCE : 0)

/* paper cutter */
#define DEFAULT_CUTTER_SPEED			        9
#define MIN_CUTTER_SPEED				0
#define MAX_CUTTER_SPEED				9
#define DEFAULT_CUT_DISTANCE			        82
#define MIN_CUT_DISTANCE				30
#define MAX_CUT_DISTANCE				255
#define CUTTER_LOCKOUT_TIMEOUT			        5000

/* Converts from direction to motion */
#define DIRN_TO_MOTION(a)				(((a) == FORWARD) ? MOTION_FORWARD : MOTION_BACKWARD)
#define MOTION_TO_DIRN(a)				(((a) == MOTION_BACKWARD) ? BACKWARD : FORWARD)

#define PTF_MAIN_MOTOR 0    /*Which motor for production test feed */

/* constants */

/* simple forward/back direction */
typedef enum
{
  /* PC Previous definition inverted on the Vantron board */
 	FORWARD,
	BACKWARD,
} DIDECTION_ENUM;

/*forward, back, stopped motion */
typedef enum
{
	MOTION_STOPPED,
	MOTION_BACKWARD,
	MOTION_FORWARD,
} MOTION_ENUM;

typedef enum
{
	PAPER,
	BACKING,
	EMPTY
} PBE_ENUM;     

/*Where to park at end of label calibration */
typedef enum
{
	PARK_AT_PEEL,
	PARK_AT_PHEAD,
	PARK_AT_SENSOR,
} PARK_AT_ENUM;

/*values for printSequence */
typedef enum
{
	PS_SKIP,                /* do the next stage immediately (ie within this motor step) */
	PS_CONTINUE,            /* continue to do this stage in future motor steps */
	PS_PRINTING,            /* ie actually print (otherwise same as continue) */
	PS_SLACKEN,             /* ie actually slacken backing paper (otherwise same as continue) */
	PS_TIGHTEN,             /* ie actually tightening backing paper (otherwise same as continue) */
	PS_DONE,                /* done this stage - move to next stage at next step */
	PS_LABEL_FEED_ERROR,    /* label feed error occurred */
} PS_ENUM;


/* extra numeric printer status enum */
/* added extra numeric printer status primarily to add cutter not ready error. */
/* this is a numeric value instead of a bool so can add more error statuses */
/* without changing the structure of the message sent. */
/* for consistency use the same values as XTRA. */
/* the enum below was copied over from XTRA v5.2.6.1033+ */
/* note most of the values are not used by the printer software */
/* but are reserved because the scale uses them. */
typedef enum
{
	PRINTEROK,
	PRINTERUSBERROR,			/* Reserved - can't make contact */
	PRINTERCOMMANDFAIL,			/* Reserved - failed to fulfill its task */
	PRINTEROVERHEAT,			/* Reserved - cooked its goose */
	PRINTERFAULT,				/* Reserved - summit nasty */
	PRINTERPAPEROUT,			/* Reserved - run out of paper */
	PRINTERFEEDERROR,			/* Reserved - couldn't position die cut labels (maybe run out of labels) */
	PRINTERRESETOCCURRED,		/* Reserved - printer has reset - need to send config/calib data again */
	PRINTERCASSETTENOTFITTED,	/* Reserved - cassette is not fully home */
	PRINTERCASHDRAWEROPEN,		/* Reserved - cash drawer is currently open */
	PRINTERLABELNOTTAKEN,		/* Reserved - last printed label is still present */
	PRINTERWRONGPROTOCOL,		/* Reserved - the printer software is too old for this version of scale app */
	PRINTERPHEADSLIGHTLYWORN,	/* Reserved - the printhead is slightly worn */
	PRINTERPHEADSEVERELYWORN,	/* Reserved - the printhead is severely worn */
	PRINTERCUTTERNOTREADY,		/* Reserved for old ADS paper cutter - the paper cutter is not in the home position */
	PRINTERTICKETMOUTHOPEN,		/* Reserved for old ADS paper cutter - the printer ticket mouth is open */
	PRINTERCUTTERNOTFITTED,		/* Reserved for old ADS paper cutter - the paper cutter is not fitted */
	PRINTERBADPRINTHEAD,		/* Reserved for scale allergen stuff - bad printhead used to stop allergen compliance label printing */
	PRINTERNONETWORK,			/* Reserved for scale allergen stuff */
	PRINTERNOOVERNIGHTCOMMS,	/* Reserved for scale allergen stuff */

	/* Add new non-paper cutter error values before here */

	/* Avery XTi paper cutter errors */
	PRINTERBLADEOVERRUNFAULT,
	PRINTERCUTHOMECYCLETIMEOUTFAULT,
	PRINTERCUTTERPLATENNOTFITTED,
	PRINTERCUTTERDOOROPEN,
	PRINTERCUTTERMESSAGEERROR,
	PRINTERCUTTERTXBUFFERFULL,
	PRINTERCUTTERRXBUFFEROVERFLOW,
	PRINTERCUTTERRXETXMISSING,
	PRINTERCUTTERRXTIMEOUT,
	PRINTERCUTTERRXCMDMISMATCH,
	PRINTERCUTTERRXLENGTHMISMATCH,
	PRINTERCUTTERRXCRCERROR,
	PRINTERCUTTERXOVERRUNERROR,
	PRINTERCUTTERXPARITYERROR,
	PRINTERCUTTERXFRAMINGERROR,
	PRINTERCUTTERXBREAKINTERRUPT,
	PRINTERCUTTERSADDLENOTFITTED

	/* Add only new paper cutter error values before here */
} EXTRA_PRINTER_STATUS_ENUM;


/*states that the stepperIntrHandler() goes through */
typedef enum
{
	SIH_FEED,					/* feed paper */
	SIH_PRINT,					/* print */
	SIH_SLACKEN,				/* just unwind the backing paper, in preparation for backwind */
	SIH_TIGHTEN,				/* just re-tighten the backing paper, after backwind */
	SIH_FINISH,					/* finish */
	SIH_STOPPED,				/* handler stopped */
} SIH_STATUS;


/* values for extra status platen */
typedef enum
{
	NO_PLATEN = 0,
	UNKNOWN_PLATEN = 1,
	SLOTTED_PLATEN = 2,
	FULL_PLATEN = 3,
} EXTRA_PLATEN_STATUS_ENUM;

/* more efficient */
typedef	unsigned char Position;


/* vars
extern unsigned char sGreyBuf[ GREY_BUF_SIZE ];  */


/* label.c functions */
void getDebugData(void);
Position Label_GetPosition(void);
bool labelCalibrate(unsigned long parkingPlace);
PrintStatus_t labelPrint(Packet_t *pckt, bool final);
PrintStatus_t testLabelPrint(unsigned long len);
unsigned char Label_ReadSensor(unsigned long whichSensor);
PrintStatus_t labelFeed(unsigned long len);
PrintStatus_t prodTestFeed(unsigned long len, bool dirn, unsigned char which);
void backWind(void);
bool posndAtPeelOrTear(void);
bool posndB4Phead(void);
void initStepperIntr(void);
void disableStepperIntr(void);
void enableStepperIntr(void);
bool sensePaper(void);
void initPosnGlobals(bool continuous);
bool positioned(void);
bool checkPheadDots(void);
void setPrintStatus(long printStatus);
#endif