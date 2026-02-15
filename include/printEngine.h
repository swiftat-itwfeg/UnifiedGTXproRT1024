#ifndef PRINTENGINE_H
#define PRINTENGINE_H
#include "MIMXRT1024.h"
#include "printHead.h"
#include "fsl_common.h"
#include "commandTable.h"
#include "globalPrinter.h"
#include "prMessages.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include "dotWearTask.h"
#include "fsl_lpspi.h"

typedef enum SizingState
{
   PRE_SIZE_TIGHTEN,                    //0
   STEP_TO_LABEL_TAKEN,                 //1
   SIZE,                                //2
   STEP_TO_NEXT,                        //3
   GO_TO_IDLE,                          //4
   TAKEUP_BUSY                          //5
}   SizingState_t;

typedef enum PrinterCommandOptions
{
   PrNoSecurityLabel,
   PrApplySecurityLabel,
   PrWaitForLabelTaken,
   PrStreamLabel,
   PrExpelToTearBar
}   PrinterCommandOptions_t;

#define PRINT_TIME                              25      /* 25.0 msec. */
#define STEP_TIME                               1       /*  1.0 msec. */
#define WAIT_TIME                               1       /*  2.0 msec. */
#define IDLE_TIME                               20      /* 20.0 msec. */  
#define DELAY_TIME                              20      /* 20.0 msec. */
#define CALIBRATION_TIME                        2       /*  2.0 msec. */
#define STEP_ISR_COUNT                          20      /* 20.0 msec. */
#define REVERSE_STEP_TIME                       2       /*  2.0 msec. */
#define FIRST_REVERSE_STEP_TIME                 2       /*  2.0 msec  */
#define SWITCH_TIME                             0       /*  0.0 msec  */

#define USEC_DELAY_COUNT                        17



#define printEngineISR                          (GPT1_IRQHandler)
#define ENGINE_TIMER_BASE                       GPT1       

#define ENGINE_TIMER_PRIORITY                   2
#define ENGINE_TIMER_COUNT                      (ENGINE_TIMER_BASE->CNT & 0x0000FFFF)       
#define ENGINE_TIMER_IRQ                        (GPT1_IRQn)
#define ENGINE_TIMER_PRESCALE                   (16U) //(64U)//(32U)//(1U)
#define DEFAULT_ENGINE_COUNT                    1476U     /* 1476U  current: changed from this to increase printspeed freestanding scale */                 
#define ONE_mS_ENGINE_TIME_BASE					3940U /* 1mS */

/*****************************************************************************
* The distance between the print line and the center of the media sensor.    *
*                2.914" / (0.00655"/step) = 445 steps                        *
* Now! Allow 10% of these steps as adustment after the sync is accomplished. *
*                         445 - 45 = 400                                     *
*****************************************************************************/
#define MEDIA_SENSOR_TO_PEEL_BAR                771  /*media sensor entry to peel bar 3.794" x 203.2 = 771*/ /* 3.661" = 744 lines @ 203.2 lines/inch */
#define BLACK_BAR_TO_EDGE_OF_LABEL               64  /* 5/16" = 63 lines @203.2 lines/inch */
#define ADJUSTMENT_FACTOR                        51  /* 1/4" - want label to stop short of expelling so "print position"
							 can adjust it (print position can only adust it forward */

#define MEDIA_SENSOR_TO_ALIGNMENT               (MEDIA_SENSOR_TO_PEEL_BAR - BLACK_BAR_TO_EDGE_OF_LABEL - ADJUSTMENT_FACTOR)

/* range from 1" to 5" */                        
#define  LOW_RANGE_START_SIZING                 203                                                                                            
#define  LOW_RANGE_END_SIZING                   1065
                                                /* apply this offset if sizing in low range */                        
#define  LOW_RANGE_OFFSET                       100     

                                                /* range from 5.5" to 7.5" */                                                                                                
#define  MID_RANGE_START_SIZING                 1066                                                                                            
#define  MID_RANGE_END_SIZING                   1573
                                                /* apply this offset if sizing in mid range */                        
#define  MID_RANGE_OFFSET                       80

                                                /* range from 8.0" to 9.5" */                                                                                                
#define  HIGH_RANGE_START_SIZING                1574                                                                                            
#define  HIGH_RANGE_END_SIZING                  2000
                                                /* apply this offset if sizing in mid range */                        
#define  HIGH_RANGE_OFFSET                      60
                                                /* 10" * 203 steps per inch */
#define CONTINUOUS_STOCK_MIN                    2030                                                                        

/* no defined in stepper driver */
typedef enum
{
    FORWARD_,
    BACKWARD_
}StepDir;
 

typedef enum
{
    FIRST_LEVEL_HIST,
    SECOND_LEVEL_HIST,
    FIRST_LEVEL_ADJ,
    SECOND_LEVEL_ADJ,
    CURRENT_LINE,
    UNKNOWN_COMPENSATION_TYPE
}HistAdjType;



typedef struct hist_adj_level_struct
{
    unsigned long      *pLinePointer;     /* PH Data, Source Address Low Word */
    unsigned short     time;
    HistAdjType compType;
}HistAdj;

#define MAX_HIST_ADJ_LEVELS           3

//#define lineTimerIsr                            (GPT2_IRQHandler)
//#define LINE_PRINTER_TIMER_BASE                 (GPT2) 
//#define LINE_PRINTER_TIMER_PRIORITY             1        /* was 2 */   
//#define LINE_PRINTER_TIMER_COUNT                (GPT2->CNT & 0x0000FFFF)       
//#define LINE_PRINTER_TIMER_IRQ                  (GPT2_IRQn)



#define STROBE_FTM_BASE                       (FTM0)   
#define STROBE_PWM_Hz                         (25265U)          /* 25.265Khz */        
#define STROBE_PWM_PERIOD                     ( 1000000000U / STROBE_PWM_Hz ) 
#define STROBE_FTM_CHANNEL                    (kFTM_Chnl_3)
#define STROBE_PWM_DEFAULT_DUTY_CYCLE         (50U)             /* [%] */
#define STROBE_PWM_POLARITY                   (kFTM_HighTrue)
#define STROBE_PWM_INTERRUPT_MASK             (kFTM_Chnl3InterruptEnable)
#define STROBE_PWM_ALIGNMENT                  (kFTM_EdgeAlignedPwm)
#define STROBE_FTM_SOURCE_CLOCK               CLOCK_GetFreq(kCLOCK_BusClk)
 

#define PRECENT_MS_CAL                        3 

#define EDGE_MARGIN     0
#define BMP_PITCH       16
#define BAR_HEIGHT      5

#define SHOOT_COUNT_ARRAY_SIZE 10000

#define GAP_SENSOR_TO_PEEL_BAR_GT 900
#define GAP_SENSOR_TO_PEEL_BAR_HT 840
#define GAP_SENSOR_TO_TEAR_BAR_GT 975
#define GAP_SENSOR_TO_TEAR_BAR_HT 970

#define LABEL_LOW_SAMPLE_COUNT                    14
#define LABEL_LOW_POINT_RECORD_THRESHOLD          75
#define LABEL_LOW_POINT_RECORD_THRESHOLD_CROSSED  (LABEL_LOW_POINT_RECORD_THRESHOLD + 1)
#define LOW_LABEL_MAX_PEELING                     1500
#define LOW_LABEL_MAX_STREAMING                   1400
#define LOW_LABEL_MAX_PEELING_DEFAULT             1000
#define LOW_LABEL_MAX_STREAMING_DEFAULT           1000
#define LOW_LABEL_MIN_PEELING_DEFAULT             600
#define LOW_LABEL_MIN_PEELING_DEFAULT_MIN         475
#define LOW_LABEL_MIN_PEELING_DEFAULT_MAX         700
#define LOW_LABEL_MIN_STREAMING_DEFAULT           600
#define LOW_LABEL_MIN_STREAMING_DEFAULT_MIN       475
#define LOW_LABEL_MIN_STREAMING_DEFAULT_MAX       700

#define DATA_LENGTH_IN_BYTES                      80U //640 dots, 8 bits per byte, 80 frames, 80mm Rohm printhead
#define FAILURE_METRIC_PERCENTAGE                 15U

#define LATCH_SETUP_TIME                          1
#define LATCH_HOLD_TIME                           1 

#define BIT0                                      0x01
#define BIT1                                      0x02
#define BIT2                                      0x04
#define BIT3                                      0x08
#define BIT4                                      0x10
#define BIT5                                      0x20
#define BIT6                                      0x40
#define BIT7                                      0x80

#define DOT_CHECKER_STROBE_WAIT_TIME              250
#define DOT_CHECKER_SETUP_WAIT_TIME               250
#define DOT_CHECKER_SAMPLES_TO_TAKE               11

#define DOT_RESISTANCE_EXPECTED                    (610.0f)
#define DOT_RESISTANCE_EXPECTED_MODIFIER_MARGINAL  ((20.0f / 100.0f) * (DOT_RESISTANCE_EXPECTED))
#define DOT_RESISTANCE_EXPECTED_MODIFIER_BAD       ((30.0f / 100.0f) * (DOT_RESISTANCE_EXPECTED))
#define DOT_RESISTANCE_MARGINAL_HIGH               ((DOT_RESISTANCE_EXPECTED) + (DOT_RESISTANCE_EXPECTED_MODIFIER_MARGINAL))
#define DOT_RESISTANCE_MARGINAL_LOW                ((DOT_RESISTANCE_EXPECTED) - (DOT_RESISTANCE_EXPECTED_MODIFIER_MARGINAL))
#define DOT_RESISTANCE_BAD_HIGH                    ((DOT_RESISTANCE_EXPECTED) + (DOT_RESISTANCE_EXPECTED_MODIFIER_BAD))
#define DOT_RESISTANCE_BAD_LOW                     ((DOT_RESISTANCE_EXPECTED) - (DOT_RESISTANCE_EXPECTED_MODIFIER_BAD))


typedef enum
{
    DOT_WEAR_INIT,
    DOT_WEAR_WRITE_DOT_TO_PH,
    DOT_WEAR_LATCH_PH,
    DOT_WEAR_STROBE_PH,
    DOT_WEAR_CALCULATE_DOT_RESISTANCE,
    DOT_WEAR_RESET,
    DOT_WEAR_EXIT,
    DOT_WEAR_WAIT
} DotCheckerState;

typedef enum
{
    DOT_WEAR_STROBE_WAIT,
    DOT_WEAR_SETUP_WAIT,
    DOT_WEAR_PH_TRANSFER_WAIT
} DotCheckerWaitType;

typedef struct 
{
    DotCheckerState             state;
    DotCheckerWaitType          waitType;
    
    uint32_t                    ADCBuffer;
    
    uint32_t                    strobeWaitTime;
    uint32_t                    setupWaitTime;
    uint16_t                    dotCount;
    uint8_t                     bitCount;
    uint8_t                     byteCount;
    uint8_t                     runCount;
    uint8_t                     dataLengthInBytes;   
    bool                        isTransferCompleted;
    bool                        isStrobeCompleted;
    bool                        isSetupCompleted;
    bool                        readyToSend;
    uint32_t                    dotResistanceValues[HEAD_DOTS_80MM];
    uint8_t                     printheadBuffer[DATA_LENGTH_IN_BYTES];
    
    lpspi_master_handle_t       masterHandle;
    lpspi_master_config_t       masterConfig;
    uint8_t                     txData[DATA_LENGTH_IN_BYTES];
    uint8_t                     rxData[DATA_LENGTH_IN_BYTES];
    uint8_t                     dotRunCount;
} DotCheckerStatus;
 
typedef enum
{
    DOT_GOOD,
    DOT_BAD,
    DOT_MARGINAL,
} HeadDotStatus;

typedef struct
{
    PRMsgType                msgType;       // 4 bytes
    uint16_t                 startingDotIndex;   // 2 byte
    char                     reserved[2];        // 2 bytes padding
    unsigned char            dots[56];  // 56 dot values (1 byte each)
} DotStatusMessage;          // total = 64 bytes

typedef struct 
{
    short               motorSteps;
    short               lastMotorSteps;
    short               samplePoint[LABEL_LOW_SAMPLE_COUNT];
    char                samplePointIndex;
    uint16_t            segmentLengthMaxPeeling;
    uint16_t            segmentLengthMinPeeling;
    uint16_t            segmentLengthMaxStreaming;
    uint16_t            segmentLengthMinStreaming;
    float               sensorEstimatedPercentageOfRollRemaining;
    int                 averagedSensorFeedback;
    char                samplesCollected;
    uint16_t            estimatedLabelCount;
    char                estimatedLabelCountPercentageOfRollRemaining;
    uint16_t            numberOfLabelsOnFullRoll;
    unsigned short      ADCSample;
    unsigned short      consecutiveSamplesAboveThreshold;
    unsigned short      consecutiveSamplesBelowThreshold;
    unsigned short      sensorFeedbackMax;
    unsigned short      sensorFeedbackMin;
    bool                thresholdSet;
} LowLabelStatus;
    
typedef struct 
{
    PRMsgType                msgType;       // 4 bytes
    short                    minValuePeeling;
    short                    maxValuePeeling;
    short                    minValueStreaming;
    short                    maxValueStreaming;
} LowLabelMinMaxMessage;
  
typedef struct {
    uint32_t stepLimit;
    uint16_t noGapCount;
    uint16_t gapCount;
} LabelCountEntry;

static const LabelCountEntry labelTable[] = 
{
    { STEPS_PER_LENGTH1_00, 1345, 1345 },
    { STEPS_PER_LENGTH1_50, 1345, 1345 },
    { STEPS_PER_LENGTH1_75, 1345, 1345 },
    { STEPS_PER_LENGTH2_00,  995,  995 },
    { STEPS_PER_LENGTH2_37,  995,  995 },
    { STEPS_PER_LENGTH2_50,  945,  945 },
    { STEPS_PER_LENGTH3_00,  810,  650 },
    { STEPS_PER_LENGTH3_50,  670,  550 },
    { STEPS_PER_LENGTH4_00,  555,  500 },
    { STEPS_PER_LENGTH4_50,  495,  450 },
    { STEPS_PER_LENGTH5_00,  445,  380 },
    { STEPS_PER_LENGTH5_50,  405,  350 },
    { STEPS_PER_LENGTH6_00,  395,  325 },
    { STEPS_PER_LENGTH6_50,  345,  285 },
    { STEPS_PER_LENGTH7_00,  320,  265 },
    { STEPS_PER_LENGTH7_50,  295,  245 },
    { STEPS_PER_LENGTH8_00,  295,  235 },
    { STEPS_PER_LENGTH8_50,  265,  225 },
    { STEPS_PER_LENGTH9_00,  245,  210 },
    { STEPS_PER_LENGTH9_50,  235,  200 },
    { STEPS_PER_LENGTH10_00, 225,  195 },
};
   
typedef struct {
    uint32_t stepLimit;
    uint16_t GTOffset;
    uint16_t HTOffset;
} LabelOffset;

static const LabelOffset labelOffsetTable[] = 
{
    { STEPS_PER_LENGTH1_00,  44,  120 },
    { STEPS_PER_LENGTH1_50,  44,  120 },
    { STEPS_PER_LENGTH1_75,  44,  120 },
    { STEPS_PER_LENGTH2_00,  44,  120 },
    { STEPS_PER_LENGTH2_37,  42,  118 },
    { STEPS_PER_LENGTH2_50,  40,  116 },
    { STEPS_PER_LENGTH3_00,  32,  114 },
    { STEPS_PER_LENGTH3_50,  30,  112 },
    { STEPS_PER_LENGTH4_00,  28,  110 },
    { STEPS_PER_LENGTH4_50,  26,  108 },
    { STEPS_PER_LENGTH5_00,  24,  106 },
    { STEPS_PER_LENGTH5_50,  22,  104 },
    { STEPS_PER_LENGTH6_00,  20,  102 },
    { STEPS_PER_LENGTH6_50,  18,  100 },
    { STEPS_PER_LENGTH7_00,  16,  100 },
    { STEPS_PER_LENGTH7_50,  14,  96 },
    { STEPS_PER_LENGTH8_00,  12,  94 },
    { STEPS_PER_LENGTH8_50,  12,  90 },
    { STEPS_PER_LENGTH9_00,  12,  88 },
    { STEPS_PER_LENGTH9_50,  12,  86 },
    { STEPS_PER_LENGTH10_00, 12,  86 },
};

typedef enum
{
	PREPARE_MEDIA_TU_CAL_START,	/* removes any media slack before attempting to cal */
	PREPARE_MEDIA_TU_CAL_END,	/* finished preparing media */
	RELAX1_MEDIA_TU_CAL,		/* relax after initial prep */
	VERIFY_INITIAL_TAKEUP_CURRENT, /* make sure that initial current is not too high */
	DETECT_MOTOR_STALL_TU_CAL,	/* intentional motor stall to find max spring tension */
	RELAX2_MEDIA_TU_CAL,		/* relax after motor stall */
	DETECT_MAX_TENSION_TU_CAL,	/* extend spring to max tension for calibration */
	COLLECT_MAX_TENSION_DATA_TU_CAL, /* Gather sensor data as current is increased from 0ma to 25ma */	
	RELAX3_MEDIA_TU_CAL,		/* relax after gathering max spring tension cal points */
	COLLECT_MIN_TENSION_DATA_TU_CAL,	/* Gather sensor data as current is increased from 0ma to 25ma */	
	CALCULATE_TU_CAL_DELTAS,	/* Calculate deltas between min and max tension */
	CALCULATE_TU_CAL_SETPOINT,	/* Calculate Emitter set point and write to config */
	IDLE_TU_CAL	
}TU_CAL_STATE;

typedef struct
{
    CmdOp currentCmd;
    
   /* The following three variables are used to select which Hist/Adj calculations
      are done. This is done so that only the types
      that are being used will be calculated (which saves lots of processing time).
      There is no 1st history variable, because it is always calculated. */
  
   bool                 calc2ndHistory;
   bool                 calc1stAdjacency;
   bool                 calc2ndAdjacency;
   bool                 pause;

   unsigned short       contrast;
   HEADTYPE             headType;
   unsigned char        levels;                     /* number of levels of compensation. i.e. 1st level history, 2nd level adjacency, and
                                                       2nd level history = 3 levels of PH compensation */
   unsigned long        lineCounter;                /* counts the Print Lines as they're loaded */
   unsigned long        lineCounter2;               /* used when label image is greater than 5 " */
   unsigned char        burnSequence;               /* tracks where we are in the line burn sequence   */
   unsigned short       pwmStartTime;               /* time that PWMing is started */
   unsigned char        pwmDutyCycle;
   unsigned short       sltTime;                    /* overall line printing time */
   unsigned short       sltHalfTime;                /* half of the SLT time  */
   bool                 linePrintDone;
   HistAdj              histAdj[MAX_HIST_ADJ_LEVELS];
   unsigned char        *pHistory;                   /* history for current print line */
   unsigned char        *pImage;                    /* print image buffer */
   signed short         numSteps;                   /* number of steps for the motor */ 
   StepDir              direction;                  /* direction of the stepper motor */
   signed short         numPrintLines;		    /* number of print lines */
   signed short         totalLinesToPrint;          /* total number of lines to print label */
   unsigned char        labelOrientation;	    /* head first or heel first */
   signed short         steps;                      /* number of steps for the step operations */
   unsigned short       outOfMediaCnt;              /* number of steps past media */
   unsigned short       maxMediaCount;              /* configuration value for cntr compare */
   unsigned short       stepsOffset;                /* number of steps used to determine sync bar */
   signed short         labelTracking;              /* number of steps to correct next label */
   
   /* TU Calibration parameters */
   unsigned long		cycleCounter;			/* used to keep track of time, engine runs every 1mS */
   unsigned long		cycleCounterRel;		/* used to keep track of time, relative to certain cal state */
   TU_CAL_STATE			TUCalState;				/* TU calibration state machine */   
   unsigned short		TUCalInitialTension;	/* used throughout the various calibration states */
   unsigned short		TUCalFinalTension;		/* used throughout the various calibration states */
   unsigned short		*TUCalMinTensionVals;	/* pointer to an array of 255 elements holding tension while spring is relaxed */
   unsigned short		*TUCalMaxTensionVals;	/* pointer to an array of 255 elements holding tension while spring is extended */
   unsigned short		*TUCalDeltaTensionVals;	/* pointer to an array of 255 elements holding deltas between relax and extended */
   unsigned char		TUCalEmittermACurrent;	/* mA current applied to TU Emitter during Cal */
   
   
} PrintEngine;


typedef struct {
    uint16_t min;
    uint16_t max;
    uint16_t noPaperValue;
    uint16_t paperValue;
} LabelRange;




AT_QUICKACCESS_SECTION_CODE( void initializePrintEngine( unsigned int contrast, unsigned int mediaCount, QueueHandle_t pHandle ) );
AT_QUICKACCESS_SECTION_CODE(unsigned short getNumPrintLinesLeft(void));
void addCmdToQueue( PrCommand *pCmd );
void setSkipLabelTakenCheck( void );
void startPrintEngine( void );
AT_QUICKACCESS_SECTION_CODE( void setLineTimerIntLevel( unsigned int level ) );
AT_QUICKACCESS_SECTION_CODE( void startLineTimer( bool start ) );
AT_QUICKACCESS_SECTION_CODE( void stopLineTimer( void ) );
AT_QUICKACCESS_SECTION_CODE( void stopLineTimer( void ) );
AT_QUICKACCESS_SECTION_CODE(void scalePrintLineTimesRamped(void));
AT_QUICKACCESS_SECTION_CODE(bool getLeadInDone( void ));
AT_QUICKACCESS_SECTION_CODE(int getTempAtStart( void ));
//AT_QUICKACCESS_SECTION_CODE( void shutdownPrintEngine( void ) );
void shutdownPrintEngine( void );
AT_QUICKACCESS_SECTION_CODE( void initializeStepper( StepDir direction ) );
AT_QUICKACCESS_SECTION_CODE( void powerOffStepper( void ) );
AT_QUICKACCESS_SECTION_CODE( void powerOnStepper( void ) );
AT_QUICKACCESS_SECTION_CODE( void setStepperDirection( StepDir direction ) );
AT_QUICKACCESS_SECTION_CODE( void halfStepMotor( void ) );
AT_QUICKACCESS_SECTION_CODE( void motorStep( StepDir dir, PrStatusInfo *pStatus ) );
AT_QUICKACCESS_SECTION_CODE( void motorStepFast( PrStatusInfo *pStatus ) );
AT_QUICKACCESS_SECTION_CODE( void initializePrintEngineTimer( uint16_t period_us ) );
AT_QUICKACCESS_SECTION_CODE( void setPrintEngineTimerSlt( void ) );
AT_QUICKACCESS_SECTION_CODE( void setPrintEngineTimer( unsigned short time ) );
AT_QUICKACCESS_SECTION_CODE( void setEngineContrast( unsigned short contrast ) );
AT_QUICKACCESS_SECTION_CODE( void resetPrintEngineTimer( void ) );
AT_QUICKACCESS_SECTION_CODE( void resetEngine( void ) );
AT_QUICKACCESS_SECTION_CODE( void pauseEngine( void ) );
AT_QUICKACCESS_SECTION_CODE( void initializePrintHeadPwm( void ) );
AT_QUICKACCESS_SECTION_CODE( PrintEngine *getPrintEngine( void ) );
AT_QUICKACCESS_SECTION_CODE( bool isEnginePaused() );
AT_QUICKACCESS_SECTION_CODE( unsigned long getPrintEngineLineCntr( void ) );
AT_QUICKACCESS_SECTION_CODE( void historyAdjacency( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadHistory( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadPrintLine( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadZeroPrintLine( void ) );
AT_QUICKACCESS_SECTION_CODE( bool isCurrentLine( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearBurnSequence( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearPrevVertOffset( void ) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerStrobe( void ) );
AT_QUICKACCESS_SECTION_CODE( void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevoius ) );
AT_QUICKACCESS_SECTION_CODE( bool testCondition( PrStatusInfo *pStatus, TestOperator oper, unsigned char bits, unsigned char result ) );
AT_QUICKACCESS_SECTION_CODE( void calibratePrinter( PrinterCal cal ) );
AT_QUICKACCESS_SECTION_CODE(void createCheckerBoardLabel( unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE(void createVerticalLinesLabel( unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE(void createSingleVerticalLineLabel(  unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE(void createHorizontalLinesLabel( unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE( void bitSet( unsigned short startBit, unsigned short numBits, unsigned char *pBuffer ) );
AT_QUICKACCESS_SECTION_CODE( int getLabelSize() );
AT_QUICKACCESS_SECTION_CODE( int getOutOfMediaCounts() );
AT_QUICKACCESS_SECTION_CODE( int getMaxOutOfMediaCounts() );
AT_QUICKACCESS_SECTION_CODE( void setOutOfMediaCounts(int val) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerBurn( void ) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerSLT( void ) ); 

AT_QUICKACCESS_SECTION_CODE( void idleOp( void ) );
AT_QUICKACCESS_SECTION_CODE( void printOp( CmdOp *pOperation ) );
void stepOp( StepOperation *pOperation );
void stepUntilOp( StepUntilOperation *pOperation );
AT_QUICKACCESS_SECTION_CODE( void stepGapOp( StepUntilOperation *pOperation ) );
void stepTakeupOp( StepUntilOperation *pOperation );
void stepEdgeOp( StepUntilOperation *pOperation );
void testForSyncOp( StepOperation *pOperation );
void testForLabelOp( StepOperation *pOperation );
void testForContinuous( StepOperation *pOperation );
void stepTakeupTightenOp( StepOperation *pOperation );
void reverseStepOp( StepOperation *pOperation );
AT_QUICKACCESS_SECTION_CODE( void waitOp( WaitOperation *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void waitUntilOp( WaitUntilOperation *pOperation ) );
void waitUntilSizingOp( WaitUntilOperation *pOperation );
void testOp( TestOperation *pOperation );
AT_QUICKACCESS_SECTION_CODE( void statusOp( StatusOperation *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void counterOp( CounterOperation *pOperation ) );
void calibrateOp( CmdOp *pOperation );
void freePrinterCalBuffers(void);
static void sendTUCalDoneToPrinterTaskFromISR( BaseType_t *xHigherPriorityTaskWoken );
void calibrateTUOp( CmdOp *pOperation );
AT_QUICKACCESS_SECTION_CODE( void dotWearOp( CmdOp *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void dotWearCalOp( CmdOp *pOperation ) );
void disableOp( CmdOp *pOperation );
AT_QUICKACCESS_SECTION_CODE( void clearCmdQueue( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearLabelImageBuffer( void ) );
void printerTests( void );
AT_QUICKACCESS_SECTION_CODE( void cutOp( GenericOperation *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void printDotWearOp( CmdOp *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void setHistoryEnabled(bool enabled) );
void detectionOp( StepUntilOperation *pOperation ); 
AT_QUICKACCESS_SECTION_CODE( void setContinuousStock( void ) );
AT_QUICKACCESS_SECTION_CODE( void clrContinuousStock( void ) );
AT_QUICKACCESS_SECTION_CODE( bool getIDF2( void) );
AT_QUICKACCESS_SECTION_CODE( void setGapCurrentToSeventyFivePercent( bool status) );
void setTUSlip(bool status);
bool getTUSlip( void );

AT_QUICKACCESS_SECTION_CODE(bool getUsingContinuous(void));
bool getSizingLabels(void);
void setSizingLabels(bool sizing);
int getBackwindOffset( void );
bool getBackwindAfterSizing( void );
void setBackwindAfterSizing( bool backwind );
bool getBackwindAfterSizingDone( void );
void setBackwindAfterSizingDone( bool backwind );
uint32_t getLTWaitCount_( void );
void setLTWaitCount_( uint32_t waitCount );
bool getExpelDone( void );
int getShootIndex( void );
void setShootIndex( int index );

AT_QUICKACCESS_SECTION_CODE(uint16_t calculateSizingBackwindSteps( void ));
AT_QUICKACCESS_SECTION_CODE(uint16_t calculateStreamingBackwindSteps( void ));
AT_QUICKACCESS_SECTION_CODE(uint16_t calculatePeelingBackwindSteps( void ));

AT_QUICKACCESS_SECTION_CODE(uint16_t calculateHTLeadInTarget( void ));
AT_QUICKACCESS_SECTION_CODE(uint16_t calculateGTLeadInTarget( void ));

AT_QUICKACCESS_SECTION_CODE(void setStreamingLabelBackwind( uint16_t steps ));
AT_QUICKACCESS_SECTION_CODE(void setTPHStepsPastGapThisPrint( uint16_t steps ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getTPHStepsPastGapThisPrint( void ));
void setSizingState( char state );

void setPrintingStatus(bool status);
bool getPrintingStatus( void );
void setSizingStatus(bool status);
bool getSizingStatus( void );
void setCanceledSizingFlag(bool status);
bool getCanceledSizingFlag( void );
bool checkForOutOfMedia( void );

AT_QUICKACCESS_SECTION_CODE(void setStartOfQueue( bool ));
AT_QUICKACCESS_SECTION_CODE(bool getStartOfQueue( void ));

AT_QUICKACCESS_SECTION_CODE(void setStreamingLeadInMod( uint16_t steps ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getStreamingLeadInMod( void ));

AT_QUICKACCESS_SECTION_CODE(void setStreamingExpelMod( uint16_t steps ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getStreamingExpelMod( void ));

AT_QUICKACCESS_SECTION_CODE(bool getFirstPrint( void ));

AT_QUICKACCESS_SECTION_CODE(uint16_t getLabelPauseTimeout( void ));
AT_QUICKACCESS_SECTION_CODE(void setLabelPauseTimeout( uint16_t timeout ));

AT_QUICKACCESS_SECTION_CODE(void updateLowLabelStatus( void ));
AT_QUICKACCESS_SECTION_CODE(void processLowLabelSamples(void));
AT_QUICKACCESS_SECTION_CODE(void setLabelLowSteps( short steps ));
AT_QUICKACCESS_SECTION_CODE(short getLabelLowSteps( void ));
AT_QUICKACCESS_SECTION_CODE(void resetLabelLowVars( void ));
AT_QUICKACCESS_SECTION_CODE(void resetLabelLowSamples( void ));
AT_QUICKACCESS_SECTION_CODE(void updateNumberOfLabelsOnRoll(void));
AT_QUICKACCESS_SECTION_CODE(void updateRollCompletionPercentage(void));

AT_QUICKACCESS_SECTION_CODE(void checkDots(uint16_t testCycleCount));

AT_QUICKACCESS_SECTION_CODE(uint32_t getHeadWearDot( int x ));
AT_QUICKACCESS_SECTION_CODE(HeadDotStatus getHeadWearDotStatus( int x ));
AT_QUICKACCESS_SECTION_CODE(char roundToNearest5(char value));
AT_QUICKACCESS_SECTION_CODE(char roundToNearest25(char value));
AT_QUICKACCESS_SECTION_CODE(LowLabelStatus* getLowLabelStatus( void ));

short getLowLabelPeelingMaxFromHost( void );
short getLowLabelStreamingMaxFromHost( void );
short getLowLabelPeelingMinFromHost( void );
short getLowLabelStreamingMinFromHost( void );
void setLowLabelPeelingMaxFromHost(short value);
void setLowLabelStreamingMaxFromHost(short value);
void setLowLabelPeelingMinFromHost(short value);
void setLowLabelStreamingMinFromHost(short value);
uint16_t calculateSizingOffset(uint16_t labelSteps);


#endif