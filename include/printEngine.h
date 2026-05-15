#ifndef PRINTENGINE_H
#define PRINTENGINE_H
#include "MIMXRT1024.h"
#include "printHead.h"
#include "fsl_common.h"
#include "commandTable.h"
//#include "globalPrinter.h"
#include "hobartPrinterMessages.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <stdbool.h>
#include "dotWearTask.h"
#include "fsl_lpspi.h"

typedef enum
{
    _UNKNOWN_ENGINE,
    _AVERY_PRINTER,
    _HOBART_PRINTER
}PrintEngineType_t;

typedef enum SizingState
{
   PRE_SIZE_TIGHTEN,                    //0
   STEP_TO_LABEL_TAKEN,                 //1
   SIZE,                                //2
   STEP_TO_NEXT,                        //3
   GO_TO_IDLE,                          //4
   TAKEUP_BUSY                          //5
}SizingState_t;

typedef enum PrinterCommandOptions
{
   PrNoSecurityLabel,
   PrApplySecurityLabel,
   PrWaitForLabelTaken,
   PrStreamLabel,
   PrExpelToTearBar,
   PrCutterJiggleEnable
}PrinterCommandOptions_t;

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

#define GAP_SENSOR_TO_PEEL_BAR_HT_PRINTER 1100

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

#define DATA_LENGTH_IN_BYTES_GT_PRINTER           80U //640 dots, 8 bits per byte, 80 frames, 80mm Rohm printhead
#define DATA_LENGTH_IN_BYTES_HT_PRINTER           56U

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
}DotCheckerWaitType;

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
    uint8_t                     printheadBuffer[DATA_LENGTH_IN_BYTES_GT_PRINTER];
    
    lpspi_master_handle_t       masterHandle;
    lpspi_master_config_t       masterConfig;
    uint8_t                     txData[DATA_LENGTH_IN_BYTES_GT_PRINTER];
    uint8_t                     rxData[DATA_LENGTH_IN_BYTES_GT_PRINTER];
    uint8_t                     dotRunCount;
}DotCheckerStatus;
 
typedef enum
{
    DOT_GOOD,
    DOT_BAD,
    DOT_MARGINAL,
}HeadDotStatus;


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
}LowLabelStatus;
    
typedef struct 
{
    PRMsgType                msgType;       // 4 bytes
    short                    minValuePeeling;
    short                    maxValuePeeling;
    short                    minValueStreaming;
    short                    maxValueStreaming;
}LowLabelMinMaxMessage;
  
typedef struct {
    uint32_t stepLimit;
    uint16_t noGapCount;
    uint16_t gapCount;
}LabelCountEntry;

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
}LabelOffset;

static const LabelOffset labelOffsetTableHTPrinter[] = 
{
    { STEPS_PER_LENGTH1_00_HT,  158,  295 }, //0 //dont have
    { STEPS_PER_LENGTH1_50_HT,  160,  295 }, //1 //dont have
    { STEPS_PER_LENGTH1_75_HT,  162,  295 }, //2 //good
    { STEPS_PER_LENGTH2_00_HT,  164,  295 }, //3 //dont have
    { STEPS_PER_LENGTH2_37_HT,  166,  295 }, //4 //good
    { STEPS_PER_LENGTH2_50_HT,  168,  295 }, //5 //dont have
    { STEPS_PER_LENGTH3_00_HT,  170,  295 }, //6 //good
    { STEPS_PER_LENGTH3_50_HT,  172,  295 }, //7 //good
    { STEPS_PER_LENGTH4_00_HT,  174,  295 }, //8 //good
    { STEPS_PER_LENGTH4_50_HT,  176,  295 }, //9 //good
    { STEPS_PER_LENGTH5_00_HT,  178,  295 }, //10 //good
    { STEPS_PER_LENGTH5_50_HT,  182,  295 }, //11 //good
    { STEPS_PER_LENGTH6_00_HT,  186,  295 }, //12 //dont have
    { STEPS_PER_LENGTH6_50_HT,  190,  295 }, //13 //good
    { STEPS_PER_LENGTH7_00_HT,  200,  295 }, //14 //good
    { STEPS_PER_LENGTH7_50_HT,  205,  295 }, //15 //good
    { STEPS_PER_LENGTH8_00_HT,  210,  295 }, //16 //good
    { STEPS_PER_LENGTH8_50_HT,  210,  295 }, //17 //good
    { STEPS_PER_LENGTH9_00_HT,  215,  295 }, //18 //good 
    { STEPS_PER_LENGTH9_50_HT,  220,  295 }, //19 //good
    { STEPS_PER_LENGTH10_00_HT, 200,  295 }, //20 //good         
};

static const LabelOffset labelOffsetTableGTPrinter[] = 
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
   HeadType_t           headType;
   unsigned char        levels;                         /* number of levels of compensation. i.e. 1st level history, 2nd level adjacency, and
                                                           2nd level history = 3 levels of PH compensation */
   unsigned long        lineCounter;                    /* counts the Print Lines as they're loaded */
   unsigned long        lineCounter2;                   /* used when label image is greater than 5 " */
   unsigned char        burnSequence;                   /* tracks where we are in the line burn sequence   */
   unsigned short       pwmStartTime;                   /* time that PWMing is started */
   unsigned char        pwmDutyCycle;
   unsigned short       sltTime;                        /* overall line printing time */
   unsigned short       sltHalfTime;                    /* half of the SLT time  */
   bool                 linePrintDone;
   HistAdj              histAdj[MAX_HIST_ADJ_LEVELS];
   unsigned char        *pHistory;                      /* history for current print line */
   unsigned char        *pImage;                        /* print image buffer */
   signed short         numSteps;                       /* number of steps for the motor */ 
   StepDir              direction;                      /* direction of the stepper motor */
   signed short         numPrintLines;		        /* number of print lines */
   signed short         totalLinesToPrint;              /* total number of lines to print label */
   unsigned char        labelOrientation;	        /* head first or heel first */
   signed short         steps;                          /* number of steps for the step operations */
   unsigned short       outOfMediaCnt;                  /* number of steps past media */
   unsigned short       maxMediaCount;                  /* configuration value for cntr compare */
   unsigned short       stepsOffset;                    /* number of steps used to determine sync bar */
   signed short         labelTracking;                  /* number of steps to correct next label */
   
   /* TU Calibration parameters */
   unsigned long		cycleCounter;		/* used to keep track of time, engine runs every 1mS */
   unsigned long		cycleCounterRel;	/* used to keep track of time, relative to certain cal state */
   TU_CAL_STATE			TUCalState;		/* TU calibration state machine */   
   unsigned short		TUCalInitialTension;	/* used throughout the various calibration states */
   unsigned short		TUCalFinalTension;		/* used throughout the various calibration states */
   unsigned short		*TUCalMinTensionVals;	/* pointer to an array of 255 elements holding tension while spring is relaxed */
   unsigned short		*TUCalMaxTensionVals;	/* pointer to an array of 255 elements holding tension while spring is extended */
   unsigned short		*TUCalDeltaTensionVals;	/* pointer to an array of 255 elements holding deltas between relax and extended */
   unsigned char		TUCalEmittermACurrent;	/* mA current applied to TU Emitter during Cal */
}PrintEngine;


typedef struct {
    uint16_t min;
    uint16_t max;
    uint16_t noPaperValue;
    uint16_t paperValue;
} LabelRange;

/****************************** avery defines *********************************/
/******************************************************************************/

typedef enum
{
	CMD_NOT_USED,
	CMD_PRINT_LABEL,
	CMD_PRINT_LABEL_NO_PARK,
	CMD_PRINT_CONT_LABEL,
	CMD_PRINT_CONT_LABEL_NO_PARK,
	CMD_FEED_LABEL,
	CMD_FEED_PAPER,
	CMD_BACK_WIND
}PRCMDS_t;

typedef enum
{
    LABELS,
    CONTINUOUS_PAPER,
    REPORT_ON_LABELS
}MediaTypes_t;

typedef enum
{
    LG_UNDEFINED,
    ON_LABEL,
    IN_GAP,
}SenseStatus_t;

typedef struct
{
    PRCMDS_t cmd;
    PRCMDS_t nextCmd;

    uint16_t sPrintingPosn;
    uint32_t printSpeed;
    uint32_t greyDataBlocks;
    uint32_t seqStep;			                /* print engine states. where in the command are we? */
    uint16_t labelSpan;                                 /* length of label + gap */
    uint16_t labelSpanCount;
    uint16_t labelLen;
    int16_t labelLenCount;
    uint16_t measuredLabelLength;
    uint16_t measuredLabelGapLength;
    uint16_t inGapCount;
    uint16_t labelPosition;
    uint32_t labelGapLength;
    int32_t onLabel;			                /* -1 undefined, 0 in gap, 1 on label */
    uint32_t onLabelCnt;
    int32_t lastOnLabel;				/* -1 undefined, 0 in gap, 1 on label */

    uint32_t reportGapSize;                             /* how big gap in report is */
    uint32_t skipTheGapCount;
    int32_t leftOnLabel;

    bool imageContinuesFlg;
    SemaphoreHandle_t semPrinting;
    SenseStatus_t senseStatus;                            /* ie in gap or on label */
    bool feedError;
    MediaTypes_t printMedia;
}AvPrEngine_t;

/******************************************************************************/
/******************************************************************************/

AT_QUICKACCESS_SECTION_CODE( void initializePrintEngine( PrintEngineType_t type, unsigned int contrast, unsigned int mediaCount, QueueHandle_t pHandle ) );
unsigned short getNumPrintLinesLeft(void);
void addCmdToQueue( PrCommand *pCmd );
void setSkipLabelTakenCheck( void );
void startPrintEngine( void );
AT_QUICKACCESS_SECTION_CODE( void setLineTimerIntLevel( unsigned int level ) );
AT_QUICKACCESS_SECTION_CODE( void startLineTimer( bool start ) );
AT_QUICKACCESS_SECTION_CODE( void stopLineTimer( void ) );
AT_QUICKACCESS_SECTION_CODE(void scalePrintLineTimesRamped(void));
AT_QUICKACCESS_SECTION_CODE(bool getLeadInDone( void ));
int getTempAtStart( void );
void shutdownPrintEngine( void );
void initializeStepper( StepDir direction );
void powerOffStepper( void );
void powerOnStepper( void );
void setStepperDirection( StepDir direction );
void halfStepMotor( void );
void motorStep( StepDir dir, PrStatusInfo *pStatus );
void motorStepFast( PrStatusInfo *pStatus );
void initializePrintEngineTimer( uint16_t period_us );
void setPrintEngineTimerSlt( void );
void setPrintEngineTimer( unsigned short time );
void setEngineContrast( unsigned short contrast );
void resetPrintEngineTimer( void );
void resetEngine( void );
AT_QUICKACCESS_SECTION_CODE( void pauseEngine( void ) );
void initializePrintHeadPwm( void );
AT_QUICKACCESS_SECTION_CODE( void *getPrintEngine( PrintEngineType_t type ) );
AT_QUICKACCESS_SECTION_CODE( bool isEnginePaused() );
AT_QUICKACCESS_SECTION_CODE( unsigned long getPrintEngineLineCntr( void ) );
AT_QUICKACCESS_SECTION_CODE( void historyAdjacency( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadHistory( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadPrintLine( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadZeroPrintLine( void ) );
AT_QUICKACCESS_SECTION_CODE( bool isCurrentLine( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearBurnSequence( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearPrevVertOffset( void ) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerStrobe( uint8_t pwmDuty ) );
AT_QUICKACCESS_SECTION_CODE( void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevoius ) );
bool testCondition( PrStatusInfo *pStatus, TestOperator oper, unsigned char bits, unsigned char result );
void calibratePrinter( PrinterCal cal );
void createCheckerBoardLabel( unsigned char offset, unsigned long length );
void createVerticalLinesLabel( unsigned char offset, unsigned long length );
void createSingleVerticalLineLabel(  unsigned char offset, unsigned long length );
void createHorizontalLinesLabel( unsigned char offset, unsigned long length );
void bitSet( unsigned short startBit, unsigned short numBits, unsigned char *pBuffer );
int getLabelSize();
int getOutOfMediaCounts();
int getMaxOutOfMediaCounts();
void setOutOfMediaCounts(int val);
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
void waitOp( WaitOperation *pOperation );
void waitUntilOp( WaitUntilOperation *pOperation );
void waitUntilSizingOp( WaitUntilOperation *pOperation );
void testOp( TestOperation *pOperation );
void statusOp( StatusOperation *pOperation );
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
void cutOp( GenericOperation *pOperation );
void printDotWearOp( CmdOp *pOperation );
AT_QUICKACCESS_SECTION_CODE( void setHistoryEnabled(bool enabled) );
void detectionOp( StepUntilOperation *pOperation ); 
void setContinuousStock( void );
void clrContinuousStock( void );
bool getIDF2( void);
void setGapCurrentToSeventyFivePercent( bool status);
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

uint16_t calculateSizingBackwindSteps( void );
uint16_t calculateStreamingBackwindSteps( void );
uint16_t calculatePeelingBackwindSteps( void );

AT_QUICKACCESS_SECTION_CODE(uint16_t calculateHTLeadInTarget( void ));
AT_QUICKACCESS_SECTION_CODE(uint16_t calculateGTLeadInTarget( void ));

void setStreamingLabelBackwind( uint16_t steps );
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
bool checkForOutOfMediaHTPrinter( void );

void setStartOfQueue( bool );
bool getStartOfQueue( void );

AT_QUICKACCESS_SECTION_CODE(void setStreamingLeadInMod( uint16_t steps ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getStreamingLeadInMod( void ));

AT_QUICKACCESS_SECTION_CODE(void setStreamingExpelMod( uint16_t steps ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getStreamingExpelMod( void ));

AT_QUICKACCESS_SECTION_CODE(bool getFirstPrint( void ));
void setFirstPrint( bool fPrint);

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
uint16_t conversion_to_voltage_dV(uint16_t conversion);
void calcLabelTakenThreshold( bool fPrint );

void strobeForceLow(void);
void strobeForceHigh(void);
void strobeReleaseToPWM(uint8_t pwmDuty);

uint16_t getCutterStatusCheckedCounter( void );
void setCutterStatusCheckedCounter( uint16_t count );
bool getCutterStatusChecked( void );
void setCutterStatusChecked( bool checked );
void stopCutterPolling( bool stopped);
void setCutterJiggleEnabled( bool enabled );
bool getCutterJiggling( void );
bool getCutMsgSent( void );

#endif