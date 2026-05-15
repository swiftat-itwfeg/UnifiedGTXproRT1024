#ifndef SENSORS_H
#define SENSORS_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_adc.h"
//#include "globalPrinter.h"
#include "hobartPrinterMessages.h"
#include <stdbool.h>
#include "fsl_common.h" 


#define sensors_task_PRIORITY ( configMAX_PRIORITIES - 1 )


/******************************* hobart defines *******************************/
/******************************************************************************/

#define BY_TWO                                  2       /* shifting by two = divide by four */
#define HEAD_DELTA                              60      /* about 1/2Volt */
#define HEAD_PERIOD                             4       /* average four readings */

/* Sensor Bits */
#define MOTOR_FORWARD                           0x01
#define LABEL_TAKEN                             0x02
#define SYNC_BAR                                0x04
#define SYNC_BAR_EDGE                           0x08
#define WIDE_LABEL                              0x10
#define THERMAL_OVERLOAD                        0x20
#define OUT_OF_MEDIA                            0x40
#define SYNCHRONIZED                            0x80

#define THERMAL_SHUTDOWN_DEBOUNCE_COUNT         5000
#define HIGH_HEAD_TEMPERATURE_WARNING_ROHM      65
#define LOW_HEAD_TEMPERATURE_WARNING_ROHM       -10
#define HIGH_HEAD_TEMPERATURE_LIMIT_ROHM        70
#define LOW_HEAD_TEMPERATURE_LIMIT_ROHM         -15

/* 12bit adc setting for global */ 
#define AD_RESOLUTION                           .00080566      /* voltage per count */

#define NO_CASSET_THRESHOLD              3600           /* 2.75Vdc ( 2.75 / .00080566 )*/  

#define FREESTAND_HEADUP_THRESHOLD       2482           /* 2.0Vdc */
#define MEDIA_CAL_VOLTAGE                372            /* 300mV  ( .300 / .00080566 ) */
#define MEDIA_SYNC_BAR_THRESHOLD         1241           /* 1.00Vdc ( 1.0 / .00080566 ) */
#define MEDIA_OUT_OF_THRESHOLD           2978           /* 2.4Vdc ( 2.4 / .00080566 ) */             



#define LABEL_EDGE_THRESHOLD                    1300            /* was 840 */
/* backing paper threshold is 60% of label edge threshold */
#define BACKING_PAPER_THRESHOLD                 1000            /* 700 was 450 */        
#define SHOOT_THROUGH_NOISE_FLOOR               20

typedef enum
{
    AD_INIT,
    AD_MANUAL,
    AD_AUTO
}ADConfig;

/* internal a/d */
#define SHOOT_SENSOR_CHANNEL                    7U              /* ad2 channel 7  */
#define TAKEN_SENSOR_CHANNEL                    0U              /* ad2 channel 0 */
#define HEAD_TEMPERATURE_CHANNEL                9U              /* ad2 channel 9  */
#define HEAD_UP_CHANNEL                         10U             /* ad2 channel 10 */
#define HEAD_DOT_CHANNEL                        11U             /* ad2 channel 11 */
#define LOW_STOCK_CHANNEL                       1U              /* ad2 channel 1 */
#define PAPER_TAKEUP_CHANNEL                    8U              /* ad2 channel 8  */ 
#define HEAD_DETECT_CHANNEL                     2U


/* isr handlers for auto mode */
#define adcEtc0HandleIsr                ADC_ETC_IRQ0_IRQHandler
#define adcEtcErrorHandleIsr            ADC_ETC_ERROR_IRQ_IRQHandler
#define pitHandleIsr                    PIT_IRQHandler 

/* isr handler for manual mode */
#define adcModule2IsrHandler            ADC2_IRQHandler

/* eDMA */
#define ADC0_DMA_MAJOR_CHANNEL                  5U
#define ADC0_DMA_MINOR_CHANNEL                  6U
#define ADC1_DMA_MAJOR_CHANNEL                  7U
#define ADC1_DMA_MINOR_CHANNEL                  8U

//#define USE_EDMA      
#define TRANSFER_BYTES                          16U


#define HEAD_UP_DEBOUNCE                        5
#define SAMPLING_FREQ                           10000   /* 10Khz */
#define NUMBER_OF_SAMPLES                       8

/* print head temperature no longer counts but actual  */       
#define ZERO_DEGREES_C                          0
#define TEN_DEGREES_C                           10
#define TWENTY_DEGREES_C                        20
#define THIRTY_DEGREES_C                        30
#define FOURTY_DEGREES_C                        40
#define FIFTY_DEGREES_C                         50
#define SIXTY_DEGREES_C                         60

#define ENGINE_BUSY_STATE                       0x01

#define OUT_OF_MEDIA_THRESHOLD                  700 

typedef enum
{
    CHANNEL_TAKEN,
    CHANNEL_LOW_STOCK,    
    CHANNEL_SHOOT,
    CHANNEL_PAPER_TAKEUP,  
    CHANNEL_HEAD_TEMP,
    CHANNEL_HEAD_STATE,             
    CHANNEL_HEAD_DOT,
    CHANNEL_HEAD_DETECT,
    MAX_ADC_CHANNELS
}ADChannles;

typedef struct pr_ad_counts
{
    bool labelTaken;
    unsigned short thermistor;
    unsigned short thermistor2;
    unsigned short headVoltage;
    unsigned short mediaDetector;
    unsigned short boardTemp;
    unsigned short supplyCurrent;
    unsigned short headUp;
}HeadSensors;

typedef struct 
{
    unsigned short dotPosition;
    unsigned short sample1;
    unsigned short sample2;
    unsigned short sample3;
}DotTest;

/* same size as the prepack head */
#define MAX_DOT_HEAD_SIZE                       680 
#define TK_MAX_SAMPLES                          10

typedef struct
{
    unsigned int  adc0Channel;                          /* current channel adc0 */
    unsigned int  chIndex;                              /* channel list index */
    ADChannles  channelList[MAX_ADC_CHANNELS];          /* channel list */
    unsigned short value[MAX_ADC_CHANNELS];             /* channel conversions */
    unsigned short tkFilter[TK_MAX_SAMPLES];        	/* takeup value buffer */
    unsigned char  tkIndex;                             /* index for filtering buffer above */
    bool adcComplete;
    bool adcPaused;
	bool avgTakeup;                                     /* start running average of takeup sensor */                                   /* start running average of takeup sensor */ 
}ADCManager;

#define MAX_CHANNEL_SAMPLE                      65535
/******************************************************************************/
/******************************************************************************/


/****************************** avery defines *********************************/
/******************************************************************************/
#define ADC_TO_MVOLTS(a)			(((a) * 3300) / 4095) 
#define MVOLTS_TO_ADC(a)			(((a) * 4095) / 3300)

#define SENSORS_ADC_BASE			ADC2
#define SENSORS_ADC_CHANNEL_GROUP	        0U


#define TAKEN_SENSOR_THRESHOLD			0	/* default value */
#define TK_UP_SENSOR_BIAS_MVOLTS		1200
#define TK_UP_SENSOR_BIAS			(MVOLTS_TO_ADC(TK_UP_SENSOR_BIAS_MVOLTS))

#define TEMP_TBL_FIRST_ENTRY			1000	// mVolts
#define TEMP_TBL_INCREMENT			25	// mVolts

#define MIN_MIN_LABEL_EDGE_DIFF			5		//out of 0-255
#define MAX_MIN_LABEL_EDGE_DIFF			200		//out of 0-255
#define DEFAULT_MIN_LABEL_EDGE_DIFF		50		//out of 0-255
#define MID_FRACTION				3
#define SENSOR_HYSTERESIS			(nv.gMinLabelEdgeDiff == DEFAULT_MIN_LABEL_EDGE_DIFF ? (DEFAULT_MIN_LABEL_EDGE_DIFF/(4 * MID_FRACTION)) : (nv.gMinLabelEdgeDiff/(4 * MID_FRACTION)))
#define GAP_SENSE_MID(max)			(nv.gMinLabelEdgeDiff == DEFAULT_MIN_LABEL_EDGE_DIFF ? sGapSenseMid : (max - (((MID_FRACTION-1) * nv.gMinLabelEdgeDiff)/MID_FRACTION)))

#define MIN_MEDIA_EDGE_DIFF			100
#define MEDIA_MID_FRACTION			2
#define MEDIA_HYSTERESIS			((MIN_MEDIA_EDGE_DIFF * 2)/(3 * MEDIA_MID_FRACTION))

enum
{
	ADC_GAP,
	ADC_TEMP,
	ADC_TK_UP,
	ADC_MEDIA,
	ADC_24V,
	ADC_LBL_TAKEN,
	NUM_ADC,
};

/******************************************************************************/
/******************************************************************************/


/************************** common public functions ***************************/
/******************************************************************************/

BaseType_t createSensorsTask( ADConfig mode );
static void sensorsTask( void *pvParameters );
TaskHandle_t getSensorsHandle( void );

/******************************************************************************/
/******************************************************************************/


/************************** hobart public functions ***************************/
/******************************************************************************/

static void initManager( ADCManager *pMgr );

void initializeSensors( void );
void initializeAdcs( ADConfig configuration );
void initializeEtcAdc( ADConfig configuration );
void initXbar( ADConfig configuration );
void initPitADC( void );

void initLabelTakenLED( void );
void pauseResumeConversions( bool pause );
unsigned short getShootThroughConversion( void );
bool isADCAutoMode( void );
void showLabelTakenCounts( void );
void showPaperTakeup( void );
bool getOutOfMedia(void);  
void setTakeupFiltering( void );
void clrTakeupFiltering( void );

AT_QUICKACCESS_SECTION_CODE(void readADChannels( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned int readHeadVoltage( void ));
AT_QUICKACCESS_SECTION_CODE(void readMediaSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(bool readLabelTakenSensor( void ));
AT_QUICKACCESS_SECTION_CODE(void processLabelTakenSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(bool readHeadUpSensor( PrStatusInfo *pStatus ));
void readLowStockSensor( PrStatusInfo *pStatus );
AT_QUICKACCESS_SECTION_CODE(unsigned short getPrintheadThermistorCounts( void ));
void readHeadTemperatureSensor( PrStatusInfo *pStatus );
AT_QUICKACCESS_SECTION_CODE(unsigned char getHeadSupplyCurrent( void ));
void getCurrentSensors( PrSensors *pSensors );
AT_QUICKACCESS_SECTION_CODE(unsigned short getMediaCounts( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short pollMediaCounts( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getLowStockSensor( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned long getLabelTaken( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getTakeUpTorque( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getPaperTakeUp( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getHeadVoltage( void ));
void setMotorStopOnGap( void );
void clrMotorStopOnGap( void );
void setMotorStopOnEdge( void );
void clrMotorStopOnEdge( void );

unsigned short sampleMediaSensor( void );
unsigned short sampleHeadDotADC( void );
unsigned short sampleHeadDot( void );
unsigned short sampleHeadVoltage( void );
void incrementIndexDotTest( void );
void showADCReadings( void );
void setADCMode( ADConfig mode );
int getGhostMCntr( void );
void clearGhostMCntr( void );
bool getBackwindAfterContExpel( void );
void setBackwindAfterContExpel( bool backwind );
bool getTUCalHeadUpFlag( void );
void setTUCalHeadUpFlag( bool cal );
bool getHeadUp( void );
void initPWM2Sensors( void );
void setMediaSensorDutyCycle( uint8_t dutyCycle );
bool getCutterBladeDelayNeeded( void );
void setCutterBladeDelayNeeded( bool delayNeeded );
uint16_t getLowestLabelTakenReading( void );

/******************************************************************************/
/******************************************************************************/


/************************** avery public functions ****************************/
/******************************************************************************/
long readPrinterAdc(unsigned long whichADC);
bool isLabelTakenSensorEnabled(void);
void enableLabelTakenSensor(void);
void disableLabelTakenSensor(void);
bool checkLabelTaken(void);
unsigned char calGapSensor(void);
unsigned char calMediaSensor(void);
unsigned char calTakeUpSensor(void);
unsigned long calTakeUpSensorSpan(void);
char convertToTemp(unsigned long adcReading);
unsigned long convertTo24vMVolts(unsigned long adcReading);
unsigned long getTUSenseDeflection(void);
bool cassetteDoorClosed(void);
/******************************************************************************/
/******************************************************************************/

#endif