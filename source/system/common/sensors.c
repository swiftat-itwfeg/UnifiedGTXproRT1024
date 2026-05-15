#include "sensors.h"
#include "lp5521.h"
#include "w25x10cl.h"
#include <stdlib.h>
#include <math.h>
//#include "filter.h"
#include "printHead.h"
#include "printEngine.h"
#include "threadManager.h"
#include "queueManager.h"
#include "serialflash.h"
#include "label.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "fsl_dmamux.h"
#include "fsl_adc_etc.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_xbara.h"
#include "fsl_qtmr.h"
#include "fsl_pwm.h"
#include "fsl_debug_console.h"
#include "takeupMotor.h"
#include "hobartPrinterTask.h"
#include "averyPrinterCfg.h"
#include "averyCutter.h"
#include "averyTakeup.h"
#include "averyUtils.h"

#include "diagnostics.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"


/******************************* hobart variables *****************************/
/******************************************************************************/
uint16_t GTXHeadUpCounter = 0;
uint16_t lowestLabelTakenReading = 4999;
uint16_t LTSamplesTaken = 0;
bool cutterBladeDelayNeeded = false;

AT_NONCACHEABLE_SECTION_INIT( static uint16_t thermalOverloadDebounceCounter ) = 0;
static bool outOfMedia = false;
//running averager for takeup sensor during Calibration
#define TAKEUPMEDIABUFFERSIZE 6   /* optimized for CAL procedure where motor step rate is 500hz */

static int TakeupMediaBuffer[TAKEUPMEDIABUFFERSIZE];
static unsigned short TakeupMediaBufferPtr = 0;
static int TakeupMediaBufferSum = 0;


static bool suspend_                    = false;
static TaskHandle_t pHandle_            = NULL;
static ADConfig mode_                   = AD_INIT;
static unsigned int measuredLength_     = 0;
static unsigned int actualLength_       = 0;
static bool motorStopGap_               = false;
static bool motorStopEdge_              = false;
unsigned long lTick                     = 0;
static bool backwindAfterContExpel      = false;

extern PrStatusInfo                     currentStatus;
extern PrStatusInfo                     prevStatus;
extern Pr_Config                        config_;
extern LabelPosition                    *pCurrentLabel;
extern int                              labelAlignment;
extern bool                             mainMotorStopped_;
extern uint16_t                         LABEL_TAKEN_THRESHOLD_LABEL;
extern uint16_t                         LABEL_TAKEN_THRESHOLD_NO_LABEL;
extern bool                             firstPrint;

extern void powerOffStepper( void );
extern bool isLabelTakenTimerStarted( void );
extern void stopLabelTakenTimer( void );
extern void sendTakeLabelError( bool error );
extern void setOperation( unsigned char operation, PrStatusInfo *pStatus );
HeadSensors  headSensors;

uint8_t pitTimerLength = 1;

AT_NONCACHEABLE_SECTION_INIT( uint32_t adcMux[9] ) = { SHOOT_SENSOR_CHANNEL, HEAD_UP_CHANNEL, HEAD_DOT_CHANNEL, 
                                                       HEAD_TEMPERATURE_CHANNEL, SHOOT_SENSOR_CHANNEL, 
                                                       TAKEN_SENSOR_CHANNEL, PAPER_TAKEUP_CHANNEL, 
                                                       LOW_STOCK_CHANNEL, HEAD_DETECT_CHANNEL };
volatile ADCManager adcManager;

/*  testing dot measurement */
unsigned short indx_ = 0;

/* track if label is present */
static int ghostM_ = 0;

bool prevHeadUp = false;
bool currentHeadUp = false;

volatile unsigned char chCntr_ = 0;
unsigned char headTemperature = 0;
volatile bool tracking_ = false;

static bool TUCalHeadUpFlag = false;

#define LABEL_TAKEN_DELAY_LENGTH 10500
/******************************************************************************/
/******************************************************************************/

/******************************* avery variables ******************************/
/******************************************************************************/
unsigned long gTakenSensorThreshold = TAKEN_SENSOR_THRESHOLD;
unsigned long gMVolts24;

static unsigned long s24v = NOMINAL_MVOLTS; //Used to provide crude filtering
static bool	sLabelTakenActive = false; //tells you if the sensor is active (ie ready to read)
static bool sLabelTakenSensorState = true; //updated each time you read the sensor

static const int8_t tempTbl[] = {   
75, /* 1000mV */ 75, /* 1025mV */ 74, /* 1050mV */ 73, /* 1075mV */ 72, /* 1100mV */ 71, /* 1125mV */ 70, /* 1150mV */ 69, /* 1175mV */ 68, /* 1200mV */ 67, /* 1225mV */
66, /* 1250mV */ 65, /* 1275mV */ 64, /* 1300mV */ 63, /* 1325mV */ 62, /* 1350mV */ 61, /* 1375mV */ 60, /* 1400mV */ 59, /* 1425mV */ 58, /* 1450mV */ 58, /* 1475mV */
57, /* 1500mV */ 56, /* 1525mV */ 55, /* 1550mV */ 54, /* 1575mV */ 54, /* 1600mV */ 53, /* 1625mV */ 52, /* 1650mV */ 51, /* 1675mV */ 50, /* 1700mV */ 50, /* 1725mV */
49, /* 1750mV */ 48, /* 1775mV */ 47, /* 1800mV */ 46, /* 1825mV */ 45, /* 1850mV */ 45, /* 1875mV */ 44, /* 1900mV */ 43, /* 1925mV */ 42, /* 1950mV */ 42, /* 1975mV */
41, /* 2000mV */ 40, /* 2025mV */ 39, /* 2050mV */ 38, /* 2075mV */ 38, /* 2100mV */ 37, /* 2125mV */ 36, /* 2150mV */ 35, /* 2175mV */ 34, /* 2200mV */ 34, /* 2225mV */
33, /* 2250mV */ 32, /* 2275mV */ 31, /* 2300mV */ 30, /* 2325mV */ 29, /* 2350mV */ 28, /* 2375mV */ 28, /* 2400mV */ 27, /* 2425mV */ 26, /* 2450mV */ 25, /* 2475mV */
24, /* 2500mV */ 23, /* 2525mV */ 22, /* 2550mV */ 21, /* 2575mV */ 20, /* 2600mV */ 19, /* 2625mV */ 18, /* 2650mV */ 17, /* 2675mV */ 16, /* 2700mV */ 15, /* 2725mV */
14, /* 2750mV */ 13, /* 2775mV */ 12, /* 2800mV */ 10, /* 2825mV */ 9, /* 2850mV */ 8, /* 2875mV */ 6, /* 2900mV */ 5, /* 2925mV */ 3, /* 2950mV */ 2, /* 2975mV */ 
0, /* 3000mV */ -1, /* 3025mV */ -3, /* 3050mV */ -6, /* 3075mV */ -8, /* 3100mV */ -10, /* 3125mV */ -13, /* 3150mV */ -16, /* 3175mV */ -20, /* 3200mV */ -25, /* 3225mV */
};

PRCfg_t *pCfg;

#define LABEL_TAKEN_COUNTS_NEEDED       3
#define ON    		                1
#define OFF   		                2 
#define LP5521_GAP_CURRENT		_LP5521_GAP_CUR_REG
#define LP5521_TK_UP_CURRENT		_LP5521_PAPER_TAKE_UP_CUR_REG
#define LP5521_MEDIA_CURRENT		_LP5521_LOW_STOCK_CUR_REG

extern DebugData_t gDebugData;
extern unsigned long gDebugIdx;
extern PRCfg_t *getAveryPrinterCfg( void );

extern bool gClamshell;
extern void uSleep( uint32_t usec );
/******************************************************************************/
/******************************************************************************/

/************************** hobart public functions ***************************/
/******************************************************************************/

/******************************************************************************/
/*!   \fn BaseType_t createSensorsTask( ADConfig mode )

      \brief
        This function intializes and creates the sensors thread.
        If the mode is set to AD_MANUAL the sensors task will manually
        intiate A/D conversions and muxing A/D channels. If the mode is set to
        AD_AUTO then no sensors task is created and the trigger for A/D conversions
        and channel muxing is handled through hardware (PDB, ADC, DMA).
      \author
          Aaron Swift
*******************************************************************************/
BaseType_t createSensorsTask(  ADConfig mode  )
{
    BaseType_t result;
    mode_ = mode;
    
    /* clear our manager */
    memset( (void *)&adcManager, 0, sizeof(ADCManager) );
    initManager( (ADCManager *)&adcManager );
    
    /* do not create managment thread if AUTO mode is used */
    if( mode_ != AD_AUTO ) {
               
        /* init sensors */ 
        initializeSensors();
        PRINTF("\r\n\r\ninit sensors\r\n\r\n");
        
        /* create sensors task thread */
        result = xTaskCreate( sensorsTask,  "SensorsTask", configMINIMAL_STACK_SIZE,
                                            NULL, sensors_task_PRIORITY, &pHandle_ );
        /* give the thread manager the handle */
        updateTaskHandle( T_SENSORS );
    } else {
        /* init sensors */ 
        initializeSensors();
        result = pdPASS;
    }
    
    pCfg = getAveryPrinterCfg();  

    
    return result;
}

/******************************************************************************/
/*!   \fn static void sensorsTask( void )

      \brief
        This function is the sensors run thread which manually intiates the 
        A/D conversions. 400uS for all channels to be processed and another 
        channel scan started. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void sensorsTask( void *pvParameters )
{
    
    ( void ) pvParameters;
        
    PRINTF("sensorsTask(): Thread running...\r\n" ); 
    #if 0   
    static int dbCnt_ = 0;
    #endif
    static adc_channel_config_t adcChanCfg;
    
    adcManager.adc0Channel = CHANNEL_TAKEN;
    adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
    adcChanCfg.enableInterruptOnConversionCompleted = true;    
   
    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
               
    static bool adcOnce_ = false;
    while( !suspend_ ) {
                
        if( !adcManager.adcPaused ) {      
            /* is conversion complete? */
            if( adcManager.adcComplete ) {
                            
                if( adcManager.adc0Channel == CHANNEL_TAKEN ) {
                    /* change the channel and start the conversion */
                    adcChanCfg.channelNumber = LOW_STOCK_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;                    
                    adcManager.adc0Channel = CHANNEL_LOW_STOCK;
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );     
                    
                } else if( adcManager.adc0Channel == CHANNEL_LOW_STOCK ) {
                    adcChanCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_SHOOT;
                    adcManager.adcComplete = false;    
                                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
                    
                } else if( adcManager.adc0Channel == CHANNEL_SHOOT ) {
                    adcChanCfg.channelNumber = PAPER_TAKEUP_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_PAPER_TAKEUP;
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );    
                    
                } else if( adcManager.adc0Channel == CHANNEL_PAPER_TAKEUP ) {
                    adcChanCfg.channelNumber = HEAD_TEMPERATURE_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_HEAD_TEMP;                
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );      
                    
                } else if( adcManager.adc0Channel == CHANNEL_HEAD_TEMP ) {
                    adcChanCfg.channelNumber = HEAD_UP_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_HEAD_STATE;
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );   
                    
                } else if( adcManager.adc0Channel == CHANNEL_HEAD_STATE ) {
                    adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_TAKEN;
                    
                    adcManager.adcComplete = false;    
                    adcOnce_ = true;
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );   
                    
                } else {
                    PRINTF("sensorsTask(): Unknown channel!\r\n" );
                }          
            }        
        }
        /* do not read until after we are through the chain */ 
        if( adcOnce_ ) {
            readADChannels();
            #if 0   
            dbCnt_++;
            if( dbCnt_ == 10000 ) {
                showADCReadings();
                dbCnt_ = 0;
            }
            #endif
            adcOnce_ = false;
        }
        
        /* check to see if any state has changed and notify the host */         
        taskYIELD();
    }
    vTaskSuspend(NULL); 
}

/******************************************************************************/
/*!   \fn static void initManager( ADCManager *pMgr )

      \brief
        This function initializes the ADC manager for manual mode operation. 
        Setup of current working channel and channel list.

      \author
          Aaron Swift
*******************************************************************************/
static void initManager( ADCManager *pMgr )
{
    /* setup current channel */    
    pMgr->adc0Channel = CHANNEL_TAKEN;
     
    /* setup our channel list */
    pMgr->channelList[ 0 ] =  CHANNEL_TAKEN;
    pMgr->channelList[ 1 ] =  CHANNEL_LOW_STOCK;
    pMgr->channelList[ 2 ] =  CHANNEL_SHOOT;
    pMgr->channelList[ 3 ] =  CHANNEL_PAPER_TAKEUP;
    pMgr->channelList[ 4 ] =  CHANNEL_HEAD_TEMP;
    pMgr->channelList[ 5 ] =  CHANNEL_HEAD_STATE;
    pMgr->channelList[ 6 ] =  CHANNEL_HEAD_DOT;
    pMgr->channelList[ 7 ] =  CHANNEL_HEAD_DETECT;
    pMgr->adcComplete      = false;
    pMgr->adcPaused        = false;
    pMgr->avgTakeup        = false;
}

TaskHandle_t getSensorsHandle(){
    return pHandle_;
}

void closeAdc1( void )
{
    ADC_Deinit( ADC2 );  
}

/******************************************************************************/
/*!   \fn void initializeSensors( void )                                                             
 
      \brief
        This function initializes the lp5521 sensor driver and sets current 
        values for shoot through and label taken sensors.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeSensors( void )
{
    Pr_Config                   cfg;
    FPMBLC3Checksums            checkSums;
    unsigned int                checksum = 0;
    
    HeadType_t head = getPrintHeadType();
    
    /* initialize sensor driver */
    if( initLp5521() ) {
        PRINTF("initializeSensors(): LP5521 initialized.\r\n" );
    } else {
        PRINTF("initializeSensors(): Failed to initialize LP5521.\r\n" );
    }

    if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
        initPWM2Sensors();
    }
    
    /* read printer configuration for bias settings of shoot through and label
       taken sensors */    
    if( getSerialPrConfiguration( &cfg ) ) {   
        /* check for valid configuration */
        if( getPageChecksums( &checkSums ) ) {
            checksum = calculateChecksum((void *)&cfg, sizeof(Pr_Config));
            if( checksum == checkSums.prConfigSum ) {
                if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
                  
                    if( cfg.shootThroughCal > 100 ) {
                        cfg.shootThroughCal = 100; 
                    }                    
                    setMediaSensorDutyCycle( (uint8_t)cfg.shootThroughCal );
                } else {
                    /* set shoot through sensor bias */
                    if( !setGapCurrent( cfg.shootThroughCal ) ) {
                        PRINTF("initializeSensors(): failed to set gap bias!\r\n" );
                    } 
                }
                
                /* set paper take up sensor bias */
                if( !setPaperTakeupCurrent( cfg.takeupDriveCurrent ) ) {   
                    PRINTF("initializeSensors(): failed to set label taken bias!\r\n" );
                } 
                
                /* set low stock sensor bias */
                if( !setLowStockCurrent( CC_TWELVE_POINT_ZERO ) ) {
                    PRINTF("initializeSensors(): failed to set low stock bias!\r\n" );
                }
                
                /* set low stock sensor duty cycle DC_90_PRECENT*/
                if( !setLowStockDutyCycle( DC_90_PRECENT ) ) {
                    PRINTF("initializeSensors(): failed to set low stock duty cycle!\r\n" );
                }
            } else {
                PRINTF("initializeSensors(): Printer config unusable, settting sensor defaults\r\n" );
                
                setGapCurrent( CC_TWO_POINT_FIVE );
                setPaperTakeupCurrent( CC_SIX_POINT_TWO );
            }
        } else {
            PRINTF("initializeSensors(): Failed to read checksums, settting sensor defaults\r\n" );
        }    
    } else {
        PRINTF("initializeSensors(): Failed to read config, settting sensor defaults\r\n" );
    }

    /* connect flextimer 1 output to gpio label taken clock */
    initXbar( mode_ );

    /* start pwm clock to label taken sensor */  
    initLabelTakenLED();    
    
    /* pit timer required if running adc in auto mode */
    if( mode_ == AD_AUTO ) {
        initPitADC();
    }
    
    headTemperature = getPrintheadTemperatureInCelsius(); 
    
    initializeAdcs( mode_ );     
}


/******************************************************************************/
/*!   \fn void initializeAdcs( void )                                                            
 
      \brief
        This function initializes module 0 A/D converter.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeAdcs( ADConfig configuration )
{
    adc_config_t adcConfig;    
    static bool calOnce_ = false;
  
    if( configuration == AD_MANUAL ) { 
        ADC_GetDefaultConfig( &adcConfig ); 
        
        adcConfig.resolution = kADC_Resolution12Bit;
        adcConfig.enableOverWrite = true;
        adcConfig.enableHighSpeed = true;
        adcConfig.samplePeriodMode = kADC_SamplePeriodShort2Clocks;
    
        /* initialize the adc modules */
        ADC_Init( ADC2, &adcConfig );
      
        adc_channel_config_t adcChansCfg[7];
        ADC_EnableHardwareTrigger( ADC2, false );
        
        adcChansCfg[0].channelNumber = LOW_STOCK_CHANNEL;
        adcChansCfg[0].enableInterruptOnConversionCompleted = true;

        adcChansCfg[1].channelNumber = SHOOT_SENSOR_CHANNEL;
        adcChansCfg[1].enableInterruptOnConversionCompleted = true;

        adcChansCfg[2].channelNumber = PAPER_TAKEUP_CHANNEL;
        adcChansCfg[2].enableInterruptOnConversionCompleted = true;

        adcChansCfg[3].channelNumber = HEAD_TEMPERATURE_CHANNEL;
        adcChansCfg[3].enableInterruptOnConversionCompleted = true;

        adcChansCfg[4].channelNumber = HEAD_DOT_CHANNEL;
        adcChansCfg[4].enableInterruptOnConversionCompleted = true;
        
        adcChansCfg[5].channelNumber = HEAD_UP_CHANNEL;
        adcChansCfg[5].enableInterruptOnConversionCompleted = true;

        adcChansCfg[6].channelNumber = HEAD_DETECT_CHANNEL;
        adcChansCfg[6].enableInterruptOnConversionCompleted = true;
        
        ADC_SetChannelConfig( ADC2, 0, &adcChansCfg[0] );   
        
    } else {

        adc_channel_config_t adcChannelConfigStruct;
        
        ADC_GetDefaultConfig( &adcConfig );
        ADC_Init( ADC2, &adcConfig );
        ADC_EnableHardwareTrigger( ADC2, true );

        adcChannelConfigStruct.channelNumber = 10U; /* head up */
        adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
        ADC_SetChannelConfig( ADC2, 0U, &adcChannelConfigStruct );
        
        adcChannelConfigStruct.channelNumber = 11U; /* head dot */
        ADC_SetChannelConfig( ADC2, 1U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 9U; /* head temp */
        ADC_SetChannelConfig( ADC2, 2U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 8U; /* paper takeup */
        ADC_SetChannelConfig( ADC2, 3U, &adcChannelConfigStruct );
        
        adcChannelConfigStruct.channelNumber = 7U; /* shoot through */
        ADC_SetChannelConfig( ADC2, 4U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 0U; /* label taken */
        ADC_SetChannelConfig( ADC2, 5U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 1U; /* low stock */
        ADC_SetChannelConfig( ADC2, 6U, &adcChannelConfigStruct );
        
        adcChannelConfigStruct.channelNumber = 2U; /* head detection */
        ADC_SetChannelConfig( ADC2, 7U, &adcChannelConfigStruct );

    }

    if( !calOnce_ ) {
        /* auto hardware calibration adc module. */
        if( configuration == AD_AUTO ) {                                           
            if( kStatus_Success != ADC_DoAutoCalibration( ADC2 ) ) {
                PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
            } else {
                calOnce_ = true;
            }                
        } else {
            if( kStatus_Success != ADC_DoAutoCalibration( ADC2 ) ) {
                PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
            } else {
                calOnce_ = true;
            }
        }
    }

    if( configuration == AD_AUTO ) {                                     
        /* initialize adc etc module */  
        initializeEtcAdc( configuration );
    }

    if( configuration == AD_AUTO ) {
        EnableIRQ( ADC_ETC_IRQ0_IRQn );
        EnableIRQ( ADC_ETC_ERROR_IRQ_IRQn );
        
        /* start pit timer channel0. */
        PIT_StartTimer( PIT, kPIT_Chnl_0 );            
    } else {
        NVIC_SetPriority( ADC2_IRQn, 5 );        
        EnableIRQ( ADC2_IRQn );           
    }
}

/******************************************************************************/
/*!   \fn void initLabelTakenLED( void )                                                            
 
      \brief
        This function initializes the quad flex timer for generating a 10Khz
        20% duty cycle signal to the label taken sensor. The xbar is used to 
        connect the timer output to the gpio pin of the sensor.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initLabelTakenLED( void )
{
    /* enable the label taken sensor */
    //enable pin must be held high for >= 50ns before going low to allow C3 to discharge fully
    GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, false ); 
    
    
    for(int wait = 0; wait < 30000; wait++)
    {
        for(int wait2 = 0; wait2 < 20000; wait2++)
        {
            __NOP();
        }
    }
    
    GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, true ); 
  
    for(int wait = 0; wait < 30000; wait++)
    {
        for(int wait2 = 0; wait2 < 20000; wait2++)
        {
            __NOP();
        }
    }
    
    GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, false );
  
    qtmr_config_t qtmrConfig;
    
    QTMR_GetDefaultConfig( &qtmrConfig );
    /* initial the output channel. */
    qtmrConfig.primarySource = kQTMR_ClockDivide_4;
    QTMR_Init( TMR1, kQTMR_Channel_1, &qtmrConfig );

    HeadType_t head = getPrintHeadType();
      
    if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
        QTMR_SetupPwm( TMR1, kQTMR_Channel_1, 1000U, 12U, false, ( CLOCK_GetFreq( kCLOCK_IpgClk ) / 4U ) );
    } else {
        /* generate a 10Khz PWM signal with 20U% dutycycle */
        QTMR_SetupPwm( TMR1, kQTMR_Channel_1, 10000U, 20U, false, ( CLOCK_GetFreq( kCLOCK_IpgClk ) / 4U ) );
    }
    
    QTMR_StartTimer( TMR1, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge); 
}

/******************************************************************************/
/*!   \fn void initializeEtcAdc( ADConfig configuration )                                                     
 
      \brief
        This function initializes the ETC ADC interface for chaining channels  
        into one trigger group which is triggered every 100mS from the pit timer.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeEtcAdc( ADConfig configuration )
{
    adc_etc_config_t adcEtcConfig;
    adc_etc_trigger_config_t adcEtcTriggerConfig;
    adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;

    ADC_ETC_GetDefaultConfig( &adcEtcConfig );
    adcEtcConfig.XBARtriggerMask = 16U;          /* enable the external XBAR trigger0. */
    /* adcEtcConfig.enableTSCBypass = false; */
    ADC_ETC_Init( ADC_ETC, &adcEtcConfig );

    /* Set the external XBAR trigger0 configuration. */
    adcEtcTriggerConfig.enableSyncMode      = false;
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength  = 6U; 
    adcEtcTriggerConfig.triggerPriority     = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay        = 0U;
    ADC_ETC_SetTriggerConfig( ADC_ETC, 4U, &adcEtcTriggerConfig );
    
    /* Set the external XBAR trigger0 chain configuration. */
    adcEtcTriggerChainConfig.enableB2BMode       = true;
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 0U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 10U; /* head up */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable;
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 0U, &adcEtcTriggerChainConfig);
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 1U;
    adcEtcTriggerChainConfig.ADCChannelSelect = 11U;  /* head dot */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 1U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 2U;
    adcEtcTriggerChainConfig.ADCChannelSelect = 9U;  /* head temp */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 2U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 3U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 8U; /* paper takeup */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 3U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 4U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 7U;  /* shoot through */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 4U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 5U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 0U;  /* label taken */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 5U, &adcEtcTriggerChainConfig); 
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 6U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 1U; /* low stock */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 6U, &adcEtcTriggerChainConfig);

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 2U << 7U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 2U; /* head detect */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 7U, &adcEtcTriggerChainConfig);    
    
}

/******************************************************************************/
/*!   \fn void initXbar( ADConfig configuration )                                                         
 
      \brief
        This function initializes the the XBAR interface and connect flex timer1 
        signal to gpio pin for label taken pwm and connect pit timer to 
        adc etc module if in auto mode. 
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initXbar( ADConfig configuration )
{   
    XBARA_Init( XBARA );
    /* configure the xbara signal connections. */ 
    XBARA_SetSignalsConnection( XBARA, kXBARA1_InputQtimer1Tmr1, kXBARA1_OutputIomuxXbarInout09 ); 
    if( configuration == AD_AUTO ) {
        /* connect pit timer output to adc1 conversion */
        XBARA_SetSignalsConnection( XBARA, kXBARA1_InputPitTrigger0, kXBARA1_OutputAdcEtcTrig10 );        
    }
}

/******************************************************************************/
/*!   \fn void initPitADC( void )                                                          
 
      \brief
        This function initializes the pit timer to initiate a conversion every
        100mS. 
          
      \required xbar connection. 

      \author
          Aaron Swift
*******************************************************************************/ 
void initPitADC( void )
{
    pit_config_t pitConfig;

    /* init pit module */
    PIT_GetDefaultConfig( &pitConfig );
    PIT_Init( PIT, &pitConfig );

    PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 100U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );
   
    PIT_EnableInterrupts( PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable );   
    EnableIRQ( PIT_IRQn );
}

/******************************************************************************/
/*!   \fn void pitHandleIsr( void )                                                          
 
      \brief
        This function handles the pit timer interrupt.
                
      \author
          Aaron Swift
*******************************************************************************/ 
void pitHandleIsr( void )
{
    PIT_ClearStatusFlags( PIT, kPIT_Chnl_0, kPIT_TimerFlag );    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn void adcEtc0HandleIsr( void )                                                          
 
      \brief
        This function handles etc adc interrupt.
                
      \author
          Aaron Swift
*******************************************************************************/ 
void adcEtc0HandleIsr( void )
{    
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
 
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource  );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_Done0StatusFlagMask );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_Done1StatusFlagMask );
    /* get trigger 0 group 0 chain 0 - 6 results. */
    adcManager.value[CHANNEL_HEAD_STATE] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 0U);  /* head up */
    adcManager.value[CHANNEL_HEAD_DOT] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 1U);  /* head dot */ 
    adcManager.value[CHANNEL_HEAD_TEMP] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 2U);  /* head temp */ 
    adcManager.value[CHANNEL_PAPER_TAKEUP] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 3U);  /* paper takeup */
    adcManager.value[CHANNEL_SHOOT] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 4U);  /* shoot through */
    adcManager.value[CHANNEL_TAKEN] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 5U);  /* label taken */ 
    adcManager.value[CHANNEL_LOW_STOCK] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 6U);  /* low stock */
    adcManager.value[CHANNEL_HEAD_DETECT] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 7U);  /* head detect */ 
	 
	/* used during calibration of takeup sensor */
    if( adcManager.avgTakeup ) {
        //running average for TAKEUP
		TakeupMediaBufferSum = TakeupMediaBufferSum - TakeupMediaBuffer[TakeupMediaBufferPtr];
		TakeupMediaBuffer[TakeupMediaBufferPtr] = adcManager.value[CHANNEL_PAPER_TAKEUP];
		TakeupMediaBufferSum += TakeupMediaBuffer[TakeupMediaBufferPtr];  
		adcManager.value[CHANNEL_PAPER_TAKEUP] = TakeupMediaBufferSum/TAKEUPMEDIABUFFERSIZE;
		
		TakeupMediaBufferPtr++;
		if(TakeupMediaBufferPtr >= TAKEUPMEDIABUFFERSIZE)
		   TakeupMediaBufferPtr = 0;
    }
    
    adcManager.adcComplete = true;
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn void adcEtcErrorHandleIsr( void )                                                          
 
      \brief
        This function handles etc adc error flags.
                
      \author
          Aaron Swift
*******************************************************************************/ 
void adcEtcErrorHandleIsr( void )
{
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource  );
    PRINTF( "adcEtcErrorHandleIsr: flags %d\r\n", flags );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_ErrorStatusFlagMask );

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}

void pauseResumeConversions( bool pause )
{
    adcManager.adcPaused = pause;
    PRINTF("ADC MANAGER PAUSE = %d\r\n", pause);
}

/******************************************************************************/
/*!   \fn unsigned short getShootThroughConversion( void )                                                           
 
      \brief
        This function sets up the shoot through channel and starts a conversion.
        This function blocks until the channel conversion is complete.
        

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getShootThroughConversion( void )
{
    if( mode_ != AD_AUTO ) {  
        adc_channel_config_t adcChanCfg;  
        
        adcChanCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = true;    
        adcManager.adc0Channel = CHANNEL_SHOOT;
        adcManager.adcComplete = false;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while( !adcManager.adcComplete ) {
            taskYIELD();      
        }
    } 
    return adcManager.value[ CHANNEL_SHOOT ];
}

/******************************************************************************/
/*!   \fn unsigned short getShootThroughConversion( void )                                                           
 
      \brief
        This function sets up the shoot through channel and starts a conversion.

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short pollMediaCounts( void )
{

    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
        
        adcChanCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_SHOOT;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {
        
        }
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        return result;
    } else {
        return adcManager.value[CHANNEL_SHOOT];
    }
    
}

/******************************************************************************/
/*!   \fn bool isADCAutoMode( void )                                                           
 
      \brief
        This function return true if adc is setup for auto mode of operation.

      \author
          Aaron Swift
*******************************************************************************/ 
bool isADCAutoMode( void )
{
    bool result = false;
    if( mode_ == AD_AUTO ) {
        result = true;
    }
    return result;
}


/******************************************************************************/
/*!   \fn void outOfMediaFilter(void)                                                   
 
      \brief Filters the Media (Shoot Through) sensor. 
           Looks for 200 consecutive out of media readings prior to setting
           "outOfMedia" true. One reading above the threshold resets the counter.
           Since "out of media" is a DC event (i.e. not fleeting), out of media
           events won't be missed because of this filter. This function is called
           from adcEtc0HandleIsr which executes every 300nS so 200 consecutive
           readings is only about 65uS!. We may want to bump this sample size up.

      \author
          Tom Fink
*******************************************************************************/
void outOfMediaFilter(void)   //TFinkMediaFilter
{
   /* a sample of one showed a shoot through sensor reading of 81.4%,
   so we may be able to lower the 0.94 threshold */
   
   static unsigned short copyOfBackingPaper;
   static unsigned short OutOfMediaThreshold;
   static unsigned short consecutiveOutOfMediaSamples = 0;
   static bool initialized = false;
   if(!initialized) {
      /* we've already checked OUT_OF_MEDIA in system startup - prior to the ADCETC ISR running */
      if((currentStatus.sensor & OUT_OF_MEDIA) == OUT_OF_MEDIA)
         consecutiveOutOfMediaSamples = 255;
      
      copyOfBackingPaper = config_.backingPaper;
      OutOfMediaThreshold = (unsigned short)((float)config_.backingPaper * OUT_OF_MEDIA_THRESHOLD);

      /* testig has shown that the "no label" value is generally below 260 counts
         in a calibrated system */
      if(OutOfMediaThreshold > 300)
         OutOfMediaThreshold = 300; 
      
      initialized = true;
   }
   
   if(copyOfBackingPaper != config_.backingPaper)   /* Handle Media Sensor calibration */
      initialized = false;
    
   if(adcManager.value[CHANNEL_SHOOT] < OutOfMediaThreshold)  {
      if(consecutiveOutOfMediaSamples < 255)
         consecutiveOutOfMediaSamples++;
   }
   else
      consecutiveOutOfMediaSamples = 0;

   /* shoot through reading invalid if cassette not loaded */
   if(headSensors.headUp > 1000)   
      consecutiveOutOfMediaSamples = 0;
   
   if(consecutiveOutOfMediaSamples > 0)  
      outOfMedia = true;
   else
      outOfMedia = false;
}

bool getOutOfMedia(void)
{
   return outOfMedia;
}

/******************************************************************************/
/*!   \fn void setTakeupFiltering( void )                                                      
 
      \brief
        This function sets the takeup average flag to true.

      \author
          Aaron Swift
*******************************************************************************/ 
void setTakeupFiltering( void )
{
    adcManager.avgTakeup = true;
    adcManager.tkIndex = 0;
}

/******************************************************************************/
/*!   \fn void setTakeupFiltering( void )                                                      
 
      \brief
        This function sets the takeup average flag to false.

      \author
          Aaron Swift
*******************************************************************************/ 
void clrTakeupFiltering( void )
{
    adcManager.avgTakeup = false; 
    memset( (void *)&adcManager.tkFilter[0], 0, TK_MAX_SAMPLES);
}

/******************************************************************************/
/*!   \fn void readADChannels( void )                                                            
 
      \brief
        This function reads all A/D convertor channels from the ADCManager.
        The ADCManager is automaticaly updated through the pdb/adc/edma chain.
        The pdb generates a channel interrupt every 333uS.

      \author
          Aaron Swift
*******************************************************************************/ 
void readADChannels( void )
{ 
    /* intiate new conversion */
    headSensors.headVoltage   = adcManager.value[CHANNEL_HEAD_DOT];
    headTemperature           = getPrintheadTemperatureInCelsius();           
    headSensors.mediaDetector = adcManager.value[CHANNEL_SHOOT];   
    headSensors.headUp        = adcManager.value[CHANNEL_HEAD_STATE];
    headSensors.labelTaken    = readLabelTakenSensor();      
    
    readHeadTemperatureSensor( &currentStatus );
    readHeadUpSensor( &currentStatus );
    processLabelTakenSensor( &currentStatus );
    
    HeadType_t head = getPrintHeadType();
  
    prevHeadUp = currentHeadUp;
    
    if( headSensors.headUp > NO_CASSET_THRESHOLD ) {
        currentHeadUp = true;
        
        currentStatus.sensor2 &= ~JAMMED_LABEL;
        currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
        currentStatus.sensor &= ~OUT_OF_MEDIA;  //Don't report Out of Media if head is up
        
        if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
            if( GTXHeadUpCounter == 0 ) {
                PRINTF("\r\nhead up curr: %d thresh: %d\r\n", headSensors.headUp, NO_CASSET_THRESHOLD);
            }            
            GTXHeadUpCounter++;
            
            if( GTXHeadUpCounter > 5000 ) {
                GTXHeadUpCounter = 0;
            }
        } else {
            setLabelQueuePaused( false );
            setLabelPauseBackwindPending( false );
            setLabelPauseTimeout( 0 );
            
            if( getTakingUpPaper() == false ) {
                resetLabelLowVars();
            }
        }
        
        setTakeupBusy( false );        
        setHeadPower( false );        
        compareStatus( &currentStatus, &prevStatus );
    } else {
        currentHeadUp = false;
    }

    if( currentHeadUp == false && prevHeadUp == true ) {
        firstPrint = true;
        GTXHeadUpCounter = 0;
        
        if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
            PRINTF("\r\nhead down curr: %d thresh: %d", headSensors.headUp, NO_CASSET_THRESHOLD);
        }
        cutterBladeDelayNeeded = true;
      
        setTUCalStatus( false );
        
        if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
            __NOP();
        } else {
            setLabelQueuePaused( false );
            setLabelPauseBackwindPending( false );
            setLabelPauseTimeout( 0 );
        }
        
        setTakeupBusy( false );
        
        currentStatus.sensor2 &= ~JAMMED_LABEL;
        currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
        
        compareStatus( &currentStatus, &prevStatus );
        
        if( ( currentStatus.mask.user == 1 ) && ( getGapCalStatus() == false ) && ( outOfMedia == false ) ) {
            checkForPaper( (uint16_t)( (float)config_.takeupMaxTension * 0.90 ), 820 );
               
            setHalfStepMode( _MAIN_STEPPER );
            setHalfStepMode( _TAKEUP_STEPPER );
          
            setTakeUpMotorDirection( FORWARDM_ ); 
            setMainMotorDirection( BACKWARDM_ );

            PRINTF("cutter expel mask.user = %d", currentStatus.mask.user);
            
            if( ( currentStatus.mask.user == 1 ) && ( getGapCalStatus() == false ) ) {    
                PRINTF("\r\ncutter expel queued");
             
                setLabelQueuePaused( true );
                
                setOperation( IDLE_DIRECTIVE, &currentStatus );
                
                if( getCutterInstalled() == true ) {
                    PRINTF("cutter expel");
                    stepToNextLabel( 475, 1000 );
                    
                    while( getTakeupBusy() == true ) {
                        __NOP();
                    }
                } else {
                  
                    PRINTF("continuous expel");
                    stepToNextLabel( 800, 1000 );
                    setBackwindAfterSizing( true );
                    
                    while( getTakeupBusy() == true ) {
                        __NOP();
                    }
                }
            }
        }    
    }
    
    compareStatus(&currentStatus, &prevStatus);
}

/******************************************************************************/
/*!   \fn UINT readHeadVoltage(void)                                                            
 
      \brief
        This function returns the adc reading of the print head voltage.                            
          
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned int readHeadVoltage( void )
{  
    return( headSensors.headVoltage );
}

/******************************************************************************/
/*!   \fn void readMediaSensor( PrStatusInfo *pStatus )                                                           
 
      \brief
        This function sets and clears the associated media sensor bits 
        in the PrStatusInfo structure.    
                                           
      \author
          Aaron Swift
*******************************************************************************/ 
 //TFinkMediaFilter  This is how I found this functin. It wasnt doing anything.
void readMediaSensor( PrStatusInfo *pStatus )
{  
 
    /* leave in for printing routines */
    if( getOutOfMediaCounts() < getMaxOutOfMediaCounts() ) {
        //pStatus->sensor &= ~OUT_OF_MEDIA;
        //pStatus->error &= ~MEDIA_SHUTDOWN;
    } else {
        /* removed for freestanding scale -- replace with shoot-through
        pStatus->sensor |= OUT_OF_MEDIA;
        */        
    }
}


/******************************************************************************/
/*!   \fn unsigned long getLabelTaken( void ) 

      \brief
        This function returns the a/d channel conversion for the label taken
        sensor.

      \return raw a/d value for the label taken sensor.
      
      \author
          Aaron Swift
*******************************************************************************/
unsigned long getLabelTaken( void ) 
{
  
  if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
        
        adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_TAKEN;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {
        
        }
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        return result;
    } else {
        return adcManager.value[CHANNEL_TAKEN];
    }
     
}

/******************************************************************************/
/*!   \fn void readLabelTakenSensor( void )

      \brief
        This function reads the label taken sensor gpio pin. Modified this 
        function for freestanding self service scale due to ESD coupling to   
        the sensor detector which is causing motor chatter. This filtering 
        of the signal should improve the issue.

      \return true when label is present.
      
      \author
          Aaron Swift
*******************************************************************************/
bool readLabelTakenSensor( void )
{
    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
    
        adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_TAKEN;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {};
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        
        return result;
    } else {
        static bool previousState_ = false; 

        /* only accept values above and below threshold throw all 
        other values out due to noise. */
        
        if(LTSamplesTaken <= 5000)
        {
            LTSamplesTaken++;
        }
        
        if(LTSamplesTaken == 4999)
        {
           //PRINTF("\r\nLT samples taken %d %d", LTSamplesTaken, adcManager.value[CHANNEL_TAKEN]);
        }
        
        if(adcManager.value[CHANNEL_TAKEN] < lowestLabelTakenReading && LTSamplesTaken >= 5000)
        {
            LABEL_TAKEN_THRESHOLD_LABEL = (LABEL_TAKEN_THRESHOLD_LABEL - (lowestLabelTakenReading - adcManager.value[CHANNEL_TAKEN]));
            LABEL_TAKEN_THRESHOLD_NO_LABEL = LABEL_TAKEN_THRESHOLD_LABEL - 1;
          
            lowestLabelTakenReading = adcManager.value[CHANNEL_TAKEN]; 
        }
       
        if( adcManager.value[CHANNEL_TAKEN] <= LABEL_TAKEN_THRESHOLD_NO_LABEL ) {
            previousState_ = true;
            return( true );
        } else if( adcManager.value[CHANNEL_TAKEN] > LABEL_TAKEN_THRESHOLD_LABEL ) {
            previousState_ = false;
            return( false );
        } else {
            return( previousState_ ); 
        }  
    }  
}

/******************************************************************************/
/*!   \fn void processLabelTakenSensor( PrStatusInfo *pStatus )

      \brief
        This function sets and clears the associated
         label taken sensor bits in the PrStatusInfo structure.

      \author
          Aaron Swift
*******************************************************************************/
void processLabelTakenSensor( PrStatusInfo *pStatus )
{ 
    if( headSensors.labelTaken ) 
    { 
        //NO LABEL IN FRONT OF SENSOR
        setLTWaitCount_(0);
      
        currentStatus.sensor |= LABEL_TAKEN;          
    } 
    else 
    {        
        //LABEL IN FRONT OF SENSOR
        if(getLTWaitCount_() < LABEL_TAKEN_DELAY_LENGTH)
        {
            setLTWaitCount_(getLTWaitCount_() + 1);
        }
        else
        {
            if(getFirstPrint() == true)
            {
                currentStatus.sensor |= LABEL_TAKEN;  
            }
            else
            {
                currentStatus.sensor &= ~LABEL_TAKEN;
            }
            
        }   
    }

    compareStatus(&currentStatus, &prevStatus);
}

/******************************************************************************/
/*!   \fn int getGhostMCntr( void )

      \brief
        This function returns the counter that tracks the number of label 
        present bits while printing a label. This is used to detect false or
        ghost missing label occurences.

      \author
          Aaron Swift
*******************************************************************************/
int getGhostMCntr( void )
{
    return ghostM_;
}

void clearGhostMCntr( void )
{
    ghostM_ = 0;   
}


/******************************************************************************/
/*!   \fn void readHeadUpSensor( PrStatusInfo *pStatus )                                                           
 
      \brief
        This function sets and clears the associated head up 
        sensor bits in the PrStatusInfo structure.  
  
                                           
      \author
          Aaron Swift
*******************************************************************************/ 
bool readHeadUpSensor( PrStatusInfo *pStatus )
{
    static unsigned char cnt_ = 0;
    
    if(TUCalHeadUpFlag == false)
    {
        if( headSensors.headUp < NO_CASSET_THRESHOLD ) 
        {  
            cnt_ = 0;
            pStatus->error &= ~HEAD_UP;   
        } 
        else 
        { 
            cnt_++;
            if( cnt_ >= HEAD_UP_DEBOUNCE ) 
            {
                cnt_ = 0;                
                pStatus->error |= HEAD_UP;
                resetTUTorqueControlVars();
                resetPeelLogStateVars(HEAD_UP_DETECTED);
            }
        }
    }
    else
    {
        pStatus->error &= ~HEAD_UP;
    }

    
    /* gloabl and freestanding does not support narrow label */
    pStatus->sensor |= WIDE_LABEL;    
    
    if(headSensors.headUp < NO_CASSET_THRESHOLD)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/******************************************************************************/
/*!   \fn void readLowStockSensor( PrStatusInfo *pStatus )                                                          
 
      \brief
        This function sets and clears the associated low stock 
        sensor bits in the PrStatusInfo structure.  
  
                                           
      \author
          Aaron Swift
*******************************************************************************/ 
void readLowStockSensor( PrStatusInfo *pStatus )
{
  //TODO: This needs fixed later when we figure out where and when we want to sample this sensor,
  //we are meant to be measuring the frequency of the change in the big and little dip as they rotate  
  
    /* only report out of stock if cassette is installed */
    if( ( pStatus->error & HEAD_UP ) != HEAD_UP ) {
        /* check if we are out of stock */
        if( adcManager.value[CHANNEL_LOW_STOCK] >= MEDIA_OUT_OF_THRESHOLD  ) {
            //pStatus->sensor2 |= OUT_OF_STOCK;
        } else {
            //pStatus->sensor2 &= ~OUT_OF_STOCK;
        }
    }
}

/******************************************************************************/
/*!   \fn unsigned short getPrintheadThermistorCounts()                                                         
 
      \brief
        Returns the ADC counts for the printhead thermistor 
                             
      \author
          Chris King
*******************************************************************************/ 
unsigned short getPrintheadThermistorCounts() 
{
    return adcManager.value[CHANNEL_HEAD_TEMP];
}

unsigned short getTakeUpTorque( void )
{
    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
    
        adcChanCfg.channelNumber = PAPER_TAKEUP_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_PAPER_TAKEUP;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {};
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        
        return result;
    } else {
       return adcManager.value[CHANNEL_PAPER_TAKEUP]; 
    }
}

/******************************************************************************/
/*!   \fn unsigned short getPaperTakeUp( void )
 
      \brief
        This function returns the takeup torque in raw counts
                                                     
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getPaperTakeUp( void )
{
    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
    
        adcChanCfg.channelNumber = PAPER_TAKEUP_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_PAPER_TAKEUP;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {};
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        
        return result;
    } else {
       return adcManager.value[CHANNEL_PAPER_TAKEUP]; 
    }
}

unsigned short getHeadVoltage( void )
{
    return adcManager.value[CHANNEL_HEAD_DOT];  
}
/******************************************************************************/
/*!   \fn void readHeadTemperatureSensor( PrStatusInfo *pStatus )                                                         
 
      \brief
        This function sets and clears the associated head temperature 
        sensor bits in the PrStatusInfo structure.  
                                             
      \author
          Aaron Swift
*******************************************************************************/ 
void readHeadTemperatureSensor( PrStatusInfo *pStatus )
{
    int printheadTemperature = getPrintheadTemperatureInCelsius();

    if( printheadTemperature >= HIGH_HEAD_TEMPERATURE_WARNING_ROHM || printheadTemperature <= LOW_HEAD_TEMPERATURE_WARNING_ROHM ) 
    {          
        /* reset debounce counter */
        thermalOverloadDebounceCounter = 0;
      
        /* set the sensor flag */
        pStatus->sensor |= THERMAL_OVERLOAD; 

        if( ( printheadTemperature >= HIGH_HEAD_TEMPERATURE_LIMIT_ROHM || printheadTemperature <= LOW_HEAD_TEMPERATURE_LIMIT_ROHM ) && pStatus->state != ENGINE_PRINTING ) 
        {
            pStatus->error |= THERMAL_SHUTDOWN;
            
            /* set printhead power OFF */
            setHeadPower(false);
        } 
    } 
    else 
    {
        /* increment debounce counter */
        thermalOverloadDebounceCounter++; /* THERMAL_SHUTDOWN_DEBOUNCE_COUNT == 5000, ~3 seconds */
                   
        if( thermalOverloadDebounceCounter >= THERMAL_SHUTDOWN_DEBOUNCE_COUNT )
        {
            /* clear the sensor and error flag */
            pStatus->sensor &= ~THERMAL_OVERLOAD; 
            pStatus->error &= ~THERMAL_SHUTDOWN; 
            
            /* reset debounce counter */
            thermalOverloadDebounceCounter = 0;
        }
    }        
}

/******************************************************************************/
/*!   \fn unsigned char getHeadSupplyCurrent( void )                                                         
 
      \brief
        This function returns the power supply current.         
                                             
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned char getHeadSupplyCurrent( void )
{
    return headSensors.supplyCurrent;
}

/******************************************************************************/
/*!   \fn getCurrentSensors( PrSensors *pSensors )                                              
 
      \brief
        This function returns the current sensor readings.      
                                             
      \author
          Aaron Swift
*******************************************************************************/ 
void getCurrentSensors( PrSensors *pSensors )
{
    pSensors->headup_reading                    = headSensors.headUp;
    pSensors->head_current_reading              = adcManager.value[CHANNEL_HEAD_STATE];        
    pSensors->head_temperature_reading          = getPrintheadTemperatureInCelsius();
    pSensors->head_voltage_reading              = adcManager.value[CHANNEL_HEAD_DOT];
    pSensors->label_high_average                = 0;
    pSensors->label_low_average                 = 0;
    
    if( !headSensors.labelTaken ) {
        pSensors->label_reading                 = (int)255;
    } else {
        pSensors->label_reading                 = 0;  
    }
    
    pSensors->label_threshold                   = 0;    
    pSensors->label_width_reading               = adcManager.value[CHANNEL_HEAD_STATE];
    pSensors->media_high_average                = 0;
    pSensors->media_low_average                 = 0;
    pSensors->media_threshold                   = 0;
    pSensors->media_reading                     = adcManager.value[CHANNEL_SHOOT];
    pSensors->takeup_reading                    = adcManager.value[CHANNEL_PAPER_TAKEUP];
}

/******************************************************************************/
/*!   \fn void adcModule2IsrHandler(void)                                            
 
      \brief
        This function handles adc2 interrupts. Handler currently read the 
        conversion value for each adc channel.

      \author
          Aaron Swift
*******************************************************************************/ 
void adcModule2IsrHandler( void )
{
    
    if( mode_ != AD_AUTO ) {
        if( adcManager.adc0Channel == CHANNEL_TAKEN ) {
            adcManager.value[0] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_LOW_STOCK ) {
            adcManager.value[1] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_SHOOT ) {
            adcManager.value[2] = ADC_GetChannelConversionValue( ADC2, 0 );
            
            if( motorStopGap_ ) {
                if( adcManager.value[2] <= BACKING_PAPER_THRESHOLD ) {
                    powerOffStepper();
                }
            }
            
            if( motorStopEdge_ ) {
                if( adcManager.value[2] >= LABEL_EDGE_THRESHOLD ) {
                    powerOffStepper();
                }
            }
            
        } else if( adcManager.adc0Channel == CHANNEL_PAPER_TAKEUP ) {
            adcManager.value[3] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_HEAD_TEMP ) {
            adcManager.value[4] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_HEAD_STATE )  {
            adcManager.value[5] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_HEAD_DOT )  {
            adcManager.value[6] = ADC_GetChannelConversionValue( ADC2, 0 );
        } 
    }
    adcManager.adcComplete = true;
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn unsigned short sampleHeadDotADC( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleHeadDotADC( void )
{   
    return adcManager.value[CHANNEL_HEAD_DOT];
}


/******************************************************************************/
/*!   \fn unsigned short sampleHeadDot( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleHeadDot( void )
{
    return adcManager.value[CHANNEL_HEAD_DOT];
}

/******************************************************************************/
/*!   \fn unsigned short sampleHeadVoltage( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleHeadVoltage( void )
{
    return adcManager.value[CHANNEL_HEAD_DOT];
}

/******************************************************************************/
/*!   \fn unsigned short getMediaCounts()                                           
 
      \brief
        This function returns the raw ad counts for the shoot through / 
        reflective channel. 

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getMediaCounts() 
{    
    return adcManager.value[CHANNEL_SHOOT];
}

void showLabelTakenCounts( void )
{
    PRINTF( "label taken counts: %d\r\n", adcManager.value[CHANNEL_TAKEN] );
}

void showPaperTakeup( void )
{
    PRINTF( "paper takeup counts: %d\r\n", adcManager.value[CHANNEL_PAPER_TAKEUP] );   
}

/******************************************************************************/
/*!   \fn unsigned short getLowStockSensor( void )                                     
 
      \brief
        This function returns the raw ad counts for the low stock channel.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getLowStockSensor( void )
{
    return adcManager.value[CHANNEL_LOW_STOCK];  
}

void setMotorStopOnGap( void )
{
    motorStopGap_ = true;
}

void clrMotorStopOnGap( void )
{
    motorStopGap_ = false;    
}

void setMotorStopOnEdge( void )
{
   motorStopEdge_ = true;
}

void clrMotorStopOnEdge( void )
{
    motorStopEdge_ = false;
}


/******************************************************************************/
/*!   \fn void setLabelAlignment( unsigned int actualSize )                                          
 
      \brief
        This function sets the label vertical alignment value used to synchronize the 
        label stock.

      \author
          Aaron Swift
*******************************************************************************/ 
void setLabelAlignment( unsigned int measuredSize, unsigned int actualSize )
{
}

/******************************************************************************/
/*!   \fn void updateLabelAlignment( void )                                          
 
      \brief
        This function updates the print position of the label.

      \author
          Aaron Swift
*******************************************************************************/ 
void updateLabelAlignment( void )
{
    setLabelAlignment( measuredLength_, actualLength_ );
}


/******************************************************************************/
/*!   \fn void incrementIndexDotTest( void )                                          
 
      \brief
        This function increments the dot wear index.

      \author
          Aaron Swift
*******************************************************************************/ 
void incrementIndexDotTest( void )
{
    if( indx_ <= HEAD_DOTS_72MM ) {
        indx_++;
    }
}

/******************************************************************************/
/*!   \fn void updateMediaSensor( void )                                        
 
      \brief
        This function updates the shoot through sensor value. 

      \author
          Aaron Swift
*******************************************************************************/ 
void updateMediaSensor( void )
{
    headSensors.mediaDetector = adcManager.value[CHANNEL_SHOOT];
}

/******************************************************************************/
/*!   \fn void showADCReadings( void )                                        
 
      \brief
        This function shows the adc values measured. 

      \author
          Aaron Swift
*******************************************************************************/ 
void showADCReadings( void )
{
    PRINTF("showADCReadings(): ***********************\r\n" );
    float resolution = .00080566, value = 0.0;

    PRINTF( "label taken counts: %d\r\n", adcManager.value[CHANNEL_TAKEN] ); 
    value = (float)(adcManager.value[CHANNEL_TAKEN] * resolution);
    PRINTF( "label taken voltage: %2.3fV\r\n", value );    
    
    if( adcManager.value[CHANNEL_TAKEN] >= LABEL_TAKEN_THRESHOLD_LABEL ) {
        PRINTF( "label taken at sensor \r\n");
    } else {
        PRINTF( "label taken\r\n");
    }
    
    PRINTF( "low stock counts: %d\r\n", adcManager.value[CHANNEL_LOW_STOCK] ); 
    value = (float)(adcManager.value[CHANNEL_LOW_STOCK] * resolution);
    PRINTF( "low stock voltage: %2.3fV\r\n", value );    
    if( adcManager.value[CHANNEL_LOW_STOCK] >= MEDIA_OUT_OF_THRESHOLD  ) {
        PRINTF( "out of stock\r\n"); 
    } 

    PRINTF( "shoot through counts: %d\r\n", adcManager.value[CHANNEL_SHOOT] ); 
    value = (float)(adcManager.value[CHANNEL_SHOOT] * resolution);
    PRINTF( "shoot through voltage: %2.3fV\r\n", value );    

    
    PRINTF( "paper takeup counts: %d\r\n", adcManager.value[CHANNEL_PAPER_TAKEUP] ); 
    value = (float)(adcManager.value[CHANNEL_PAPER_TAKEUP] * resolution);
    PRINTF( "paper takeup voltage: %2.3fV\r\n", value );    

    PRINTF( "head temp counts: %d\r\n", adcManager.value[CHANNEL_HEAD_TEMP] ); 
    value = (float)(adcManager.value[CHANNEL_HEAD_TEMP] * resolution);
    PRINTF( "head temp voltage: %2.3fV\r\n", value );      
    PRINTF( "head temperature: %dC\r\n", getPrintheadTemperatureInCelsius() );             

    
    PRINTF( "head state counts: %d\r\n", adcManager.value[CHANNEL_HEAD_STATE] ); 
    value = (float)(adcManager.value[CHANNEL_HEAD_STATE] * resolution);
    PRINTF( "head state voltage: %2.3fV\r\n", value );    
    if( adcManager.value[CHANNEL_HEAD_STATE] >= NO_CASSET_THRESHOLD  ) {
        PRINTF( "head is up\r\n"); 
    } 
}

/******************************************************************************/
/*!   \fn void setADCMode( ADConfig mode )                                    
 
      \brief
        This function sets both adc modules into either auto or manual mode
        of operation. 

      \author
          Aaron Swift
*******************************************************************************/ 
void setADCMode( ADConfig mode )
{
    mode_ = mode;
}

/******************************************************************************/
/*!   \fn bool getBackwindAfterContExpel( void )

      \brief
        This function returns backwind flag. 

      \author
          Chris King
*******************************************************************************/
bool getBackwindAfterContExpel( void )
{
    return backwindAfterContExpel;
}

/******************************************************************************/
/*!   \fn void setBackwindAfterContExpel( bool backwind )

      \brief
        This function sets backwind flag. 

      \author
          Chris King
*******************************************************************************/
void setBackwindAfterContExpel( bool backwind )
{
    backwindAfterContExpel = backwind;
}

/******************************************************************************/
/*!   \fn void getTUCalHeadUpFlag( void )

      \brief
        This function returns head up flag. 

      \author
          Chris King
*******************************************************************************/
bool getTUCalHeadUpFlag( void )
{
    return TUCalHeadUpFlag;
}

/******************************************************************************/
/*!   \fn void setTUCalHeadUpFlag( bool cal )

      \brief
        This function sets head up flag. 

      \author
          Chris King
*******************************************************************************/
void setTUCalHeadUpFlag( bool cal )
{
    TUCalHeadUpFlag = cal;
}

/******************************************************************************/
/*!   \fn void getHeadUp( void )

      \brief
        This function returns head up state. 

      \author
          Chris King
*******************************************************************************/
bool getHeadUp( void )
{
    if(headSensors.headUp > NO_CASSET_THRESHOLD)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/******************************************************************************/
/*!   \fn void initPWM2Sensors( void )

      \brief
        This function intiailizes the pwm signal for media sensor.

      \author
          Chris King
*******************************************************************************/
void initPWM2Sensors( void )
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal;

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_FLEXPWM2_PWMA03, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_06_FLEXPWM2_PWMA03, 0x10B0U);
    
    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent = 0;
    pwmSignal.deadtimeValue    = 0U;
    pwmSignal.faultState       = kPWM_PwmFaultState0;

    PWM_GetDefaultConfig(&pwmConfig);
    pwmConfig.prescale        = kPWM_Prescale_Divide_1;
    pwmConfig.reloadLogic     = kPWM_ReloadImmediate;
    pwmConfig.enableDebugMode = true;
    pwmConfig.pairOperation   = kPWM_Independent;
    
    PWM_Init(PWM2, kPWM_Module_0, &pwmConfig);
    PWM_Init(PWM2, kPWM_Module_3, &pwmConfig);

    PWM_SetupPwm(
        PWM2,
        kPWM_Module_3,
        &pwmSignal,
        1U,
        kPWM_EdgeAligned,
        15000U,
        CLOCK_GetFreq(kCLOCK_IpgClk)
    );
    
    PWM_SetupPwm(
        PWM2,
        kPWM_Module_0,
        &pwmSignal,
        1U,
        kPWM_EdgeAligned,
        25265U,
        CLOCK_GetFreq(kCLOCK_IpgClk)
    );

    /* 1. Disable ALL fault mapping for this submodule */
    PWM2->SM[0].DISMAP[0] = 0x0000;
    PWM2->SM[0].DISMAP[1] = 0x0000;
    PWM2->SM[1].DISMAP[0] = 0x0000;
    PWM2->SM[1].DISMAP[1] = 0x0000;
    PWM2->SM[2].DISMAP[0] = 0x0000;
    PWM2->SM[2].DISMAP[1] = 0x0000;
    PWM2->SM[3].DISMAP[0] = 0x0000;
    PWM2->SM[3].DISMAP[1] = 0x0000;

    /* 2. Clear any latched fault status */
    PWM2->FSTS = PWM2->FSTS;
}

/******************************************************************************/
/*!   \fn void setMediaSensorDutyCycle( uint8_t dutyCycle )

      \brief
        This function set the media sensor duty cycle.

      \author
          Chris King
*******************************************************************************/
void setMediaSensorDutyCycle( uint8_t dutyCycle )
{
    pwm_signal_param_t pwmSignal;

    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent = dutyCycle;
    pwmSignal.deadtimeValue    = 0U;
    pwmSignal.faultState       = kPWM_PwmFaultState0;
    
    PWM_SetupPwm(
        PWM2,
        kPWM_Module_3,
        &pwmSignal,
        1U,
        kPWM_EdgeAligned,
        15000U,
        CLOCK_GetFreq(kCLOCK_IpgClk)
    );
    
    PWM_SetPwmLdok( PWM2, kPWM_Control_Module_3, true );
    
    PWM_StartTimer(PWM2, kPWM_Control_Module_3);     
}

/******************************************************************************/
/*!   \fn bool getCutterBladeDelayNeeded( void )

      \brief
        This function returns flag for if the cutter needs a delay. 

      \author
          Chris King
*******************************************************************************/
bool getCutterBladeDelayNeeded( void )
{
    return cutterBladeDelayNeeded;
}

/******************************************************************************/
/*!   \fn void setCutterBladeDelayNeeded( bool delayNeeded )
      \brief
        This function sets flag for if the cutter needs a delay. 

      \author
          Chris King
*******************************************************************************/
void setCutterBladeDelayNeeded( bool delayNeeded )
{
    cutterBladeDelayNeeded = delayNeeded;
}

/******************************************************************************/
/*!   \fn uint16_t getLowestLabelTakenReading( void )
      \brief
        This function gets lowest label taken value.

      \author
          Chris King
*******************************************************************************/
uint16_t getLowestLabelTakenReading( void )
{
    return lowestLabelTakenReading;
}
/******************************************************************************/
/******************************************************************************/


/************************** avery public functions ****************************/
/******************************************************************************/

/******************************************************************************/
/*!   \fn long readPrinterAdc( unsigned long whichAdc )

      \brief
        This function returns 10/12 bit adc value from appropriate channel.

      \author
          Nick Barnes 
*******************************************************************************/
long readPrinterAdc(unsigned long whichAdc)
{
	adc_channel_config_t adcChannelConfigStruct;
	long adcVal = 0;
	uint32_t channel[NUM_ADC] = 
		{
			7U,  /* ADC_GAP:       SHOOT_THROUGH_SENSOR   GPIO_AD_B1_07  J10A.9  */
			9U,  /* ADC_TEMP:      PHEAD_THERM_SENSOR     GPIO_AD_B1_09  J24.13  */
			8U,  /* ADC_TK_UP:     PAPER_TAKEUP_SENSOR    GPIO_AD_B1_08  J10A.13 */
			1U,  /* ADC_MEDIA:     LOW_LABEL_STOCK_SENSOR GPIO_AD_B0_14  J10A.5  */
			11U, /* ADC_24V:       PHEAD_DOT              GPIO_AD_B1_11  J24.3,4,5,6, V24_PH */
			0U   /* ADC_LBL_TAKEN: LABEL_TAKEN_SENSOR     GPIO_AD_B0_13  J26.5   */
		};
        

	if (whichAdc < NUM_ADC) 
	{
		#define ADC_COUNT_MAX 6
		#define ADC_LOOP_MAX  3
		
		uint8_t adc_count;
		uint8_t adc_loop;

		/* Configure the user channel and interrupt. */
		adcChannelConfigStruct.channelNumber                        = channel[whichAdc];
		adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
		
		/*
			When in software trigger mode, each conversion would be launched once calling the "ADC_ChannelConfigure()"
			function, which works like writing a conversion command and executing it. For another channel's conversion,
			just to change the "channelNumber" field in channel's configuration structure, and call the
			"ADC_ChannelConfigure() again.

			Do up to 2 tries to request ADC convertion and status flag up at end of convertion
		*/
		adc_loop = 0;
		do
		{
			adc_loop++;
			adc_count = 0;

			ADC_SetChannelConfig( SENSORS_ADC_BASE, SENSORS_ADC_CHANNEL_GROUP, &adcChannelConfigStruct );

			while (   ( 0U == ADC_GetChannelStatusFlags( SENSORS_ADC_BASE, SENSORS_ADC_CHANNEL_GROUP ) ) 
				   && (adc_count < ADC_COUNT_MAX)
				  )
			{
				usleep(1);
				adc_count++;
			}
		}
		while (   (adc_count == ADC_COUNT_MAX)
			   && (adc_loop < ADC_LOOP_MAX)
			  );
		
		/* pc:  i'm not sure it's secure to read if convertion not completed!!!! */
		adcVal = ADC_GetChannelConversionValue(SENSORS_ADC_BASE, SENSORS_ADC_CHANNEL_GROUP);
		if (adc_count == ADC_COUNT_MAX)
		{
			PRINTF("[%s timeout: %d]\r\n", 
					(whichAdc == ADC_GAP)?"ADC_GAP":
					(whichAdc == ADC_TEMP)?"ADC_TEMP":
					(whichAdc == ADC_TK_UP)?"ADC_TK_UP":
					(whichAdc == ADC_MEDIA)?"ADC_MEDIA":
					(whichAdc == ADC_24V)?"ADC_24V":
					(whichAdc == ADC_LBL_TAKEN)?"ADC_LBL_TAKEN":"???",
					adcVal
				  );
		}

		/* The RT1024 return ADC values on 12 bits. Record only the 8 higher bits for debug
		   save debug value for gap or take up */
		switch(whichAdc)
		{
			case ADC_GAP:
				gDebugData.gapSense[gDebugIdx] = (unsigned char)(adcVal>>4);	// convert 12 bits to 8 bits
				break;
				
			case ADC_MEDIA:
				gDebugData.mediaSense[gDebugIdx] = (unsigned char)(adcVal>>4);	// convert 12 bits to 8 bits
				break;
#ifdef DEBUG
			case ADC_LBL_TAKEN:
				break;
#endif
		}
	}

	return adcVal;
}

/******************************************************************************/
/*!   \fn bool isLabelTakenSensorEnabled(void)

      \brief
        Check the current activation state of the label taken sensor

      \author
          Nick Barnes 
*******************************************************************************/
bool isLabelTakenSensorEnabled(void)
{
	return sLabelTakenActive;
}

/******************************************************************************/
/*!   \fn void enableLabelTakenSensor(void)

      \brief
        activate the label taken sensor 

      \author
          Nick Barnes 
*******************************************************************************/
void enableLabelTakenSensor(void)
{
	/* for app, we want to switch on the label taken sensor */
	if( !gClamshell ) {
            /* enable the conditioning chip */
            GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, true ); 
	}
	sLabelTakenActive = true;	
}

/******************************************************************************/
/*!   \fn void disableLabelTakenSensor(void)

      \brief
        deactivate the label taken sensor 

      \author
          Nick Barnes 
*******************************************************************************/
void disableLabelTakenSensor(void)
{
	if( !gClamshell ) {
            /* disable the conditioning chip */
            GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, false ); 
	}
	sLabelTakenActive = false;	
}

/******************************************************************************/
/*!   \fn bool checkLabelTaken(void)

      \brief
        just returns true if label is taken else false 

      \author
          Nick Barnes 
*******************************************************************************/
bool checkLabelTaken(void)
{        
	if( !gClamshell && pCfg->labelTakenEnabeld == ON ) {
		static int labelTakenCount = 0;

		/* if sensor is active */
		if( isLabelTakenSensorEnabled() ) {
			/* if sensor threshold is valid (ie we've previously printed a label)... */
			if( gTakenSensorThreshold > TAKEN_SENSOR_THRESHOLD ) {
				/* ...then check for label not taken.
				   WARNING - We do not use the threshold val stored in flash, 
				   but instead still use gTakenSensorThreshold which is calculated
				   on the fly */
				if( readPrinterAdc( ADC_LBL_TAKEN ) < gTakenSensorThreshold ) {   
                    /* sensor indicates label taken
				       need to see LABEL_TAKEN_COUNTS_NEEDED successive label
				       taken ADC readings before flag label taken. this is to
				       mask problem of occassional rogue low ADC reading. */
					if( labelTakenCount < LABEL_TAKEN_COUNTS_NEEDED ) {     /* prevent labelTakenCount rolling over */
						labelTakenCount++;
					
						if( labelTakenCount >= LABEL_TAKEN_COUNTS_NEEDED ) {
							sLabelTakenSensorState = true;
						}
					}
				} else { 
                    /* sensor indicates label not taken */
					labelTakenCount = 0; /* reset label taken count */
					sLabelTakenSensorState = false;
				}
				return sLabelTakenSensorState;
			} else {
				labelTakenCount = 0; /* reset label taken count */
			}
		} else {
			labelTakenCount = 0; /* reset label taken count */

			/* just return what it said last time we read it */
			return sLabelTakenSensorState;
		}
	}	
	return true;
}

/******************************************************************************/
/*!   \fn char convertToTemp(unsigned long adcReading)

      \brief
        convert from thermistor reading to temperature

      \author
          Nick Barnes 
*******************************************************************************/
char convertToTemp( unsigned long adcReading )
{
	/* first entry in tempTbl is at 200mV and the increment is 25mV	*/ 
	unsigned long idx = ( ADC_TO_MVOLTS( adcReading ) - TEMP_TBL_FIRST_ENTRY ) / TEMP_TBL_INCREMENT;

	/* range check */
	if( idx >= sizeof( tempTbl ) ) {
		idx = 0;//this will say its v. hot and fail
	}

	return (char)tempTbl[ idx ];
}
	
/******************************************************************************/
/*!   \fn unsigned long convertTo24vMVolts( unsigned long adcReading )

      \brief
        onvert from voltage adc reading to mVolts

      \author
          Nick Barnes 
*******************************************************************************/
unsigned long convertTo24vMVolts( unsigned long adcReading )
{
	#define STAGE1NUM24VSAMPLES 4 /* initial light filter over 0.5mm of steps */
	#define STAGE2NUM24VSAMPLES 80 /* heavy filter over 10mm worth of motor steps */
	unsigned long v = ADC_TO_MVOLTS(adcReading) * 1082/82 ; /* 1082/82 reverses the reduction in voltage by */
	                                             /* the potential divider of R19 (100K) and R28 (8K2) */        
	/* if v is greater than 20 volts... */
	if( v > 20000 ) {
		/* ...we filter it to avoid speed and strobe len variations caused by 
		   noise on the adc when reading the supply volts */
		
		/* first apply initial light filter */
		v = ( v + ( STAGE1NUM24VSAMPLES -1 ) * s24v ) / STAGE1NUM24VSAMPLES;
	
		/* if in band... */
		if( RANGE( v, s24v -500, s24v +500 ) )
		{
			/* ...slug it with a heavier filter */
			v = ( v + ( STAGE2NUM24VSAMPLES -1 ) * s24v )/ STAGE2NUM24VSAMPLES;
		}		
		s24v = v;
	} else {
		s24v = NOMINAL_MVOLTS;
	}
	
	/* conversion from count to mvolts and then by ratio due to pot divider */
	return v;
}

/******************************************************************************/
/*!   \fn static unsigned char calSensorLedDrive( unsigned char whichLedChan, 
                                                  unsigned char whichAdcChan, 
                                                  unsigned long biasMilliVolts )
      \brief
        set up the current drive to the LED for specified sensor.
        Does binary chop search to find correct bias current quickly.
      \param 
        unsigned char whichLedChan      - channel for the led driver chip
        unsigned char whichAdcChan      - the adc input channel
        unsigned long biasMilliVolts    - what voltage (mV) you want to 
                                          see on the photo transitor reach
      \author
          Nick Barnes 
*******************************************************************************/
static unsigned char calSensorLedDrive( unsigned char whichLedChan, 
                                        unsigned char whichAdcChan, 
                                        unsigned long biasMilliVolts )
{
	unsigned char max = 255;
	unsigned char min = 0;
	unsigned char n = 0;

	/* max and min will converge until we reach a point where we are not getting
	   any closer.  That should be good enough! */
	while(n != (max + min)/2)
	{
		n = (max + min)/2;

		/* set the current through the sensors led  */
		if(write5521LedDriverReg(n, whichLedChan))
		{
			unsigned long val;

			/* give the adc a chance  */
			delay(MILLISECS_TO_TB(5));

			/* read the voltage across the photo transistor  */
			val = readPrinterAdc(whichAdcChan);

			/* nb smaller values of n give bigger vals back.
			   if val is now >= desired voltage, n is at least as small as required,
			   so bring the min up to this.  Else n is still too big, so reduce max
			   down to this. */
			if(val >= MVOLTS_TO_ADC(biasMilliVolts))
			{
				min = n;
			}
			else
			{
				max = n;
			}
		}
		else
		{
			return 0; /* ie dead */
		}
	}
	return n;
}

/******************************************************************************/
/*!   \fn unsigned char calGapSensor(void)

      \brief
        Set up the current drive to the LED for the gap sensor.
        This must be called when there is tally paper in the sensor.  

      \author
          Nick Barnes 
*******************************************************************************/
unsigned char calGapSensor(void)
{
	return calSensorLedDrive(LP5521_GAP_CURRENT, ADC_GAP, 1650);
}

/******************************************************************************/
/*!   \fn unsigned char calMediaSensor(void)

      \brief
        Set up the current drive to the LED for the media sensor
    
      \author
          Nick Barnes 
*******************************************************************************/
unsigned char calMediaSensor(void)
{
	#define MEDIA_SENSOR_BIAS_MVOLTS 2500
	return calSensorLedDrive( LP5521_MEDIA_CURRENT, ADC_MEDIA, MEDIA_SENSOR_BIAS_MVOLTS );
}

/******************************************************************************/
/*!   \fn unsigned char calTakeUpSensor(void)

      \brief
        Set up the current drive to the LED for the take up sensor
        This must be called when take up tensioner is in fully relaxed position

      \author
          Nick Barnes 
*******************************************************************************/
unsigned char calTakeUpSensor(void)
{
	return calSensorLedDrive(LP5521_TK_UP_CURRENT, ADC_TK_UP, 
	                         TK_UP_SENSOR_BIAS_MVOLTS);
}

/******************************************************************************/
/*!   \fn unsigned long calTakeUpSensorSpan(void)

      \brief
        determine the spread of values you get when take up tensioner
        goes from relaxed to fully extended. This must be called when 
        take up tensioner is in fully tensioned position

      \author
          Nick Barnes 
*******************************************************************************/
unsigned long calTakeUpSensorSpan(void)
{
	#define TAKE_UP_SENSOR_MIN_SPAN_MVOLTS 250

	long value = readPrinterAdc(ADC_TK_UP);

	value -= pCfg->gTakeUpRelaxedCounts;

	if(value > MVOLTS_TO_ADC(TAKE_UP_SENSOR_MIN_SPAN_MVOLTS))
	{
		return (unsigned long)value;
	}
	return 0;
}

/******************************************************************************/
/*!   \fn unsigned long getTUSenseDeflection(void)

      \brief
       return the deflection seen in the tu sensor (in 0.001 degrees)
    
      \author
          Nick Barnes 
*******************************************************************************/
unsigned long getTUSenseDeflection(void)
{
	/* crude filter */
	static unsigned long sTuSense = 0;
	unsigned long tuSense = readPrinterAdc(ADC_TK_UP);

#ifdef DEBUGSENSORS
	debugTUSense[ debugIdx ] = ( tuSense / 4 );
	if( ++debugIdx >= DEBUGSAMPLES ) {
		debugIdx = 0;
	}
#endif

	/* safety  */
	if(tuSense < pCfg->gTakeUpRelaxedCounts)
	{
		/* here if read value is less than relaxed (could happen because of noise).  */
		tuSense = 0;
	}
	else
	{
		/* take of the zero offset to give the sense torque value  */
		tuSense -= pCfg->gTakeUpRelaxedCounts;
	}

	sTuSense = (9 * sTuSense + tuSense)/10;
	
	/* uses the square of sTuSense divided by the square of the
	   span (ie the max posible sense val) because this curve 
	   approximates better to the characteristic of the sensor
	   than a straight line would.  When this is multipllied by
	   the max twist (in 0.001 degree units) should give the
	   actual deflection in increments of 0.001 degree */
	return (MAX_TWIST * sTuSense)/pCfg->gTakeupSpan * sTuSense/pCfg->gTakeupSpan;
}

/******************************************************************************/
/*!   \fn bool cassetteDoorClosed(void)

      \brief
       eturn the state of the cassette closed micro switch 
    
      \author
          Nick Barnes 
*******************************************************************************/
bool cassetteDoorClosed(void)
{
	if( !gClamshell ) {
		long n;
		long fitted = 0;
		long notFitted = 0;
	
		for( n = 0; n < 20; n++ )
		{
			//we've got a voting system here!!
			if( !getHeadUp() ) {
				fitted++;
			} else {
				notFitted++;
			}
		}
	
		//return the most common result
		return( fitted > notFitted );
	}
	return( !getHeadUp() );
}
/******************************************************************************/
/******************************************************************************/
