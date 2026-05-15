#include "averyPrinterTask.h"
#include "averyScaleCfg.h"
#include "averyProtocol.h"
#include "averyPrStatus.h"
#include "averyLabel.h"
#include "averyCutter.h"
#include "threadManager.h"
#include "queueManager.h"
#include "translator.h"
#include "systemTimer.h"
#include "sensors.h"
#include "dvr8818.h"
#include "printHead.h"
#include "printEngine.h"
#include "diagnostics.h"
#include "vendor.h"
#include "fsl_debug_console.h"
#include <stdlib.h>

/* task management */
static bool                     suspend_        = false;
static bool                     hostConnected_  = false;
static TaskHandle_t             pHandle_        = NULL;
static QueueHandle_t            pMsgQHandle_    = NULL;
static uint32_t                 prMsg[MAX_AVERY_P_MSG_LENGTH]   = { 0 };
static uint32_t                 debugIdx        = 0;

static FlashPrCFG_t             flashCfg;
static PRCfg_t                  aConfig_;
static PrintStatus_t            gPrinterStatus;
static ExtraStatus_t            gExtraPrinterStatus;
static AvPrEngine_t             *pEng           = NULL;
static DebugData_t              debugData;

static bool clamShell;
static bool cutter;
static bool continuousPrint;
static bool continuousBackwind;

unsigned char sGreyBuf[ GREY_BUF_SIZE ];

uint32_t gSetSpeed = DEFAULT_SPEED;
bool linerLessLabels = false;
uint8_t gCutterSpeed = DEFAULT_CUTTER_SPEED;
uint8_t gCutDistance = DEFAULT_CUT_DISTANCE;
uint8_t gLabelCutDistance = DEFAULT_CUT_DISTANCE;
uint8_t gInitialCutDistance = DEFAULT_CUT_DISTANCE;
uint8_t gPaperCutterState = PAPER_CUTTER_IDLE;

extern char *printer_version_string;
extern char *boot_version_string;



const DIAG_DESC diagDescTbl[] = {
		// type	no.				name					field
		4,	1,				(char *)"setSpeed",			DIAG_SETSPEED,
		1,	DIAGLEN,		        (char *)"mmSpeed",			DIAG_MMSPEED,
		1,	DIAGLEN,		        (char *)"tuSpeed",			DIAG_TUSPEED,

		4,	1,				(char *)"labelLength",			DIAG_LABELLENGTH,		//(steps)
		4,	1,				(char *)"labelGapLength",		DIAG_LABELGAPLENGTH,	        //(steps)
		4,	1,				(char *)"setGapLength",	 		DIAG_GAPLENGTH,			//(steps)
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

		4,	1,				(char *)"tuDiameter",			DIAG_TUDIAMETER,		//(mm)
		4,	1,				(char *)"tuBiasUAmps",	 		DIAG_TUBIASUAMPS,		//(uAmps)
		4,	1,				(char *)"tuBiasMVolts",		 	DIAG_TUBIASMVOLTS,		//(mVolts)
		4,	1,				(char *)"tuSpan",				DIAG_TUSPAN,			//(raw adc)

		4,	1,				(char *)"lblTknSense",			DIAG_LBLTKNSENSE,		//(raw adc)
		4,	1,				(char *)"lblTknThreshold",		DIAG_LBLTKNTHRESHOLD,	        //(raw adc)
		4,	1,				(char *)"lblTknMode",			DIAG_LBLTKNMODE,		// ON or OFF

		4,	1,				(char *)"mediaSenseMin",		DIAG_MEDIASENSEMIN,
		4,	1,				(char *)"mediaSenseMid",		DIAG_MEDIASENSEMID,
		4,	1,				(char *)"mediaSenseMax",		DIAG_MEDIASENSEMAX,
		4,	1,				(char *)"mediaSenseBias",		DIAG_MEDIASENSEBIAS,	        //(uAmps)
		1,	DIAGLEN,		        (char *)"mediaSense",			DIAG_MEDIASENSE,
		4,	1,				(char *)"mediaDiameter",		DIAG_MEDIADIAMETER,
		4,	1,				(char *)"mediaAveLittleDips",	        DIAG_MEDIAAVELITTLEDIPS,
		4,	1,				(char *)"mediaAveLargeDips",	        DIAG_MEDIAAVELARGEDIPS,

		1,	PRINTER_HEAD_SIZE_56MM,         (char *)"pheadDots",		        DIAG_PHEADDOTS, 	        //(in 10 ohm increments)

		1,	1,				(char *)"pcbRevision",			DIAG_PCBREV,

		1,	NUM_CUTTER_SPEED_PROFILE_MARKS, (char *)"cutterSpeedProfile",           DIAG_CUTTERSPEEDPROFILE,
};

/* private functions */
static void handleAppVersionMsg( void );
static void handleAppDateMsg( void );
static void handleGetModeMsg( void );
static void handleBootVersionMsg( void );
static void handleBootDateMsg( void );
static void handleGetModeMsg( void );
static void handlePrintDensity( unsigned char density );
static void handleGetLabelLenMsg( void );
static void handleSetLabelGapMsg( unsigned char *pGapLen );
static void handleCalGapSensorMsg( void );
static void handleCalMediaSensorMsg( void );
static void handleCalMediaSensorMsg( void );
static void handleSetLabelEdgeMsg( unsigned char *pMinEdge );
static void handleSetPrinterIDMsg( unsigned char id );
static void handleSetPaperThresholdMsg( unsigned char threshold );
static void handleGetTakenThresholdMsg( void );
static void handleSetTakenThresholdMsg( unsigned char val );
static void handleSetTakenModeMsg( bool enable );
static void copyConfigurationFlash( PRCfg_t *pCfg, FlashPrCFG_t *pFlash );
static void copyConfiguration( FlashPrCFG_t *pFlash, PRCfg_t *pCfg );

/******************************************************************************/
/*!   \fn BaseType_t createAveryPrinterTask( QueueHandle_t msgQueue )                                                           
      \brief
        This function initializes printer resources and creates 
        a global printer task. 
       
      \author
          Aaron Swift
*******************************************************************************/ 
BaseType_t createAveryPrinterTask( QueueHandle_t msgQueue )
{
    BaseType_t result = pdFAIL;
    
    /* set our message queue */
    pMsgQHandle_ = msgQueue;

    /* initialize our config, status and extra status */
    memset( (void *)&aConfig_, 0, sizeof(PRCfg_t) );

    memset( (void *)&gPrinterStatus, 0, sizeof( PrintStatus_t ) );
    gPrinterStatus.resetOccurred = true;

    memset( (void *)&gExtraPrinterStatus, 0, sizeof( ExtraStatus_t ) );
    gExtraPrinterStatus.maxDotRes = NOMINAL_PHEAD_RESISTANCE;
    gExtraPrinterStatus.cassettePrintheadWidth[0] = PRINTER_HEAD_SIZE_56MM / 0x100;
    gExtraPrinterStatus.cassettePrintheadWidth[1] = PRINTER_HEAD_SIZE_56MM % 0x100;
    gExtraPrinterStatus.cassettePrintheadDotsPerMm = 8;
    
    /* intialize our printer state */
    initPrinterState();
    /* intialize our debug data */
    memset( (void *)&debugData, 0, sizeof( DebugData_t ) );
    debugIdx = 0;
    
    /* open flash interface to load configuration */
    if( openInterface() ) {
        PRINTF("printerTask(): opening flash interface.\r\n" );        
        if( !readAPrConfigFromFlash( &flashCfg ) ) {
            PRINTF("printerTask(): Flash blank loading defaults.\r\n" );
            /* configuration is blank, load defaults */
            setAPrConfigDefaults( &flashCfg );
            if( writeAPrConfigToFlash( &flashCfg ) ) {
                PRINTF("printerTask(): Configuration defaults saved!\r\n" );                
            } else {
                PRINTF("printerTask(): Critical error saving defaults failed!\r\n" );                
            }                                          
        } 
    } else {
        /* failed to open interface, just run with defaults */
        PRINTF("printerTask(): Failed to open flash interface.\r\n" );
        setAPrConfigDefaults( &flashCfg );
    }
    closeInterface(); 
    copyConfiguration( &flashCfg, &aConfig_ );
    showAPrConfiguration( &flashCfg );     
    PRINTF("printerTask(): closing flash interface.\r\n" );         
    
    
    /* create printer task thread */
    result = xTaskCreate( averyPrinterTask,  "PrinterTask", (5 * 1024),
                                        NULL, printer_task_PRIORITY, &pHandle_ );
    if( pHandle_ != NULL ) {
        /* register our handle with the thread manager */
        updateTaskHandle( T_PRINTER );   
        /* initialize our printer resoureces */
        initPrinter( );
    } 
    return result;
}

PRCfg_t *getAveryPrinterCfg( void ) { return &aConfig_; }
bool isClamShell( void ) { return clamShell; }
bool isContinuousPrint( void ) { return continuousPrint; }

/******************************************************************************/
/*!   \fn static void initPrinter( void )                                                          

      \brief This function initializes the printer resources.

      \author
          Aaron Swift
*******************************************************************************/ 
static void initPrinter( void )
{
    clamShell = false;
    cutter = false;
    continuousPrint = false;
    continuousBackwind = false;
    /* get print engine */
    pEng = getPrintEngine( _HOBART_PRINTER );
    /* initialize our print head */
    initPrintHead( DT_AV_SERVICE_56MM );
    /* make sure power to the head is off */
    headPower( false );
  
    if( !initPrinterEngine( _HOBART_PRINTER ) ) {
        PRINTF("initPrinter(): failed to init print engine!\r\n" );    
    }
    if( aConfig_.labelGapLength != 0 ) {
        registerLabelGapLengthConfig( aConfig_.labelGapLength );
    }
    initStepper( _AVERY_STEPPER );
}

/******************************************************************************/
/*!   \fn static void averyPrinterTask( void *pvParameters )                                                         
 
      \brief
        This function is the task thread for the printer. The task waits for 
        a select from the queue set and evaluates and processes.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void averyPrinterTask( void *pvParameters )
{
    ( void ) pvParameters;
    
    PRINTF("printerTask(): Thread running...\r\n" );              
    
    while( !suspend_ ) {
        /* wait for host message */        
        if( xQueueReceive( pMsgQHandle_, &prMsg[0], portMAX_DELAY ) ) {            
            handleAveryPrinterMsg( &prMsg[0] );                
        } else {
            PRINTF("printerTask(): Failed to get Printer message from queue!\r\n" );              
        } 
        taskYIELD();
    }
    vTaskSuspend(NULL);   
}

/******************************************************************************/
/*!   \fn TaskHandle_t getAveryPrinterHandle( void )
 
      \brief
        This function returns the printer task handle.
       
      \author
          Aaron Swift
*******************************************************************************/ 
TaskHandle_t getAveryPrinterHandle( void )
{
    return pHandle_;
}

/******************************************************************************/
/*!   \fn static void handleAveryPrinterMsg( uint32_t *pMsg )                                                         
 
      \brief
        This function handles printer messages from the host.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleAveryPrinterMsg( uint32_t *pMsg )
{
    bool result = false;    
    switch( ((Packet_t *)pMsg)->cmd  ) 
    {
        case AV_GETPRINTERSTATUSCMD: {
            sendAVPrinterMsg( (unsigned char *)&gPrinterStatus, sizeof(PrintStatus_t) );
            break;
        }
        case AV_GETEXTRAPRINTERSTATUSCMD: {
            sendAVPrinterMsg( (unsigned char *)&gExtraPrinterStatus, sizeof(ExtraStatus_t) );          
            break;
        }
        case AV_GETAPPFIRMWARENUM: {
            handleAppVersionMsg();
            break;
        }
        case AV_GETBOOTFIRMWARENUM: {
            handleBootVersionMsg();
            break;
        }
        case AV_GETAPPFIRMWAREDATE: {
            handleAppDateMsg();
            break;
        }
        case AV_GETBOOTFIRMWAREDATE: {
            handleBootDateMsg();
            break;
        }
        case AV_GETMODE: {
            handleGetModeMsg();
            break;
        }        
        case AV_CHANGEMODE: {
            gPrinterStatus.resetOccurred = false;
            if( !clamShell ) {
                /* are we changing modes? */
                if( ((Packet_t *)pMsg)->data[0] != continuousPrint ) {
                    continuousPrint = ((Packet_t *)pMsg)->data[0];
                    /* TO DO: 
                    initPosnGlobals( newMode ); */
                }

                if( ((Packet_t *)pMsg)->data[0] == PPM_CONTINUOUS_NO_BACKWIND ) {
                    continuousBackwind = false;
                } else {
                    continuousBackwind = true;
                }
                
                result = true;
                sendAVPrinterMsg( (unsigned char *)&result, 1 );
            }
            break;
        }
        case AV_GETPRINTDENSITY: {
            sendAVPrinterMsg( &aConfig_.labelPrintDensity, 1 );            
            break;
        }        
        case AV_ADJUSTPRINTDENSITY: {
            handlePrintDensity( ((Packet_t *)pMsg)->data[0]  );          
            break;
        }
        case AV_GETPRINTERDEBUG: {
            break;
        }        
        case AV_PRINTLABEL: 
        case AV_PRINTLABELNOPARK: {
            break;
        }
        case AV_FORMFEED: {
            break;
        }
        case AV_GETLABELLENGTH: {
            handleGetLabelLenMsg();
            break;
        }
        case AV_SETLABELLENGTH: {
            if( pEng != NULL ) {
                pEng->labelLen = MM_TO_DOTS( charToLong( &((Packet_t *)pMsg)->data[0] ) );
                PRINTF( "handleAveryPrinterMsg() SETLABELLENGTH: %d\r\n", pEng->labelLen );
                
                result = true;
            } 
            sendAVPrinterMsg( (unsigned char *)&result, 1 );            
            break;
        }
        case AV_GETLABELGAPLENGTH: {
            if( pEng != NULL ) {
                pEng->labelGapLength = aConfig_.labelGapLength;                                
                unsigned long val = DOTS_TO_MM( pEng->labelGapLength );
                PRINTF( "handleAveryPrinterMsg() GETLABELGAPLENGTH: %d\r\n", val );
                sendAVPrinterMsg( (unsigned char *)&val, sizeof(unsigned long) );   
            } else {
                PRINTF( "handleAveryPrinterMsg() GETLABELGAPLENGTH: Engine is null!\r\n" );
            }
            break;
        }        
        case AV_SETLABELGAPLENGTH: {
            handleSetLabelGapMsg( &((Packet_t *)pMsg)->data[0] );
            break;
        }
        case AV_GETGAPSENSORBIAS: {
            sendAVPrinterMsg( &aConfig_.gapCalVal, 1 );
            break;
        }        
        case AV_CALGAPSENSORBIAS: {
            handleCalGapSensorMsg();
            break;
        }
        case AV_SETGAPSENSORBIAS: {
            if( ((Packet_t *)pMsg)->data[0] != aConfig_.gapCalVal ) {
                aConfig_.gapCalVal = ((Packet_t *)pMsg)->data[0];
                /* TO DO: adjust sensor bias 
                */
                copyConfigurationFlash( &aConfig_, &flashCfg );
                /* save to flash */
                writeAPrConfigToFlash( &flashCfg );                
                result = true;
            }
            sendAVPrinterMsg( (unsigned char *)&result, 1 ); 
            break;
        }
        case AV_GETMEDIASENSORBIAS: {
            sendAVPrinterMsg( &aConfig_.mediaSensorCalVal, 1 );
            break;
        }        
        case AV_CALMEDIASENSORBIAS: {
            handleCalMediaSensorMsg();
            break;
        }
        case AV_SETMEDIASENSORBIAS: {
            if( ((Packet_t *)pMsg)->data[0] != aConfig_.mediaSensorCalVal ) {
                aConfig_.mediaSensorCalVal = ((Packet_t *)pMsg)->data[0];
                /* TO DO: adjust sensor bias 
                */     
                copyConfigurationFlash( &aConfig_, &flashCfg );
                /* save to flash */
                writeAPrConfigToFlash( &flashCfg );
                result = true;                
            }
            sendAVPrinterMsg( (unsigned char *)&result, 1 );           
            break;
        }
        case AV_SETPRINTERID: {
            handleSetPrinterIDMsg( ((Packet_t *)pMsg)->data[0] );
            break;
        }
        case AV_GETPRINTERID: {
            sendAVPrinterMsg( &aConfig_.gPrinterId, 1 );
            break;
        }
        case AV_SETPAPEROUTTHRESHOLD: {
            handleSetPaperThresholdMsg( ((Packet_t *)pMsg)->data[0] );
            break;
        }
        case AV_GETPAPEROUTTHRESHOLD: {
            sendAVPrinterMsg( &aConfig_.paperDetectVal, 1 );   
            break;
        }
        case AV_GETMINLABELEDGEDIFF: {
            sendAVPrinterMsg( &aConfig_.minLabelEdgeDiff, 1 );    
            break;
        }
        case AV_SETMINLABELEDGEDIFF: {
            handleSetLabelEdgeMsg(  &((Packet_t *)pMsg)->data[0]  );
            break;
        }
        case AV_SETPRINTSPEED: {
            if( pEng != NULL ) {
                pEng->printSpeed = MM_TO_DOTS( charToLong( &((Packet_t *)pMsg)->data[0] ) ); 
                PRINTF( "handleAveryPrinterMsg() set print speed: %d mm/s\r\n", pEng->printSpeed );
                result = true;
            }   
            sendAVPrinterMsg( (unsigned char *)&result, 1 ); 
            break;
        }  
        case AV_FORCERESET: {
            if( charToLong( (unsigned char *)&((Packet_t *)pMsg)->ptReset.check ) == 0x12345678 ) {
                PRINTF( "handleAveryPrinterMsg() force reset A\r\n" );
                result = true;
            } else if( charToLong( (unsigned char *)&((Packet_t *)pMsg)->ptReset.check ) == 0x12345678 ) {
                PRINTF( "handleAveryPrinterMsg() force reset B\r\n" );
                result = true;
            } else {
                PRINTF( "handleAveryPrinterMsg() force reset ?\r\n" );
            }
            sendAVPrinterMsg( (unsigned char *)&result, 1 );
            break;
        }
        case AV_PROGRAM: {
            break;
        }
        case AV_GETTAKENSENSORTHRESHOLD: {
            handleGetTakenThresholdMsg();           
            break;
        }
        case AV_SETTAKENSENSORTHRESHOLD: {
            handleSetTakenThresholdMsg( ((Packet_t *)pMsg)->data[0] );         
            break;
        }
        case AV_SETTAKENSENSORMODE: {
            handleSetTakenModeMsg( (bool)((Packet_t *)pMsg)->data[0] );
            break;
        }
        case AV_GETTAKENSENSORMODE: {
            sendAVPrinterMsg( (unsigned char *)&aConfig_.labelTakenEnabeld, 1 );
            break;
        }
        case AV_GETCUTTERFIRMWARENUM: 
        case AV_GETCUTTERFIRMWAREDATE: 
        case AV_SETPAPERCUTTERCONFIG:
        case AV_INITIALPAPERCUT:
        case AV_GETPAPERCUTTERSTATUSCMD: 
        case AV_TESTPAPERCUT: 
        case AV_PAPERCUTTERINTERFACERESET:  
        case AV_PROGRAMWITHPROCESSORTYPE:
        case AV_CALTAKEUPSENSORBIAS:
        case AV_SETTAKEUPSENSORBIAS:
        case AV_CALTAKEUPSENSORSPAN:
        case AV_SETTAKEUPSENSORSPAN:
        case AV_CALTAKENSENSORTHRESHOLD:
        case AV_GETTAKEUPSENSORBIAS:
        case AV_GETTAKEUPSENSORSPAN:
        case AV_GETPCBREV:
        case AV_SETLINERLESSLABELPRINTER:
        case AV_OPENCASHDRAWER: 
        case AV_SETLOGOLEDMODE:           
        case AV_SETSTROBELEN: {
            PRINTF( "handleAveryPrinterMsg() Unsupported msg %d!\r\n", ((Packet_t *)pMsg)->cmd );
            break;
        }
        
        default: {
            PRINTF("handleAveryPrinterMsg():Unknown message type %d\r\n", ((Packet_t *)pMsg)->cmd);   
            break;
        }      
    }
}

/******************************************************************************/
/*!   \fn static void copyConfigurationFlash( PRCfg_t *pCfg, FlashPrCFG_t *pFlash )                                             
 
      \brief
        This function copies the RAM version of configuration to flash.
       
      \author
          Aaron Swift
*******************************************************************************/
static void copyConfigurationFlash( PRCfg_t *pCfg, FlashPrCFG_t *pFlash )
{
    pFlash->sensorToPheadDistance =     pCfg->sensorToPheadDistance;
    pFlash->printerId           =       pCfg->gPrinterId;
 
    pFlash->labelPrintDensity   =       pCfg->labelPrintDensity;
    pFlash->labelEdgeVal        =       pCfg->minLabelEdgeDiff;
    pFlash->paperDetectVal      =       pCfg->paperDetectVal;
    pFlash->gapSensorBackingVal =       pCfg->gapSensorBackingVal;
    pFlash->gapSensorLabelVal   =       pCfg->gapSensorLabelVal;
    pFlash->receiptPrintDensity =       pCfg->receiptPrintDensity;
    pFlash->takenSensorMode     =       pCfg->labelTakenEnabeld;

    pFlash->gapCal              =       pCfg->gapCalVal;
    pFlash->labelTkCal          =       pCfg->labelCalVal;
    pFlash->labelEdgeMinDiff    =       pCfg->minLabelEdgeDiff;
    pFlash->mediaSensorCalVal   =       pCfg->mediaSensorCalVal;  
}

/******************************************************************************/
/*!   \fn static void copyConfiguration( flashCfg *pFlash, PRCfg_t *pCfg )                                              
 
      \brief
        This function copies the flash version of configuration to RAM.
       
      \author
          Aaron Swift
*******************************************************************************/
static void copyConfiguration( FlashPrCFG_t *pFlash, PRCfg_t *pCfg )
{
    pCfg->sensorToPheadDistance =       pFlash->sensorToPheadDistance;
    pCfg->gPrinterId            =       pFlash->printerId;
 
    pCfg->labelPrintDensity     =       pFlash->labelPrintDensity;
    pCfg->minLabelEdgeDiff      =       pFlash->labelEdgeVal;
    pCfg->paperDetectVal        =       pFlash->paperDetectVal;
    pCfg->gapSensorBackingVal   =       pFlash->gapSensorBackingVal;
    pCfg->gapSensorLabelVal     =       pFlash->gapSensorLabelVal;
    pCfg->receiptPrintDensity   =       pFlash->receiptPrintDensity;
    pCfg->labelTakenEnabeld     =       pFlash->takenSensorMode;

    pCfg->gapCalVal             =       pFlash->gapCal;
    pCfg->labelCalVal           =       pFlash->labelTkCal;
    pCfg->minLabelEdgeDiff      =       pFlash->labelEdgeMinDiff;
    pCfg->mediaSensorCalVal     =       pFlash->mediaSensorCalVal;  
}

/******************************************************************************/
/*!   \fn static void handleAppVersionMsg( void )                                                   
 
      \brief
        This function handles returning application firmware version to the 
        host.
       
      \author
          Aaron Swift
*******************************************************************************/
static void handleAppVersionMsg( void )
{
	const char inAppStr[] = " *";
        unsigned char txBuf[ MAX_PACKET_SIZE ];        
	version_t v = get_version( printer_version_string );
        
	strncpy( (char*)&txBuf[0], v.firmware, MAX_PACKET_SIZE - strlen(inAppStr) -1 );
	strcat( (char*)&txBuf[0], inAppStr );
	txBuf[MAX_PACKET_SIZE -1] = 0;
        
        sendAVPrinterMsg( &txBuf[0], strlen((char*)txBuf) +1 );
}

/******************************************************************************/
/*!   \fn static void handleAppDateMsg( void )                                                   
 
      \brief
        This function handles returning application firmware date version to the 
        host.
       
      \author
          Aaron Swift
*******************************************************************************/
static void handleAppDateMsg( void )
{
	const char inAppStr[] = " *";
        unsigned char txBuf[ MAX_PACKET_SIZE ];        
	version_t v = get_version( printer_version_string );
        
	strncpy( (char*)&txBuf[0], v.date, MAX_PACKET_SIZE - strlen(inAppStr) -1 );
	strncat( (char*)&txBuf[0], " ", MAX_PACKET_SIZE -1 );
	strncat( (char*)&txBuf[0], v.time, MAX_PACKET_SIZE -1 );
	txBuf[MAX_PACKET_SIZE -1] = 0;
        
        sendAVPrinterMsg( &txBuf[0], strlen((char*)txBuf) +1 );	  
}

/******************************************************************************/
/*!   \fn static void handleBootDateMsg( void )                                                    
 
      \brief
        This function handles returning bootloader firmware version to the 
        host.
       
      \author
          Aaron Swift
*******************************************************************************/
static void handleBootVersionMsg( void )
{
        unsigned char txBuf[ MAX_PACKET_SIZE ];        
	version_t v = get_version( &boot_version_string );
        
	strncpy( (char*)&txBuf[0], v.firmware, MAX_PACKET_SIZE -1 );
	strncat( (char*)&txBuf[0], " ", MAX_PACKET_SIZE -1 );
	strncat( (char*)&txBuf[0], v.time, MAX_PACKET_SIZE -1 );
	txBuf[MAX_PACKET_SIZE -1] = 0;
        
        sendAVPrinterMsg( &txBuf[0], strlen((char*)txBuf) +1 );	  
}

/******************************************************************************/
/*!   \fn static void handleBootDateMsg( void )                                                    
 
      \brief
        This function handles returning bootloader firmware version date to the 
        host.
       
      \author
          Aaron Swift
*******************************************************************************/
static void handleBootDateMsg( void )
{
        unsigned char txBuf[ MAX_PACKET_SIZE ];        
	version_t v = get_version( &boot_version_string );
        
	strncpy( (char*)&txBuf[0], v.date, MAX_PACKET_SIZE -1 );
	strncat( (char*)&txBuf[0], " ", MAX_PACKET_SIZE -1 );
	strncat( (char*)&txBuf[0], v.time, MAX_PACKET_SIZE -1 );
	txBuf[MAX_PACKET_SIZE -1] = 0;
        
        sendAVPrinterMsg( &txBuf[0], strlen((char*)txBuf) +1 );	    
}

/******************************************************************************/
/*!   \fn static void handleGetModeMsg( void )                                                    
 
      \brief
        This function handles returning the current printer mode to the host.
       
      \author
          Aaron Swift
*******************************************************************************/
static void handleGetModeMsg( void )
{
    unsigned char txBuf[ MAX_PACKET_SIZE ];    
    if( continuousPrint ) {
        if( continuousBackwind ) {
            txBuf[0] = PPM_CONTINUOUS_BACKWIND;
        } else {
            txBuf[0] = PPM_CONTINUOUS_NO_BACKWIND;
        }
    } else {
        txBuf[0] = PPM_SEPARATE;
    }
    PRINTF( "handleAveryPrinterMsg() GETMODE: %d\r\n", txBuf[0]);
    sendAVPrinterMsg( &txBuf[0], 1 );            
}

/******************************************************************************/
/*!   \fn static void handleGetLabelLenMsg( void )                                                    
 
      \brief
        This function handles setting the printer print density ( contrast ) and
        saving configuration to flash.
       
      \author
          Aaron Swift
*******************************************************************************/
static void handlePrintDensity( unsigned char density )
{
    if( aConfig_.labelPrintDensity != density ) {
        aConfig_.labelPrintDensity = density;
        
        copyConfigurationFlash( &aConfig_, &flashCfg );
        
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                        
    }    
}

/******************************************************************************/
/*!   \fn static void handleGetLabelLenMsg( void )                                                    
 
      \brief
        This function handles sending the label in mm to host.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleGetLabelLenMsg( void )
{
    unsigned long len = 0;
    if( pEng != NULL ) {
        len = DOTS_TO_MM( pEng->labelLen ); 
        PRINTF( "handleAveryPrinterMsg() GETLABELLENGTH: %d\r\n", len );
    }
    sendAVPrinterMsg( (unsigned char *)&len, sizeof(unsigned long) );
}

/******************************************************************************/
/*!   \fn static void handleSetLabelGapMsg( unsigned char *pGapLen )                                                         
 
      \brief
        This function handles setting the label gap length. This is used to 
        specify the size of the gap between labels, in mm, typically where the 
        actually gap is different to what the printer will be able to measure 
        - eg labels with a dome shaped trailing edge. Labels with funny shaped 
        leading edge should work happily without any special value.
      \note A value of zero will indicate that the printer should operate in the 
            normal way ie it will measure the gap and act accordingly.
 
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleSetLabelGapMsg( unsigned char *pGapLen )
{
    bool result = true;
    uint32_t gapLength = MM_TO_DOTS( nCharToLong( pGapLen ) );
    if( aConfig_.labelGapLength != gapLength ) {
        aConfig_.labelGapLength = gapLength;  

        copyConfigurationFlash( &aConfig_, &flashCfg );
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                
        
        /* if not continuous paper */
        if( continuousPrint == false ) {
            /* TO DO: 
            initPosnGlobals( newMode ); */          
        }
    }
    sendAVPrinterMsg( (unsigned char *)&result, 1 );
}

/******************************************************************************/
/*!   \fn static void handleCalGapSensorMsg( void )                                                    
 
      \brief
        This function handles host message to calibrate the gap sensor and sends
        result of calibration to the host.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleCalGapSensorMsg( void )
{
    unsigned char cVal = 0;
    /* TO DO: 
        cVal = calGapSensor();
    */
    
    /* zero means dead sensor */
    if( cVal && cVal != aConfig_.gapCalVal ) {
        aConfig_.gapCalVal = cVal;
        PRINTF( "handleAveryPrinterMsg() CALGAPSENSORBIAS: save gapCalVal %d\r\n", aConfig_.gapCalVal );  
        copyConfigurationFlash( &aConfig_, &flashCfg );
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                
    }
    sendAVPrinterMsg( &cVal, 1 );    
}

/******************************************************************************/
/*!   \fn static void handleCalMediaSensorMsg( void )                                                      
 
      \brief
        This function handles host message to calibrate the media sensor and 
        sends result of calibration to the host.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleCalMediaSensorMsg( void )
{
    unsigned char cVal = 0;
    /* TO DO: 
        cVal = calMediaSensor();
    */
    /* zero means dead sensor */
    if( cVal && cVal != aConfig_.mediaSensorCalVal ) {
        aConfig_.mediaSensorCalVal = cVal;
        PRINTF( "handleAveryPrinterMsg() CALMEDIASENSORBIAS: save mediaSensorCalVal %d\r\n", aConfig_.mediaSensorCalVal ); 
        copyConfigurationFlash( &aConfig_, &flashCfg );
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                        
    }
    sendAVPrinterMsg( &cVal, 1 );    
}

/******************************************************************************/
/*!   \fn static void handleSetLabelEdgeMsg( unsigned char *pMinEdge )                                                       
 
      \brief
        This function handles host message to set the minimum label edge and 
        save configuration to flash.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleSetLabelEdgeMsg( unsigned char *pMinEdge )
{
    bool result = false;
    /* make sure value is in range before we set */
    if( RANGE( *pMinEdge, MIN_MIN_LABEL_EDGE_DIFF, MAX_MIN_LABEL_EDGE_DIFF ) ) {
        if( *pMinEdge  != aConfig_.minLabelEdgeDiff ) {
            aConfig_.minLabelEdgeDiff = *pMinEdge;
            PRINTF( "handleAveryPrinterMsg() SETMINEDGE: save minLabelEdgeDiff %d\r\n", aConfig_.minLabelEdgeDiff );  
            copyConfigurationFlash( &aConfig_, &flashCfg );
            /* save to flash */
            writeAPrConfigToFlash( &flashCfg );                
        }
        result = true;
    }
    sendAVPrinterMsg( (unsigned char *)&result, 1 );  
}

/******************************************************************************/
/*!   \fn static void handleSetPrinterIDMsg( unsigned char id )                                                      
 
      \brief
        This function handles host message to set printer id and  save 
        configuration to flash.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleSetPrinterIDMsg( unsigned char id )
{
    bool result = false;
    /* 255 is an invalid id value */
    if( id != 255 ) {
        aConfig_.gPrinterId = id;
        PRINTF( "handleAveryPrinterMsg() SETPRNITERID: save gPrinterId %d\r\n", aConfig_.gPrinterId );  
        copyConfigurationFlash( &aConfig_, &flashCfg );
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                
        result = true;
    }
    sendAVPrinterMsg( (unsigned char *)&result, 1 );  
}

/******************************************************************************/
/*!   \fn static void handleSetPrinterIDMsg( unsigned char id )                                                      
 
      \brief
        This function handles host message to set paper threshold and  save 
        configuration to flash.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleSetPaperThresholdMsg( unsigned char threshold )
{
#if 0   /* TO DO: complete */  
    bool result = false;
    /* make sure value is in range before setting and saving */
    if( threshold < MAX_PAPER_THRESHOLD ) {
        if( threshold != aConfig_.paperDetectVal ) {
            aConfig_.paperDetectVal = threshold;
            PRINTF( "handleAveryPrinterMsg() SETTHRESHOLD: save paperDetectVal %d\r\n", aConfig_.paperDetectVal ); 
            copyConfigurationFlash( &aConfig_, &flashCfg );
            /* save to flash */
            writeAPrConfigToFlash( &flashCfg );                
        }
        result = true;
    }
    sendAVPrinterMsg( (unsigned char *)&result, 1 );  
#endif    
}

/******************************************************************************/
/*!   \fn static void handleGetTakenThresholdMsg( void )                                                      
 
      \brief
        This function handles host message to get label taken sensor threshold. 
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleGetTakenThresholdMsg( void )
{
    unsigned char val = aConfig_.labelCalVal / 4;
    sendAVPrinterMsg( &val, 1 );   
}

/******************************************************************************/
/*!   \fn static handleSetTakenThresholdMsg( unsigned char val )                                                   
 
      \brief
        This function handles host message to set label taken sensor threshold
        and save configuration to flash. 
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleSetTakenThresholdMsg( unsigned char val )
{
    bool result = false;
    val = val * 4;
    if( aConfig_.labelCalVal != val ) {
        aConfig_.labelCalVal = val;
        PRINTF( "handleAveryPrinterMsg() SETTAKENTHRESHOLD: save labelCalVal %d\r\n", aConfig_.labelCalVal );  
        copyConfigurationFlash( &aConfig_, &flashCfg );
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                
        result = true;  
    }
    sendAVPrinterMsg( (unsigned char *)&result, 1 );    
}

static void handleSetTakenModeMsg( bool enable )
{
    bool result = false;
    /* enable or disable the sensor */
    if( aConfig_.labelTakenEnabeld != enable ) {
        aConfig_.labelTakenEnabeld = enable;
        PRINTF( "handleAveryPrinterMsg() SETTAKENMODE: save takenSensorMode %d\r\n", aConfig_.labelTakenEnabeld );  
        copyConfigurationFlash( &aConfig_, &flashCfg );
        /* save to flash */
        writeAPrConfigToFlash( &flashCfg );                
        result = true;          
    }
    sendAVPrinterMsg( (unsigned char *)&result, 1 );    
}
