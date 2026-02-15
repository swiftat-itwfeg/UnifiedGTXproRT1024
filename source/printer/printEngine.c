#include <stdlib.h>
#include <limits.h>
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"
#include "fsl_common_arm.h"
#include "fsl_dmamux.h"
#include "fsl_clock.h"
#include "fsl_gpt.h"
#include "fsl_pwm.h"
#include "pin_mux.h"
#include "sensors.h"
#include "label.h"
#include "dvr8818.h"
#include "serialFlash.h"
#include "w25x10cl.h"
#include "systemTimer.h"
#include "printEngine.h"
#include "printhead.h"
#include "commandTable.h"
#include "drawingPrimitives.h"
#include "threadManager.h"
#include "queueManager.h"
#include "prMessages.h"
#include "semphr.h"
#include "averyPrinter.h"
#include "dotWearTask.h"
#include "commandTable.h"
#include "averyCutter.h"
#include "takeupMotor.h"
#include "fsl_debug_console.h"
#include "lp5521.h"
#include "translator.h"
#include "TPHMotor.h"
#include "globalPrinterTask.h"
#include "virtualScope.h"
#include "lp5521.h"
#include "fsl_pit.h"
#include "dotWearTask.h"
#include "fsl_adc_etc.h"





extern const unsigned short rohm80mmSLTTimes[10]; 
extern const unsigned short rohm80mmHistory[8];
extern const unsigned short rohm80mmCurrentLine[8];
extern const unsigned short rohm80mmPwmStart[8];
extern const unsigned short rohm80mmPwmDuty[8];

AT_NONCACHEABLE_SECTION_INIT(int tempAtStart) = 0;

AT_NONCACHEABLE_SECTION_INIT(uint16_t labelPauseTimeout) = 0;
AT_NONCACHEABLE_SECTION_INIT(uint16_t takeupBusyTimeout) = 0;

AT_NONCACHEABLE_SECTION_INIT(bool firstPrint) = true;

AT_NONCACHEABLE_SECTION_INIT(bool startOfQueue) = false;
AT_NONCACHEABLE_SECTION_INIT(uint16_t streamingLeadInMod) = 0;
AT_NONCACHEABLE_SECTION_INIT(uint16_t streamingExpelMod) = 0;

bool canceledSizingFlag = false;
bool OutOfMedia = false;  //TFinkMediaFilter

bool TUSlip = false;
AT_NONCACHEABLE_SECTION_INIT(bool printingStatus) = false;
bool sizingStatus = false;

AT_QUICKACCESS_SECTION_DATA(PrintEngine engine);
AT_NONCACHEABLE_SECTION_INIT(bool historyEnabled_) = true;

AT_QUICKACCESS_SECTION_DATA(short* shootCounts);

AT_NONCACHEABLE_SECTION_INIT(uint16_t prevLineCounter) = 0;
AT_NONCACHEABLE_SECTION_INIT(uint16_t currLineCounter) = 0;
uint32_t LTWaitCount_ = 0;

AT_NONCACHEABLE_SECTION_INIT(int leadInSteps) = 0;
AT_NONCACHEABLE_SECTION_INIT(bool leadInDone) = false;
AT_NONCACHEABLE_SECTION_INIT(int expelSteps) = 0;
AT_NONCACHEABLE_SECTION_INIT(bool expelDone) = false;

static bool stepCounterEnabled_         = false;
static bool skipMissingLabel_           = false;
static bool skipLabelTaken_             = false;
static bool backwindAfterSizing         = false;
static bool backwindAfterSizingDone     = false;

#if 1   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
lpspi_master_edma_handle_t spiHeadMasterHandle;
#else 
AT_QUICKACCESS_SECTION_DATA( lpspi_master_handle_t spiHeadMasterHandle ); 
#endif

AT_QUICKACCESS_SECTION_DATA( LabelPosition   label0 );
AT_QUICKACCESS_SECTION_DATA( LabelPosition   label1 );
AT_QUICKACCESS_SECTION_DATA( LabelPosition   label2 );
AT_QUICKACCESS_SECTION_DATA( LabelPosition   *pCurrentLabel );

AT_NONCACHEABLE_SECTION_INIT( static SemaphoreHandle_t pCutSemaphore ) = NULL;
AT_NONCACHEABLE_SECTION_INIT( static bool applyVerticalOffset_ )       = false;
AT_NONCACHEABLE_SECTION_INIT( static short prevVertOffset_ )           = 0;

/* command queue */
AT_NONCACHEABLE_SECTION_INIT( static QueueHandle_t pCmdQHandler_ )     = NULL;

/* line timer interrupt level for print operation and dotwear */
static unsigned int interruptLevel_ = 1; 

AT_NONCACHEABLE_SECTION_INIT(bool continuousStock_) = false;
AT_NONCACHEABLE_SECTION_INIT(bool mainMotorStopped_ ) = false;

#if 1   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData );
#else 
AT_QUICKACCESS_SECTION_CODE( void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData ) );
#endif

AT_NONCACHEABLE_SECTION_INIT( volatile bool spiDmaTransferComplete ) = false;
AT_QUICKACCESS_SECTION_DATA( edma_handle_t eDmaHandle_0 );
AT_QUICKACCESS_SECTION_DATA( edma_handle_t eDmaHandle_1 );
AT_QUICKACCESS_SECTION_DATA( edma_handle_t eDmaHandle_2 );

/* printer status tracking */
AT_QUICKACCESS_SECTION_DATA( PrStatusInfo currentStatus );
AT_QUICKACCESS_SECTION_DATA( PrStatusInfo prevStatus );

extern void setOperation( unsigned char operation, PrStatusInfo *pStatus );
extern void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus);
extern void setNextOperation( PrStatusInfo *pStatus );
extern void skipNextOperation( PrStatusInfo *pStatus ); 
extern void jumpToOperation( PrStatusInfo *pStatus, unsigned char index );
extern int getGhostMCntr( void );
extern void clearGhostMCntr( void );
extern void setTriggerMotorStop( void );
extern void clearTriggerMotorStop( void );
extern void sendPrStatus( PrStatusInfo *pStatus, bool interrupt );
extern bool startLabelTakenTimer( void );

AT_NONCACHEABLE_SECTION_INIT(static uint16_t tModifier) = 0;

extern Pr_Config config_;
extern SemaphoreHandle_t pCutDoneSemaphore;
extern volatile ADCManager adcManager;

int labelAlignment;
bool sizingLabels = false;

AT_NONCACHEABLE_SECTION_INIT(uint16_t leadInStepTarget) = 0;
AT_NONCACHEABLE_SECTION_INIT(uint16_t expelStepTarget) = 0;

AT_NONCACHEABLE_SECTION_INIT(uint16_t streamingLabelBackwind) = 0;

AT_NONCACHEABLE_SECTION_INIT(int shootIndex) = 0;

AT_NONCACHEABLE_SECTION_INIT(bool paused_) = false;

unsigned short getNumPrintLinesLeft(void)
{
   return engine.numPrintLines;
}

void delay( unsigned long time );
extern void testPrintHeadTransfer( void );

AT_NONCACHEABLE_SECTION_INIT(uint16_t TPHStepsPastGapThisPrint) = 0;
char sizingState = 0;

AT_QUICKACCESS_SECTION_DATA( static unsigned char pattern1[( 80 * 2 )] ) = {0};

AT_QUICKACCESS_SECTION_DATA(DotCheckerStatus dotChecker);

AT_QUICKACCESS_SECTION_DATA(uint32_t dotCheckerCalibratedAverage) = 0;

PrHeadCalResponse response;

AT_QUICKACCESS_SECTION_DATA(LowLabelStatus lowLabelStatus);


AT_QUICKACCESS_SECTION_DATA(short lowLabelPeelingMaxFromHost);
AT_QUICKACCESS_SECTION_DATA(short lowLabelPeelingMinFromHost);
AT_QUICKACCESS_SECTION_DATA(short lowLabelStreamingMaxFromHost);
AT_QUICKACCESS_SECTION_DATA(short lowLabelStreamingMinFromHost);


/******************************************************************************/
/*!   \fn void initializePrintEngine( unsigned int contrast, unsigned int mediaCount, 
                                      QueueHandle_t pHandle )

      \brief
        This function initializes the print engine.
        

      \author
          Aaron Swift
*******************************************************************************/
void initializePrintEngine( unsigned int contrast, unsigned int mediaCount, 
                            QueueHandle_t pHandle )
{
    if( pHandle != NULL ) {
        /* sizing debug - freestanding scale 
        pTakeup = pvPortMalloc( 1000 ); */
 
        /* assign our command queue */
        pCmdQHandler_ = pHandle;
        /* get the head temperature index */
        unsigned char tempIndex = getPrintheadTemperatureInCelsius();

        pCutSemaphore = (SemaphoreHandle_t)getCutSemaphore();
                
        engine.headType = getPrintHeadType();
        /* get the line compensation levels */
        engine.levels = getCompLevel( engine.headType );
        /* get the over all line burn time */
        engine.sltTime = getSltTime( engine.headType, contrast );
        
        engine.contrast = config_.contrast_adjustment;
        engine.sltHalfTime = engine.sltTime / 2;
        engine.lineCounter = 0;
        engine.burnSequence = 0; 

        engine.numSteps = 0;
        engine.numPrintLines = 0;
        engine.outOfMediaCnt = 0;
        engine.stepsOffset = 0;
        engine.labelTracking = 0;
        engine.totalLinesToPrint = 0;
        engine.maxMediaCount = 200;
        engine.direction = FORWARD_;
        engine.labelOrientation = HEEL_FIRST;
        engine.pHistory = getFirstHistoryLine();
        engine.pImage        = getImageBuffer();
        
        labelAlignment      = 0;
        /* keep track of three label positions */
        label0.position     = 0;
        label0.next         = &label1;
        label1.position     = 0;
        label1.next         = &label2;
        label2.position     = 0;
        label2.next         = &label0;
                
        pCurrentLabel = &label0;
        
        applyVerticalOffset_ = false;
        prevVertOffset_ = 0;
#if 1
        /* intialize stepper motor */
        initializeStepper( engine.direction );
        
        /* initialize the paper takeup motor control 
        initTakeupIntr();       */
        
        /* gets rid of whine before running the stepper */
        powerOffStepper();
#endif        
        /* set the operational directive to idle operation */
        engine.currentCmd.generic.directive =  IDLE_DIRECTIVE;
        
        /* setup the start times for history, adjacency and current line */        
        //PRINTF("initializePrintEngine() - engine.levels == %d\r\n", engine.levels);
        
        if( engine.levels == 3 ) {
            engine.histAdj[0].compType = FIRST_LEVEL_HIST;
            engine.histAdj[0].time = getHistoryTime( contrast );
            engine.histAdj[1].compType = FIRST_LEVEL_ADJ;
            engine.histAdj[1].time = getAdjacencyTime( contrast );
            engine.histAdj[2].compType = CURRENT_LINE;
            engine.histAdj[2].time = getCurrentLineTime( contrast );

            engine.pwmStartTime = getPwmStartTime( contrast );
            engine.pwmDutyCycle = getPwmDutyCycle( contrast );
        } else {
            engine.histAdj[0].compType = FIRST_LEVEL_HIST;
            engine.histAdj[0].time = getHistoryTime( contrast );
            engine.histAdj[1].compType = CURRENT_LINE;
            engine.histAdj[1].time = getCurrentLineTime( contrast );

            engine.pwmStartTime = getPwmStartTime( contrast );
            engine.pwmDutyCycle = getPwmDutyCycle( contrast );
        }
        
        
        /* initialize print engine timer */
        startLineTimer( false );
        initializePrintEngineTimer( DEFAULT_ENGINE_COUNT );
    } else {
        PRINTF("initializePrintEngine(): PR Command Queue NULL! \r\n");      
    }    
}

/******************************************************************************/
/*!   \fn void initializePrintHeadSPI( void )

      \brief
        This function initializes the SPI peripheral and edma used for data 
        transfer to the print head.
        Note: 4" of label data can be transfered to the print head in 300mS

      \author
          Aaron Swift
*******************************************************************************/
void initializePrintHeadSPI( void )
{
    lpspi_master_config_t masterConfig;
    
    #if 0       /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    PrintEngine *pEngine  = getPrintEngine();
    if( pEngine != NULL ) {            
        masterConfig.baudRate                      = 12000000u; /* 10Mhz */
        masterConfig.bitsPerFrame                  = 8;      
        masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
        masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;

        masterConfig.direction                     = kLPSPI_MsbFirst;
        masterConfig.pcsToSckDelayInNanoSec        = 50;
        masterConfig.lastSckToPcsDelayInNanoSec    = 50;
        masterConfig.betweenTransferDelayInNanoSec = 50;
        masterConfig.whichPcs                      = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;    
        masterConfig.dataOutConfig                 = kLpspiDataOutTristate;

        /* intialize the spi interface */
        LPSPI_MasterInit( LPSPI4, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
        NVIC_SetPriority( LPSPI4_IRQn, 1 );
        LPSPI_MasterTransferCreateHandle( LPSPI4, &spiHeadMasterHandle, LPSPI_MasterUserCallback, NULL );                
    } else {
        PRINTF("initializePrintHeadSPI(): printEngine is NULL. critical error!\r\n" );
    }
    #else  
    
    edma_config_t edmaConfig;
  
    lpspi_transfer_t masterXfer;
    
    LPSPI_MasterGetDefaultConfig( &masterConfig );
    
    /* intialize the dma mux */
    DMAMUX_Init( DMAMUX );
    /* setup the dma channel 0 for source 79: lpspi4 rx this channel will not be used but needs setup! */
    DMAMUX_SetSource( DMAMUX, 0, kDmaRequestMuxLPSPI4Rx );
    DMAMUX_EnableChannel( DMAMUX, 0 );

    /* setup the dma channel 1 for source 80: lpspi4 tx  */
    DMAMUX_SetSource( DMAMUX, 1, kDmaRequestMuxLPSPI4Tx );
    DMAMUX_EnableChannel( DMAMUX, 1 );
    
    /* edma already intialized in sensors.c  */
    EDMA_GetDefaultConfig( &edmaConfig );   
    EDMA_Init( DMA0, &edmaConfig );
    
    PrintEngine *pEngine  = getPrintEngine();
    if( pEngine != NULL ) {        
        /* setup spi4 master configuration kjt-56 max clock rate 12Mhz */     
        masterConfig.baudRate                      = 12000000u; /* 10Mhz */
        masterConfig.bitsPerFrame                  = 8;       
        masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
        masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
        //if( pEngine->labelOrientation ==  HEAD_FIRST ) {
            //masterConfig.direction                 = kLPSPI_LsbFirst; 
        //} else {
            masterConfig.direction                 =  kLPSPI_MsbFirst;
        //}
        masterConfig.pcsToSckDelayInNanoSec        = 50;
        masterConfig.lastSckToPcsDelayInNanoSec    = 50;
        masterConfig.betweenTransferDelayInNanoSec = 50;
        masterConfig.whichPcs                      = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;    
        masterConfig.dataOutConfig                 = kLpspiDataOutTristate;
        
        
        /*
        masterConfig.baudRate                      = (1000000U * 14); //~12Mhz
        masterConfig.bitsPerFrame                  = 8U;
        masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;
        masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
        masterConfig.direction                     = kLPSPI_MsbFirst;
        masterConfig.whichPcs                      = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;
        masterConfig.betweenTransferDelayInNanoSec   = 0U; 
        */
        
        /* intialize the spi interface */
        LPSPI_MasterInit( LPSPI4, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
        NVIC_SetPriority( LPSPI4_IRQn, 1 );
          
        memset( &eDmaHandle_0, 0, sizeof(edma_handle_t) );
        memset( &eDmaHandle_1, 0, sizeof(edma_handle_t) );
        
        /* create handles for all three dma channels */ 
        EDMA_CreateHandle( &eDmaHandle_0, DMA0, 0 ); 
        EDMA_CreateHandle( &eDmaHandle_1, DMA0, 1 );
      
        /* inialize the master handle with the callback */ 
        LPSPI_MasterTransferCreateHandleEDMA( LPSPI4, &spiHeadMasterHandle, LPSPI_MasterUserCallback, 
                                             NULL, &eDmaHandle_0, &eDmaHandle_1 );        
    } else {
        PRINTF("initializePrintHeadSPI(): printEngine is NULL. critical error!\r\n" );
    }
    #endif    
    spiDmaTransferComplete = false;
}

/******************************************************************************/
/*!   \fn void DSPI_MasterUserCallback( SPI_Type *base, 
                                        lpspi_master_handle_t *handle, 
                                        status_t status, void *userData )
      \brief
        This function is called after one complete dma transfer has occured.
       
      \author
          Aaron Swift
*******************************************************************************/
#if 1 /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData )
#else 
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData )
#endif
{
    /* assert data latch only on history loads */
    if( !isCurrentLine() ) {
        /* data is transfered to the head, assert the data latch */ 
        GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, false );         //false
        
        /* load history line*/
        //historyAdjacency();
        //loadHistory();
    }
 
    if( status == kStatus_Success ) {
        spiDmaTransferComplete = true;
    } else {
        PRINTF("DSPI_MasterUserCallback(): Dma transfer failed. critical error!\r\n" );
    }
}


/******************************************************************************/
/*!   \fn void addCmdToQueue( PrCommand *pCmd )

      \brief
        This function adds printer commands to the command queue
        

      \author
          Aaron Swift
*******************************************************************************/
void addCmdToQueue( PrCommand *pCmd )
{
    /* add command message to the command queue */
    BaseType_t result = xQueueSendToBack( pCmdQHandler_, (void *)pCmd, 0 );
    if( result != pdTRUE ) {
        PRINTF("handlePrinterMsg(): PR Command Queue full! \r\n");  
    }               
}

/******************************************************************************/
/*!   \fn void setSkipLabelTakenCheck( void )

      \brief
        This function set the skip label taken flag. Flag is used in wait till
        engine function which we will skip if bit is set.
        
      \author
          Aaron Swift
*******************************************************************************/
void setSkipLabelTakenCheck( void )
{
    skipLabelTaken_  = true;  
}

/******************************************************************************/
/*!   \fn void startPrintEngine( void )

      \brief
        This function resets the print engine counters and starts the line burn/printhead loading,
        print roller motor and paper takeup motor interrupts.    

      \author
          Aaron Swift
*******************************************************************************/
void startPrintEngine( void )
{ 
    //PRINTF("startPrintEngine()\r\n");
    //PRINTF("streamingLabelBackwind = %d\r\n", streamingLabelBackwind);
    //PRINTF("startPrintEngine() numPrintLines: %d\r\n", engine.numPrintLines);
    //PRINTF("backwindAfterSizing = %d\r\n", backwindAfterSizing);
    //PRINTF("startPrintEngine() contrast = %d\r\n", config_.contrast_adjustment);
    //PRINTF("startPrintEngine() label_width = %d\r\n", config_.label_width);
    
    //PRINTF("PH Temp = %d\r\n", getPrintheadTemperatureInCelsius());
    //PRINTF("numLabelsOnRoll %d\r\n", numLabelsOnRoll);
     
    //set the estimated amount of labels on roll of labels
    updateNumberOfLabelsOnRoll();
  
    //calculate roll completion percentage estimations and update currentStatus
    updateRollCompletionPercentage();
    
    setLTWaitCount_(0);
    
    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
    
    /* reset engine counters */
    engine.lineCounter = 0;
    engine.burnSequence = 0;
    engine.linePrintDone = false; // are we done burning and loading lines?
    leadInSteps = 0; // the amount of steps before lines begin to be loaded in the printhead and burnedd
    leadInDone = false; // are we done with the lead in?
    expelSteps = 0; // the amount of steps to continue stepping after we are done burning and loading llines
    expelDone = false; // are we done with the expel?
    //labelLowIndexOffset = 0; // reset the label low buffer index
    setShootIndex(0);
    //TPHStepsPastGapThisPrint = 0;
    
    setHeadTimings();
      
    /* turn on print head power */ 
    setHeadPower( true );
    //setHeadPower( false );
    
    /* initialize the printhead roller motor interrupt, this function is currently defined in 
    takeupMotor.c */
    initTPHIntr();
    
    /* if we are taking up paper, tighten the paper before the print starts */
    if(getTakingUpPaper() == true)
    {
        /* if(engine.totalLinesToPrint >= 1200) tensionModifier is incremented by 2 each print, 
        otherwise it is incremented by 1 each print, currently it is capped to 400. tensionModifier is
        reset/estimated every cassette open/close*/
        uint16_t startingTUSpeed = (760 + getTensionModifier());
        
        if(startingTUSpeed > 1160) //1160 uS
        {
            startingTUSpeed = 1160;
        }
      
        enableVScope();
        /*tightenStock starts an interupt that pulses the takeup motor step pin at startingSpeed until
        the takeup clutch tension sensor counts equal the first argument. */
        
        unsigned short tensionSetting = 0;
        static float tensionDivider = 0.85; /* made this variable so it can be adjusted while debugging */
        tensionSetting = (unsigned short)((float)config_.takeup_sensor_max_tension_counts/tensionDivider);
        //PRINTF("tension SP: %d\r\n",tensionSetting);
        tightenStock(   
            tensionSetting,   
            startingTUSpeed, 
            true,
                        HALF_STEP);
    }        
    
    /* if we are taking up paper, start the takeup motor interrupt */
    if(getTakingUpPaper() == true)
    {
        tModifier = getTensionModifier();
        uint16_t startingSpeed = 760 + getTensionModifier();
        
        if(startingSpeed > 1500) //1160 uS
        {
            startingSpeed = 1500;
        }
      
        /* increment the tension modifier more for larger labels  */
        if(engine.totalLinesToPrint >= 1200)
        {
            tModifier += 2;
            setTensionModifier(tModifier);
        }
        else
        {
            tModifier++;
            setTensionModifier(tModifier);
        }

        if(tModifier >= 400)
        {
            tModifier = 400;
            setTensionModifier(tModifier);
        }

        /* start the takeup motor intr */
        stepTUMotor(engine.totalLinesToPrint + 6000, startingSpeed);
    }
    
    /* start the printhead roller motor */
    if(continuousStock_ == true)
    {
        /* start the print roller motor interrupt, for a continuous stock print we want to travel the 
        amount of steps equal to (lead in + numPrintLines + expel)*/
        startTPHIntr(engine.numPrintLines + 210 + getIndirectData( 4 ));
    }
    else
    {
        /* getLargeGapFlag() will return true if during the last sizing the label gap was estimated to
        to be larger than 55 steps (HT/RFID labels with a larger gap than the standard smaller GT gap)*/
        if(getLargeGapFlag() == true)
        {    
            if( getTakingUpPaper() == true )
            {
                startTPHIntr((engine.numPrintLines) + 215 + getIndirectData( 4 )); //printlines + expel + lead in
            }
            else
            {
                if(getWaitForLabelTaken() == true)
                {
                    startTPHIntr((engine.numPrintLines) + getStreamingLeadInMod() + getStreamingExpelMod()); //printlines + expel + lead in 
                }
                else
                {
                    startTPHIntr((engine.numPrintLines) + getStreamingLeadInMod()); //printlines + expel + lead in                     
                }   
            }
        }  
        else
        {
            if( getTakingUpPaper() == true )
            {
                startTPHIntr((engine.numPrintLines) + 125 + getIndirectData( 4 )); //printlines + expel + lead in                
            }
            else
            {
                if(getWaitForLabelTaken() == true)
                {
                    startTPHIntr((engine.numPrintLines) + getStreamingLeadInMod() + getStreamingExpelMod()); //printlines + expel + lead in 
                }
                else
                {
                    startTPHIntr((engine.numPrintLines) + getStreamingLeadInMod()); //printlines + expel + lead in                     
                }  
            }
        }      
    }
    
    /* init and start the line timer intr that loads the printhead and burns the lines */
    startLineTimer( true );
}


/******************************************************************************/
/*!   \fn void setLineTimerIntLevel( unsigned int level )

      \brief
        This function sets the line timer interrupt level.
        Normal printing operations should run at a level of 2 and dot wear 
        should execute with a level of 5. If dotwear runs at level 2 a timing 
        issue occurs with sampling the dot within the current bubble.

      \author
          Aaron Swift
*******************************************************************************/
void setLineTimerIntLevel( unsigned int level )
{
    interruptLevel_ = level;  
    NVIC_SetPriority( GPT2_IRQn, interruptLevel_ );
    EnableIRQ( GPT2_IRQn );
}

/******************************************************************************/
/*!   \fn void startLineTimer(  bool start  )

      \brief
        This function initializes and starts the line print timer if the start
        flag is true.

        Timer period    dependent on contrast setting
        Timer Freq      1.953Mhz   
      \author
          Aaron Swift
*******************************************************************************/
void startLineTimer(  bool start  )
{ 
    //PRINTF("startLineTimer contrast = %d\r\n", config_.contrast_adjustment);

    /* set GPT2 intr handler type to printhead loading and line burning */
    setGPT2IntrType(BURN_LINE);
  
    /* initialize line printer timer with 3 compare interrupts */    
    gpt_config_t gptConfig;
    
    /* obtain default configuration for general purpose timer module */
    GPT_GetDefaultConfig( &gptConfig );
    
    gptConfig.enableRunInWait = false; 
    gptConfig.enableRunInStop = false;
    gptConfig.enableRunInDoze = false;  
    gptConfig.enableRunInDbg = false;   
    gptConfig.enableFreeRun = true;

    /* initialize gpt module */
    GPT_Init( GPT2, &gptConfig );    
    GPT_SetClockDivider( GPT2, 32 );      

    /* load the compare registers with their initial values and setup the output 
       compare mode to toggle output on a match*/

    /* set channel 1 for slt time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.histAdj[0].time ); 
    /* set channel 2 for history time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2,  engine.histAdj[1].time ); 
    /* set channel 3 for current line time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel3, engine.sltHalfTime ); 
    
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel2, kGPT_OutputOperation_Disconnected );
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel3, kGPT_OutputOperation_Disconnected );
    
    /* enable channel interrupt flags. */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );   
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare2InterruptEnable );    
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare3InterruptEnable );         

    setLineTimerIntLevel( LINE_PRINTER_TIMER_PRIORITY );

    if( start ) {
        /* start the timer */
        GPT_StartTimer( GPT2 );
    }
}


/******************************************************************************/
/*!   \fn void scalePrintLineTimesRamped(void)

      \brief
          This function changes the line burn times based on the speed of the print
          roller and the PH temperature. It is necessary because we are now printing
          at a slower speed until after the peel operation has occurred.

          This code is based on code Chris wrote in the CK/27 branch
      \author
          Tom Fink / Chris King
*******************************************************************************/
void scalePrintLineTimesRamped(void)
{
   /* Adjust the currentLineTime based on PH temp */
   int currentTemperature = getTempAtStart();
   float currentLineTimeTemperatureAdjusted = 0;
   int difference = currentTemperature - 25; 
   int increments = difference / 5;
   float modificationFactor = 1 + ((float)increments * 0.0125);
   currentLineTimeTemperatureAdjusted = ((float)rohm80mmCurrentLine[ engine.contrast ] * modificationFactor);
  
   if(  /* getLeadInDone() == true && */ engine.linePrintDone == false )
   {
         
      float percentageToScale = getRollerMotorPercentFinalSpeed();
      /* percentage should always be > 100% because we're printing slower than full speed before the peel */
      if(percentageToScale < 1)
         percentageToScale = 1;   
               
      /* scale sltTime to match print roller motor speed */
      engine.sltTime = (unsigned short)(rohm80mmSLTTimes[engine.contrast]*percentageToScale);
      
      /* change in SLT time should always be positive (we're printing slower */
      short changeInSLTTime = 0;
      changeInSLTTime = (short)(engine.sltTime-rohm80mmSLTTimes[engine.contrast]);
      if(changeInSLTTime < 10)
         changeInSLTTime = 0;
      
      changeInSLTTime = (short)(changeInSLTTime/4);  
      //changeInSLTTime = 0;
      
      if(percentageToScale == 1)
         changeInSLTTime = 1;  /* Debug only. For setting a breakpoint. TFinkToDo delete */
      
      /* Adjust history and current line times so they burn the same amount of time they
         will at full speed. This is probably the simplest adjustment and it seems to work pretty well.
         There may be a better but more complicated way to adjust */
      engine.sltHalfTime = engine.sltTime / 2;
      engine.histAdj[0].compType = FIRST_LEVEL_HIST;        
      engine.histAdj[0].time = ((unsigned short) (rohm80mmHistory[ engine.contrast ]*percentageToScale))+changeInSLTTime;
      engine.histAdj[1].compType = CURRENT_LINE;    
      engine.histAdj[1].time = ((unsigned short) (currentLineTimeTemperatureAdjusted*percentageToScale)) + changeInSLTTime; 
      engine.pwmStartTime = engine.sltTime - 10;  /* We're not using PWM. Set it a bit before sltTime so the PWM and SLT interrupts don't happen at the same time */
      engine.pwmDutyCycle = ( rohm80mmPwmDuty[ engine.contrast ]);
   }
   else
   {     
      /* Lead in not done. Lead in is blank lines, so it shouldn't what the burn times are */
      engine.sltTime = rohm80mmSLTTimes[ engine.contrast ];
      engine.sltHalfTime = engine.sltTime / 2;
      engine.histAdj[0].compType = FIRST_LEVEL_HIST;        
      engine.histAdj[0].time = rohm80mmHistory[ engine.contrast ];
      engine.histAdj[1].compType = CURRENT_LINE;    
      engine.histAdj[1].time = currentLineTimeTemperatureAdjusted; 
      engine.pwmStartTime = rohm80mmPwmStart[ engine.contrast ];
      engine.pwmDutyCycle = ( rohm80mmPwmDuty[ engine.contrast ]);
   }
   
   
   
}


/******************************************************************************/
/*!   \fn void stopLineTimer( void )                                                             
 
      \brief
        This function disables all line timer interrupts and stops the print
        timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void stopLineTimer( void )
{
    /* stop the timer */
    GPT_StopTimer( GPT2 );
    
    /* clear any flags that might be set */
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag );
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag );
    
    /* modifify print line times for ramping print roller speed */
    scalePrintLineTimesRamped();

    /* set channel 1 for slt time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.histAdj[0].time); 
    /* set channel 2 for history time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2,  engine.histAdj[1].time); 
    /* set channel 3 for current line time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel3, engine.sltTime); 
}
 

/******************************************************************************/
/*!   \fn static void( void )

      \brief
        This interrupt expires at the SLT time with a rate of .666uSec.
        Turn off PWM of the stobe and reinitialize the strobe line as gpio.
        Prep the next line data by determining history and adjacency and loading
        the next line to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void lineTimerSLT( void ) 
{    
    /* deinitialize strobe pwm channel, we are not currently PWM'ing the strobe pins */
    //PWM_Deinit( PWM2, kPWM_Module_0 );
    
    /*  re-enable peripheral io control of the strobe pin */
    gpio_pin_config_t strobeConfig = { kGPIO_DigitalOutput, 0 };
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 0x70A0U );
   
    /* release data strobe */
    GPIO_PinInit( PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, &strobeConfig );
    GPIO_PinInit( PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, &strobeConfig );
    
    /* release strobe enable */
    //GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, true ); 
    
    engine.burnSequence = 0; // reset the burnSequence

    /* if we aren't done with the lead in yet and we are using continuous stock */
    if(getTakingUpPaper() == false)
    {
        if(leadInDone == false && engine.linePrintDone == false)
        {
            if(getCutterInstalled_() == true || getUsingContinuous() == true)
            {
                if(getCutterInstalled_() == true)
                {
                    leadInStepTarget = (config_.verticalPosition);
                }
                else
                {
                    leadInStepTarget = (config_.verticalPosition + 25);
                }
                
            }
            else
            {
                if( getLargeGapFlag() == true )
                {
                    leadInStepTarget = (calculateHTLeadInTarget() + getStreamingLeadInMod() + config_.verticalPosition);
                }
                else
                {
                    leadInStepTarget = (calculateGTLeadInTarget() + getStreamingLeadInMod() + config_.verticalPosition);
                }
            }
            
            leadInSteps++;
          
            if(leadInSteps >= leadInStepTarget)
            {
                leadInDone = true;

                stopLineTimer();
                GPT_StartTimer( GPT2 );
            }
            else /* continue lead in */
            {
                stopLineTimer();
                GPT_StartTimer( GPT2 );
            }
        }
        else if(leadInDone == true && engine.linePrintDone == true && expelDone == false)
        {            
            if(getWaitForLabelTaken() == true)
            {
                expelStepTarget = 0;
            }
            else
            {
                expelStepTarget = 0;
            }
            
            expelSteps++;
          
            setHeadPower(false);
            
            if(expelSteps >= expelStepTarget)
            {
                expelDone = true;
                
                stopLineTimer();
            }
            else 
            {
                stopLineTimer();
                GPT_StartTimer( GPT2 );
            }
        }
        else /* lead in is done, printing in progress, when engine.linePrintDone == true expel will begin */
        {
            /* are we done with the image? */
            if( --engine.numPrintLines <= 0 )   
            {
                //PRINTF("line print done\r\n");
                engine.linePrintDone = true;
                
                /* printhead power off */
                setHeadPower(false);
                
                if(getDotWearHandle() != NULL && readyToNotify() )
                    xTaskNotifyFromISR( getDotWearHandle(), DOT_DONE,eSetBits, pdFALSE );
                
                stopLineTimer(); 
                GPT_StartTimer( GPT2 );
            } 
            else 
            {
                stopLineTimer();       

                /* load history line*/
                loadHistory();  
                
                GPT_StartTimer( GPT2 );
            }
        } 
    }
    else
    {
        if(leadInDone == false && continuousStock_ == true)
        {
            /* increment our lead in steps */
            leadInSteps++;
          
            /* if our lead in steps are greater than or equal to our desired lead in (hardcoded to expel + 56) 
            then set leadInDone to true and restart our timer*/
            
            uint16_t contLeadInSteps = 0;
            
            if(getCutterInstalled_() == true)
            {
                contLeadInSteps = (config_.verticalPosition);
            }
            else
            {
                contLeadInSteps = (config_.verticalPosition + 25);
            }
            
            if(leadInSteps >= contLeadInSteps)
            {
                //PRINTF("lead in done\r\n");
                leadInDone = true;
                
                stopLineTimer();
                GPT_StartTimer( GPT2 );
            }
            else /* continue lead in */
            {
                stopLineTimer();
                GPT_StartTimer( GPT2 );
            }
        }/* if we aren't done with the lead in yet and we are using die-cut stock */
        else if(leadInDone == false && continuousStock_ == false)
        {
            /* increment our lead in steps */
            leadInSteps++;
          
            /* Walmart RFID/HT stock lead in */
            if(getLargeGapFlag() == true)
            {
                leadInStepTarget = calculateHTLeadInTarget() + config_.verticalPosition;
                
                /* if our lead in steps are greater than or equal to our desired lead in step amount
                then set leadInDone to true and restart our timer*/
                if(leadInSteps >= leadInStepTarget)
                {
                    leadInDone = true;
                    
                    //PRINTF("lead in target = %d\r\n", leadInStepTarget);
                                    
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
                else/* continue lead in */
                {
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
            }
            else /* GT stock lead in*/
            {               
                leadInStepTarget = calculateGTLeadInTarget() + config_.verticalPosition;
                //PRINTF("\r\nlead in step target = %d\r\n", calculateGTLeadInTarget());
              
                /* if our lead in steps are greater than or equal to our desired lead in step amount
                then set leadInDone to true and restart our timer*/
                if(leadInSteps >= leadInStepTarget)
                {
                    leadInDone = true;
                    
                    //PRINTF("lead in target = %d\r\n", leadInStepTarget);
                    
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
                else /* continue lead in */
                {
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
            }
        } /* lead in is done, printing is done, continuous stock expel */
        else if(leadInDone == true && engine.linePrintDone == true && expelDone == false && continuousStock_ == true)
        {           
            /* increment expel steps */
            expelSteps++;
            
            /* if our expel steps are greater than or equal to our desired expel (hardcoded to expel + 136) 
            then set expelDone to true and restart our timer*/
            if(expelSteps >= (getIndirectData( 4 )))
            {
                //PRINTF("expel done\r\n");
                expelDone = true;
                
                stopLineTimer();
            }
            else /* continue exxpel */
            {
                stopLineTimer();
                GPT_StartTimer( GPT2 );
            }
        } /* lead in is done, printing is done, die-cut stock expel */
        else if(leadInDone == true && engine.linePrintDone == true && expelDone == false && continuousStock_ == false)
        {
            expelDone = true;
                    
            stopLineTimer();
        }
        else /* lead in is done, printing in progress, when engine.linePrintDone == true expel will begin */
        {
            /* are we done with the image? */
            if( --engine.numPrintLines <= 0 )   
            {
                //PRINTF("line print done\r\n");
                engine.linePrintDone = true;
                
                /* printhead power off */
                setHeadPower(false);
                
                if(getDotWearHandle() != NULL && readyToNotify() )
                    xTaskNotifyFromISR( getDotWearHandle(), DOT_DONE,eSetBits, pdFALSE );
                
                stopLineTimer(); 
                GPT_StartTimer( GPT2 );
            } 
            else 
            {
                stopLineTimer();       

                /* load history line*/
                loadHistory();  
                
                GPT_StartTimer( GPT2 );
            }
        } 
    }
}

/******************************************************************************/
/*!   \fn void loadHistory( void )

      \brief
        This function configures and starts the DMA loading of the history line
        to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void loadHistory( void ) 
{
    if( historyEnabled_ ){
        //PRINTF("\r\n\r\nHISTORY ENABLED\r\n\r\n");
        lpspi_transfer_t masterXfer;  
         
        clearBurnSequence();
       
        /* setup master transfer */
        masterXfer.txData = (unsigned char *)engine.pHistory;
        masterXfer.rxData = NULL;
             
        if( getHeadStyleSize() == HEAD_DOTS_72MM ) {
            masterXfer.dataSize = PRINTER_HEAD_SIZE_72MM;
        } else if( getHeadStyleSize() == HEAD_DOTS_80MM ) {
            masterXfer.dataSize = PRINTER_HEAD_SIZE_80MM;        
        } else {
            PRINTF( "loadHistory(): unsupported head style!\r\n"); 
        }
        
        /*
        PRINTF("\r\n\r\nHistory:\r\n");
        for(int i = 0; i < masterXfer.dataSize; i++)
        {
            PRINTF("%b,", masterXfer.txData[i]);
            //takeupDelayShort();
        }
        PRINTF("\r\n\r\n");
        */
        masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
        
        #if 1   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadHistory(): Print head load error! %d\r\n", result );
            PRINTF("lines left: %d\r\n", engine.numPrintLines);
            PRINTF("lines to print: %d\r\n", engine.totalLinesToPrint);
            /* stop engine timer and fix problem */
            stopLineTimer();
            
            #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
            /* abort the current transfer and retry */
            LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
            #else
            LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
            #endif            
            
            /* try re-init of SPI/DMA */
            initializePrintHeadSPI();
            #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
            result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
            #else 
            result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
            #endif            
            if( kStatus_Success != result ) {
                PRINTF( "loadHistory(): Nope that doesn't work! %d\r\n", result );    
            }
            /* restart our timer */
            startLineTimer( true );
        }  
    }
}


/******************************************************************************/
/*!   \fn void loadPrintLine( void )

      \brief
        This function configures and starts the DMA loading of the next line
        to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void loadPrintLine( void ) 
{

    lpspi_transfer_t masterXfer;  
    
    unsigned long offset = 0;    
    
    unsigned long printLines = 0;
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        printLines = N_PRINTER_LINES_72MM;
    }
    else
    {
        printLines = N_PRINTER_LINES_80MM;
    }
    
    /* do not allow the print engine outside the image buffer */
    //if( engine.lineCounter > N_PRINTER_LINES ) {
    if( engine.lineCounter ==  printLines) {
        engine.lineCounter = 0;
        /* label image is greater than 5" */ 
        offset = engine.lineCounter;        
    } else {
    	
    	if(getHeadStyleSize() == HEAD_DOTS_72MM)
    	{
    		offset = ( engine.lineCounter * PRINTER_HEAD_SIZE_72MM );	 
    	}
    	else
    	{
    		offset = ( engine.lineCounter * PRINTER_HEAD_SIZE_80MM );	
    	}
    	
           
    }
    
        
    /*
    if( engine.lineCounter == ( N_PRINTER_LINES - 2 ) ) {
        memcpy( &lastBfr[0], (unsigned char *)engine.pImage + offset, 144 );   
    } */

    /* setup master transfer */
    masterXfer.txData = (unsigned char *)engine.pImage + offset; 
    masterXfer.rxData = NULL;
    
    
    

    if( getHeadStyleSize() == HEAD_DOTS_72MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_72MM;
    } else if( getHeadStyleSize() == HEAD_DOTS_80MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_80MM;        
    } else {
        PRINTF( "loadHistory(): unsupported head style!\r\n"); 
    }
    
    
    masterXfer.configFlags =  kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;

    #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );    
    #else
    unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
    #endif
        
    
    if( kStatus_Success != result ) {
        PRINTF( "loadPrintLine(): Print head load error! %d\r\n", result );
        while(1)
        {
            __NOP();
        }
        /* stop engine timer and fix problem */
        stopLineTimer();
        
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        /* abort the current transfer and retry */
        LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
        #else
        LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
        #endif        
        /* try re-init of SPI/DMA */
        initializePrintHeadSPI();
        
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadPrintLine(): Print head load error! %d\r\n", result );      
        }
        /* restart our timer */
        startLineTimer( true );
    }
   
    /* NOTE: printhead latch is negated in the SPI-EDMA callback */
    
    //prevLineCounter = currLineCounter;
    engine.lineCounter++;
    currLineCounter = engine.lineCounter;
    
    /* are we about to rollover? ( + 1 still need to print last line ) SOF-5965*/   
}

/******************************************************************************/
/*!   \fn void loadZeroPrintLine( void )

      \brief
        This function configures and starts the DMA loading of a blank line into
        the printhead

      \author
          Aaron Swift
*******************************************************************************/
void loadZeroPrintLine( void ) 
{
    lpspi_transfer_t masterXfer;  
    
    unsigned long offset = 0; 
    
    /* setup master transfer */
    masterXfer.txData = (unsigned char *)pattern1;
    masterXfer.rxData = NULL;
    
    if( getHeadStyleSize() == HEAD_DOTS_72MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_72MM;
    } else if( getHeadStyleSize() == HEAD_DOTS_80MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_80MM;        
    } else {
        PRINTF( "loadZeroPrintLine(): unsupported head style!\r\n"); 
    }
    
    masterXfer.configFlags =  kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    
    #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
    #else
    unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
    #endif
    
    if( kStatus_Success != result ) {
        PRINTF( "loadZeroPrintLine(): Print head load error! %d\r\n", result );
        /* stop engine timer and fix problem */
        stopLineTimer();
        
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */        
        /* abort the current transfer and retry */
        LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
        #else
        LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
        #endif        
        
        /* try re-init of SPI/DMA */
        initializePrintHeadSPI();
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadZeroPrintLine(): Print head load error! %d\r\n", result );      
        }
        /* restart our timer */
        startLineTimer( true );                     
    }
    
    delay(150);
    GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true );     
}

bool isCurrentLine( void )
{
    bool result = false;
    if( engine.burnSequence > 0 ) {
        result = true;
    }
    return result;
}

void clearBurnSequence( void ) 
{
    engine.burnSequence = 0;   
}

void clearPrevVertOffset( void )
{
    prevVertOffset_ = 0;    
}

/******************************************************************************/
/*!   \fn void lineTimerStrobe( void )

      \brief
        This function sets up and start pwm of the print head strobe signal.
        PWM of the strobe line regulates the heat to each dot in the print head 
        until the slt time has expired.

      \author
          Aaron Swift
*******************************************************************************/
void lineTimerStrobe( void )
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal;

    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;          // kPWM_LowTrue kPWM_HighTrue;
    /* strobe signal is active low! pulse width low is measured*/ 
    pwmSignal.dutyCyclePercent = engine.pwmDutyCycle;      
    pwmSignal.deadtimeValue    = 0;
    pwmSignal.faultState       = kPWM_PwmFaultState0;

    
    /* disable GPIO control of the pin select ftm0 pwm control for printhead strobe */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0 );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0x10B0U ); 

    PWM_GetDefaultConfig( &pwmConfig );
    pwmConfig.prescale = kPWM_Prescale_Divide_1;
    /* Use full cycle reload */
    pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
    /* PWM A & PWM B operate as 2 independent channels */
    pwmConfig.pairOperation   = kPWM_Independent;
    pwmConfig.enableDebugMode = true;
    
    PWM_Init( PWM2, kPWM_Module_0, &pwmConfig);     
    PWM_SetupPwm( PWM2, kPWM_Module_0, &pwmSignal, 1U, kPWM_SignedCenterAligned, 25265U , CLOCK_GetFreq( kCLOCK_IpgClk ) );
    
    PWM_SetPwmLdok( PWM2, kPWM_Control_Module_0, true );
    PWM2->SM[kPWM_Module_0].DISMAP[kPWM_Module_0] = 0x00;
    
    PWM_StartTimer( PWM2, kPWM_Control_Module_0 );
}

/******************************************************************************/
/*!   \fn static void lineTimerBurn( void )

      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void lineTimerBurn( void )
{    
    if( engine.numPrintLines != 0 ) {
        /* track where we are history / adjacency */
        engine.burnSequence++;
        /* transfer the data to the print head */
        if(leadInDone == true && engine.linePrintDone == false)
        {
            loadPrintLine();  
        }
        else
        {
            loadZeroPrintLine();
        }
    }
}

/******************************************************************************/
/*!   \fn void lineTimerIsr( void )

      \brief
         This function handles the general purpose line timer interrupts.
        Compare 2 is dual purpose history and pwm start time.


      \author
          Aaron Swift
*******************************************************************************/
#if 0
void lineTimerIsr ( void )
{  
    static bool pwmStartTime_ = false, pwmSltTime_ = false;
    
    /* latch hist/adj load and burn, start pwm to hold line temperature */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag ) & kGPT_OutputCompare1Flag ) == kGPT_OutputCompare1Flag ) {
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );		
        /* setup for pwm start time after history isr */
        if( !pwmStartTime_ ) { 
                
            /* latch control during history */ 
            if( !isCurrentLine() ) {    
  
                    /* latch was set Low in LPSPI_MasterUserCallback(); lets set it back to High now.
                       this should give us around ~150Us pulse width. */ 
                    GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true ); //true
                    
                    /* small delay before we assert the strobe, 
                       according to datasheet should be 100ns min 
                    delay(1);*/
            }

            /* enable the strobe*/
            //GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, false );  
            
            GPIO_WritePinOutput(PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, true);  
            GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, true);

            /* get the current line loaded in the head but don't latch until current line time */
            lineTimerBurn();  
            /* setup for pwm start time */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.pwmStartTime );
            pwmStartTime_ = true;
        } else {
            lineTimerStrobe();
            
            /* latch control during current line. 
               data latch was set low above, lets set it back high now. 
               this should give us enough pulse width */ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true );                      
            pwmStartTime_ = false;
            
            /* setup for history ( next line ) */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.histAdj[0].time );            
        }        
    }  
	
    
    /* latch control during Current line load and burn, slt end */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag ) & kGPT_OutputCompare2Flag ) == kGPT_OutputCompare2Flag ) {
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag );
        if( !pwmSltTime_ ) {
            /* current line data is transfered to the head, assert the data latch LOW*/ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, false );    //false
            /* setup for end of slt time */
             GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, engine.sltTime );
             pwmSltTime_ = true;
        } else { 
            /* end of line burn. reset for next line */
            lineTimerSLT();    
            
            /* reset for current line ( next line ) */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, engine.histAdj[1].time );            
            pwmSltTime_ = false;            
        }
    }
    
	/* half way through line burn (half slt time) */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag ) & kGPT_OutputCompare3Flag ) == kGPT_OutputCompare3Flag ) {     
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag );
        
        if( getDotWearHandle() != NULL && readyToNotify()) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR( getDotWearHandle(), DOT_SAMPLE, eSetBits, &xHigherPriorityTaskWoken );  
        }   
        /* added for debug */
	engine.steps++;			
        //if(getTakingUpPaper() == true)
        //{
        //    halfStepMotor();    //half step TPH and quarter step takeup here
        //}
        //else
        //{
            stepMainMotor();    //half step TPH here
        //}    
    }
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}
#endif

/******************************************************************************/
/*!   \fn void shutdownPrintEngine( void )

      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void shutdownPrintEngine(void)
{
    stopLineTimer();  
}


/******************************************************************************/
/*!   \fn void initializeStepper( StepDir direction  )                                                             
 
      \brief
        This function sets the stepper motor direction. 
        

      \author
          Aaron Swift
*******************************************************************************/ 
void initializeStepper( StepDir direction  )
{    
    initializeMotors( (StepDirM)direction );
}


/******************************************************************************/
/*!   \fn void powerOffStepper( void )                                                             
 
      \brief
        This function turns the stepper motor power off.
        

      \author
          Aaron Swift
*******************************************************************************/ 
void powerOffStepper( void )
{
    //powerOffMotors();
}


/******************************************************************************/
/*!   \fn void powerOnStepper( void )                                                             
 
      \brief
        This function turns the stepper motor power on.  
        

      \author
          Aaron Swift
*******************************************************************************/ 
void powerOnStepper( void )
{
    //powerOnMotors();
}

/******************************************************************************/
/*!   \fn void setStepperDirection( StepDir direction )                                                             
 
      \brief
        This function set the stepper motor direction.
        

      \author
          Aaron Swift
*******************************************************************************/ 
void setStepperDirection( StepDir direction ) 
{
    setMainMotorDirection( (StepDirM)direction );
    /* takeup motor need to run opposite of the main motor */
    if( (StepDirM)direction == FORWARDM_ )
        setTakeUpMotorDirection( BACKWARDM_ );
    else
        setTakeUpMotorDirection( FORWARDM_ );      
}

/******************************************************************************/
/*!   \fn void halfStepMotor( void )                                                             
 
      \brief
        This function half steps the motor  
        

      \author
          Aaron Swift
*******************************************************************************/ 
void halfStepMotor( void )
{    
    stepMotors();
}

/******************************************************************************/
/*!   \fn void motorStep( StepDir dir, PrStatusInfo *pStatus )

      \brief
        This function counts steps of the motor and set status information
        Assumes the motor is going forward (only use while printing)

      \author
          Kurtis Bell
*******************************************************************************/
void motorStep( StepDir dir, PrStatusInfo *pStatus )
{
    if( dir == FORWARD_ ) {
        pStatus->sensor |= MOTOR_FORWARD;
        
        if( stepCounterEnabled_ == true )
            ++pStatus->counter;

        engine.outOfMediaCnt++;

        label0.position++;
        label1.position++;
        label2.position++;
    } else {
        pStatus->sensor &= ~MOTOR_FORWARD;

        if( stepCounterEnabled_ == true )
            --pStatus->counter;
        
        label0.position--;
        label1.position--;
        label2.position--;
    }
  
}

/******************************************************************************/
/*!   \fn void halfStepMotor( void )

      \brief
        This function counts steps of the motor and set status information
        Assumes the motor is going forward (only use while printing)

      \author
          Kurtis Bell
*******************************************************************************/
void motorStepFast( PrStatusInfo *pStatus )
{
    static bool evenSteps_ = false;
    if( evenSteps_ ) {
        /* LinePrinterEngine direction is always FORWARD */
        pStatus->sensor |= MOTOR_FORWARD;

        if( stepCounterEnabled_ == true ) {
            ++pStatus->counter;
        }
        
        engine.outOfMediaCnt++;
        
        label0.position++;
        label1.position++;
        label2.position++;
    } else {
        evenSteps_ = true;
    }
}

/******************************************************************************/
/*!   \fn void initializePrintEngineTimer( void )

      \brief
        This function initializes and starts the printer operation
        timer with the specified period [us]


      \author
          Aaron Swift
*******************************************************************************/
void initializePrintEngineTimer( uint16_t period_us )
{
     gpt_config_t gptConfig; 

    /* obtain default configuration for GPT module */
    GPT_GetDefaultConfig( &gptConfig );
    gptConfig.divider = ENGINE_TIMER_PRESCALE;

    /* initialize FTM module */
    GPT_Init( ENGINE_TIMER_BASE, &gptConfig );

    GPT_SetOutputCompareValue( ENGINE_TIMER_BASE, kGPT_OutputCompare_Channel1, period_us  );
    GPT_EnableInterrupts( ENGINE_TIMER_BASE, kGPT_OutputCompare1InterruptEnable );    

    /* set gpt interrupt priority. */
    NVIC_SetPriority( ENGINE_TIMER_IRQ, ENGINE_TIMER_PRIORITY );
    EnableIRQ( ENGINE_TIMER_IRQ );
    
    /* start the timer  */
    GPT_StartTimer( ENGINE_TIMER_BASE );   
}

/******************************************************************************/
/*!   \fn void setPrintEngineTimerSlt( void )                                                             
 
      \brief
        This function sets the PrinterOperationTimer to SLT time. 
        SLT time is the overall time for one line to print. This is done 
        so that the step_isr or stepUntil_isr move the paper at the same 
        speed as the Line. 
        

      \author
          Aaron Swift
*******************************************************************************/ 
void setPrintEngineTimerSlt( void )
{
    GPT_Deinit( ENGINE_TIMER_BASE );
    initializePrintEngineTimer( engine.sltHalfTime );
}

/******************************************************************************/
/*!   \fn void setPrintEngineTimer( unsigned short time )                                                             
 
      \brief
        This function sets the PrinterOperationTimer to time. 
        This is done so that we can slow the step rate when sizing a label. 
  
      \author
          Aaron Swift
*******************************************************************************/ 
void setPrintEngineTimer( unsigned short time )
{
    GPT_Deinit( ENGINE_TIMER_BASE );
    initializePrintEngineTimer( time );
}

void setEngineContrast( unsigned short contrast )
{
    if( contrast <= MAX_CONTRAST ) {
        engine.contrast = contrast;
    }
    resetTUTorqueControlVars();
}

/******************************************************************************/
/*!   \fn void resetPrintEngineTimer( void )

      \brief
        This function reset the print engine timer back to the 1mS rate.

      \author
          Aaron Swift
*******************************************************************************/
void resetPrintEngineTimer( void )
{
    /* disable Timer before changing values */
    GPT_Deinit( ENGINE_TIMER_BASE );
    initializePrintEngineTimer( DEFAULT_ENGINE_COUNT );
}

/******************************************************************************/
/*!   \fn void resetEngine( void )

      \brief
        This function reset the print engine after it is paused.
        This function is called by the general purpose timer and 
        is not intended to be called directly.                                            

      \author
          Aaron Swift
*******************************************************************************/
void resetEngine( void )
{
    /* setHeadPower( true );  removed for print head engine timing measurements*/
   
    setStepperDirection( FORWARD_ );
    powerOnStepper();
        
    startLineTimer( true );  
    
    engine.pause = false;
}

/******************************************************************************/
/*!   \fn void resetEngine( void )

      \brief
        This function pauses the print engine.

      \author
          Aaron Swift
*******************************************************************************/
void pauseEngine( void )
{  
    engine.pause = true;
    
    stopLineTimer();

    /* so motors do not overheat */
    powerOffStepper();
   
    setHeadPower( false );  
}

/******************************************************************************/
/*!   \fn void initializePrintHeadPwm( void )                                                             
 
      \brief
        This function configures the PWM channel for the print strobe.
                                                          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializePrintHeadPwm( void )
{

    pwm_signal_param_t pwmSignal;
    
 
    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;
    /* strobe signal is active low! pulse width low is measured */
    pwmSignal.dutyCyclePercent = 100 - engine.pwmDutyCycle;
    pwmSignal.deadtimeValue    = 0;
    pwmSignal.faultState       = kPWM_PwmFaultState0;    
    /* disable gpio control of the pin */
    /* select pwm control of the printhead strobe pin */
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 0x10B0U );

    /* period = 39.58uS ==> frequency ~= 25,625 Hz */
    /* duty Cycle updated in start printEngine */
    if( PWM_SetupPwm( PWM2, kPWM_Module_0, &pwmSignal, 1U, kPWM_SignedCenterAligned, 25265U , CLOCK_GetFreq( kCLOCK_IpgClk ) ) != kStatus_Success ) {                       
        PRINTF( "initializePrintHeadPwm(): Error critical print strobe pwm failure! \r\n" );                        
    }
}

/******************************************************************************/
/*!   \fn PrintEngine *getPrintEngine( void )                                                             
 
      \brief
        This function returns a pointer to the print engine. 
                                                          
      \author
          Aaron Swift
*******************************************************************************/ 
PrintEngine *getPrintEngine( void )
{
    return &engine;
}

/******************************************************************************/
/*!   \fn bool isEnginePaused( void )                                                             
 
      \brief
        This function returns . 
                                                          
      \author
          Aaron Swift
*******************************************************************************/ 
bool isEnginePaused()
{
    if(engine.pause) {
        engine.pause = false;  
        return true;
    } else {
        return false;
    }
}

/******************************************************************************/
/*!   \fn unsigned long getPrintEngineLineCntr( void )                                                     
 
      \brief
        This function returns the number of processed print lines. 
                                                          
      \author
          Aaron Swift
*******************************************************************************/                             
unsigned long getPrintEngineLineCntr( void )
{
    return  engine.lineCounter;
}

/******************************************************************************/
/*!   \fn static void printEngineISR( void )

      \brief
        This function is the flex timer interrupt handler used for the 
        print engine. This timer executes at a rate of 400uS or 2.5Khz.
 
      \author
          Aaron Swift
*******************************************************************************/
void printEngineISR( void ) 
{       
    #if 0   /* added for debugging printEngine */
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    #endif
    
    if( ( GPT_GetStatusFlags( ENGINE_TIMER_BASE, kGPT_OutputCompare1Flag ) & 
          kGPT_OutputCompare1Flag ) == kGPT_OutputCompare1Flag ) {
  
        /* no thread running in auto mode to update the sensor info */    
        if( isADCAutoMode() ) {
            readADChannels();    
        }

        /* call the current operational command */ 
        switch( engine.currentCmd.generic.directive )
        {              
            case IDLE_DIRECTIVE: {
                idleOp();
                break;
            }
            case PRINT_DIRECTIVE: { 
                printOp( &engine.currentCmd );
                break;
            }
            case STEP_DIRECTIVE: { 
                stepOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case STEP_UNTIL_DIRECTIVE: {
                stepUntilOp( (StepUntilOperation *)&engine.currentCmd );                       
                break;
            }
            case WAIT_DIRECTIVE: {         
                waitOp( (WaitOperation *)&engine.currentCmd );              
                break;
            }
            case WAIT_UNTIL_DIRECTIVE: {            
                waitUntilOp( (WaitUntilOperation *)&engine.currentCmd );
                break;
            }
            case TEST_DIRECTIVE: {   
                testOp( (TestOperation *)&engine.currentCmd );
                break;
            }
            case STATUS_DIRECTIVE: { 
                statusOp( (StatusOperation *)&engine.currentCmd );
                break;
            }
            case COUNTER_DIRECTIVE: {            
                counterOp( (CounterOperation *)&engine.currentCmd );
                break;
            }
            case DISABLE_DIRECTIVE: {          
                disableOp( &engine.currentCmd );
                break;
            }
            case CALIBRATE_DIRECTIVE: {            
                calibrateOp( &engine.currentCmd );
                break;
            }
            case CALIBRATE_TU_DIRECTIVE: {            
                calibrateTUOp( &engine.currentCmd );
                break;
            }
            case NOOPERATION_DIRECTIVE: {
                break;
            }
            case CUT_DIRECTIVE:{
                cutOp( (GenericOperation *)&engine.currentCmd );
                break;
            }
            case HEAD_TEST_DIRECTIVE: {
                dotWearOp( &engine.currentCmd );
                break;
            }
            case HEAD_RESISTANCE_CAL_DIRECTIVE: {
                dotWearCalOp( &engine.currentCmd );
                break;
            }
            case VIRTUAL_CUT_DIRECTIVE: {
                stepOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case STEP_GAP_DIRECTIVE: {
                stepGapOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            case STEP_EDGE_DIRECTIVE: {
                stepEdgeOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            case TEST_FOR_SYNC: {  
                testForSyncOp( (StepOperation *)&engine.currentCmd ); 
                break;
            }
            case TEST_FOR_LABEL: {
                testForLabelOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case TEST_FOR_CONTINUOUS: {
                testForContinuous( (StepOperation *)&engine.currentCmd );
                break;
            }
            case WAIT_UNTIL_SIZING: {
                waitUntilSizingOp( (WaitUntilOperation *)&engine.currentCmd );
                break;
            }
            case STEP_TAKEUP_DIRECTIVE: {
                stepTakeupOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            case STEP_TAKEUP_TIGHTEN: {
                stepTakeupTightenOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case DETECTION_STEP_UNTIL: {
                detectionOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            default:
                break;
          
        }
    }

    GPT_ClearStatusFlags( ENGINE_TIMER_BASE,  kGPT_OutputCompare1Flag );   

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;   
    
    #if 0   /* added for debugging printEngine */    
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
    #endif    
}

/******************************************************************************/
/*!   \fn void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevoius )

      \brief
        This function compare the current printer status with the prevoius


      \author
          Aaron Swift
*******************************************************************************/
void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevious)
{
     #if 0  //TFinkQueueSetFix    //Test Debug only!
     pCurrent->mask.sensor = 0x02;  //TFinkToDo! Don't leave this hanging here
     #endif

    /* set error bits to history */
    pCurrent->history |= pCurrent->error;
    if( (pPrevious->error != pCurrent->error) ||
      ( pPrevious->state != pCurrent->state ) ||
      ( pPrevious->command != pCurrent->command ) ||  
    ( ( pPrevious->sensor & pCurrent->mask.sensor ) != ( pCurrent->sensor & pCurrent->mask.sensor ) ) ||
    ( ( pPrevious->sensor2 & pCurrent->mask.sensor2 ) != ( pCurrent->sensor2 & pCurrent->mask.sensor2 ) ) ||
    ( ( pPrevious->user & pCurrent->mask.user) != ( pCurrent->user & pCurrent->mask.user ) ) ) {
      

        #if 0
        PRINTF("\r\n");
        if( pPrevious->error != pCurrent->error ) { 
            PRINTF("compareStatus(): pPrevious->error: %d\r\n", pPrevious->error ); 
            PRINTF("compareStatus(): pCurrent->error: %d\r\n", pCurrent->error );  
        }
        if( pPrevious->state != pCurrent->state ) {
            PRINTF("compareStatus(): pPrevious->state: %d\r\n", pPrevious->state);  
            PRINTF("compareStatus(): pCurrent->state: %d\r\n", pCurrent->state);  
        }
        if( ( pPrevious->sensor & pCurrent->mask.sensor ) != ( pCurrent->sensor & pCurrent->mask.sensor ) ) {
            PRINTF("compareStatus(): pPrevious->sensor: %d\r\n", pPrevious->sensor );  
            PRINTF("compareStatus(): pCurrent->sensor: %d\r\n", pCurrent->sensor );  
        }
        if(( pPrevious->sensor2 & pCurrent->mask.sensor2 ) != ( pCurrent->sensor2 & pCurrent->mask.sensor2 )) {
            PRINTF("compareStatus(): pPrevious->sensor2: %d\r\n", pPrevious->sensor2 );  
            PRINTF("compareStatus(): pCurrent->sensor2: %d\r\n", pCurrent->sensor2 );  
        }
        if(( pPrevious->user & pCurrent->mask.user) != ( pCurrent->user & pCurrent->mask.user )) {        
            PRINTF("compareStatus(): pPrevious->user: %d\r\n", pPrevious->user );  
            PRINTF("compareStatus(): pCurrent->user: %d\r\n", pCurrent->user );            
        }        
        #endif
                
        sendPrStatus( pCurrent, true );
        
        if( ( pCurrent->sensor2 & OUT_OF_DATA_BUFFERS ) == OUT_OF_DATA_BUFFERS ) {
            //PRINTF("compareStatus(): sending out of data buffers\r\n");  
            pCurrent->sensor2 &= ~OUT_OF_DATA_BUFFERS;
            pPrevious->sensor2 &= ~OUT_OF_DATA_BUFFERS;
        }

		if( ( pCurrent->sensor2 & OUT_OF_DATA_BUFFERS2 ) == OUT_OF_DATA_BUFFERS2 ) {
            //PRINTF("compareStatus(): sending out of data buffers\r\n");  
            pCurrent->sensor2 &= ~OUT_OF_DATA_BUFFERS2;
            pPrevious->sensor2 &= ~OUT_OF_DATA_BUFFERS2;
        }
        
        /* set the prevoius to the current */
        memcpy( pPrevious, pCurrent, sizeof(PrStatusInfo) );
        /* SOF-5976 */
       //TFinkQueueSetFix sendPrStatus( pCurrent, true );   
    }   
}

/******************************************************************************/
/*!   \fn bool testCondition( PrStatusInfo *pStatus, TestOperator oper, 
                              unsigned char bits, unsigned char result )

      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
bool testCondition( PrStatusInfo *pStatus, TestOperator oper, unsigned char bits, 
                    unsigned char result )
{
    bool r = false; 
    if( oper == BITS_EQUAL ) {
        if( ( pStatus->sensor & bits ) == bits ) {
            r = true;
        }     
    } else if( oper == BITS_NOT_EQUAL  ) {
        if( ( pStatus->sensor & bits )!= bits ) {
            r = true;
        }          
    } else {  
        PRINTF("testCondition(): Error: Unknown compare operation!\r\n" ); 
    }
    return r;
}

/******************************************************************************/
/*!   \fn void calibratePrinter( PrinterCal cal )
                             
      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void calibratePrinter( PrinterCal cal )
{ 
	/* Only calibrate if engine is in IDLE state */
	if( currentStatus.state == ENGINE_IDLE )
	{
		currentStatus.command = CALIBRATE_COMMAND;   
		//engine.currentCmd.generic.directive = CALIBRATE_DIRECTIVE;

		if( cal == MEDIA_SENSOR_CALIBRATION ) { 
			setOperation( CALIBRATE_DIRECTIVE, &currentStatus );        
		} 
		else if( cal == TU_SENSOR_CALIBRATION ) { 
			
			/* 
			*	Cal buffers should be NULL, but just incase verify. Could be that
			*	previous Cal failed and buffers didn't get released.
			*/
			freePrinterCalBuffers();
			
			if( initTUMotorCal() == true )
			{
				/* allocate memory in the printEngine to hold min max cal tension values */
				engine.TUCalMinTensionVals 		= pvPortMalloc((CC_TWENTY_FIVE_POINT_FIVE+1) * sizeof(unsigned short));	
				engine.TUCalMaxTensionVals 		= pvPortMalloc((CC_TWENTY_FIVE_POINT_FIVE+1) * sizeof(unsigned short));	
				engine.TUCalDeltaTensionVals 	= pvPortMalloc((CC_TWENTY_FIVE_POINT_FIVE+1) * sizeof(unsigned short));	
				
				if( engine.TUCalMinTensionVals 		!= NULL		&& 
				    engine.TUCalMaxTensionVals 		!= NULL		&&
					engine.TUCalDeltaTensionVals 	!= NULL )
				{
					setOperation( CALIBRATE_TU_DIRECTIVE, &currentStatus );        
				}
				else
				{
					PRINTF("calibratePrinter(): failed to allocate printEngine CAL Arrays !\r\n" ); 
					freePrinterCalBuffers();
				}
				
			}
			else
			{
				PRINTF("calibratePrinter(): failed to initTUMotorCal() !\r\n" ); 
			}
				
		} 
	}
	else
	{  
        PRINTF("calibratePrinter(): engine is not in IDLE state! \t%d!\r\n", currentStatus.state ); 
		PrGapCalStatus calStatus;
		
		/* Let Host application know we failed to start */
		memset(&calStatus, 0, sizeof(calStatus) );
		
		calStatus.state 				= _TakeupCalFailure;
		calStatus.deflectionVoltage		= (unsigned short)PREPARE_MEDIA_TU_CAL_START;//debug

		sendPrTUCalStatus( &calStatus );
    }
}

/******************************************************************************/
/*!   \fn createCheckerBoardLabel(  unsigned char offset, unsigned long length )
                             
      \brief
        This function blits the checkerboard test pattern into the 
        printer image buffer.

      \author
          Aaron Swift
*******************************************************************************/
void createCheckerBoardLabel( unsigned char offset, unsigned long length )
{    
#if 1   /* make like cm4 / k64 checkerboard image */
    PRINTF("\r\n\r\n-----CREATING CHECKERBOARD LABEL LOCAL------\r\n\r\n");
    //unsigned short rowByteWidth = getHeadStyleSize() / 8;
    unsigned short rowByteWidth = PRINTER_HEAD_SIZE_80MM; /* was 72 */
    unsigned short centeringOffset = 0;
    unsigned char bmpPitch = BMP_PITCH;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    //unsigned short leftMostBit = centeringOffset + EDGE_MARGIN; 
    unsigned short leftMostBit = 0;
    unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - (8 * 2);
    //unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
    

    /* our buffer is only big enough for a 3" label*/
    if( length <= STEPS_PER_LENGTH3_00 ) {
      
        /* right most bit position cannot be bigger then the print head 
        if( rightMostBit > getHeadStyleSize() ) {
                rightMostBit = getHeadStyleSize();            
        }*/
        /* clear our pattern buffers  */
        //memset( &pattern1[0], '\0', ( PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        //memset( &pattern2[0], '\0', ( PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        /*
        memset( &pattern1[0], '\0', ( 72 * 2 ) ); 
        memset( &pattern2[0], '\0', ( 72 * 2 ) );  */

        /* create our first bitmap pattern */
        unsigned short i = leftMostBit;
        while( i < rightMostBit ) {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;                
                //bitSet( i, iMax - i, &pattern1[0] );
                i = i + 2 * bmpPitch;
        }
        
        /* create our second bitmap pattern */
        i = leftMostBit + bmpPitch;
        while (i < rightMostBit)
        {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;               
                //bitSet( i, iMax - i, &pattern2[0] );
                i = i + 2 * bmpPitch;
        }
                    
        unsigned char *pImage = getImageBuffer();

        unsigned long rowCount = offset;
        bool isPattern1 = true;
        /* copy the test patterns to the image buffer */
        while( rowCount < length ) {
            if( isPattern1 ) {
                    //memcpy( ( pImage + rowCount * rowByteWidth ), &pattern1[0], rowByteWidth );
            } else {
                    //memcpy(( pImage + rowCount * rowByteWidth ), &pattern2[0], rowByteWidth );
            }
            rowCount++;

            if ( ( rowCount % 32 ) == 0 ) {       // bmpPitch
                isPattern1 = !isPattern1;
            }
        }

        /* clear our pattern buffers */
        //memset( &pattern1[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        //memset( &pattern2[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        /*
        memset( &pattern1[0], '\0', ( 72 * 2 ) ); 
        memset( &pattern2[0], '\0', ( 72 * 2 ) );  */
        
    } else {
        PRINTF("createCheckerBoardLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }    
#else    
    unsigned short rowByteWidth = getHeadStyleSize() / 8;
    unsigned short centeringOffset = 0;
    unsigned char bmpPitch = BMP_PITCH;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    unsigned short leftMostBit = centeringOffset + EDGE_MARGIN; 
    unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
      
    /* our buffer is only big enough for a 5" label*/
    if( length <= STEPS_PER_LENGTH5_00 ) {
      
        /* right most bit position cannot be bigger then the print head */
        if( rightMostBit > getHeadStyleSize() ) {
                rightMostBit = getHeadStyleSize();            
        }
        /* clear our pattern buffers */
        memset( &pattern1[0], '\0', ( PRINTER_HEAD_SIZE_72MM * 2 ) ); 
        memset( &pattern2[0], '\0', ( PRINTER_HEAD_SIZE_72MM * 2 ) ); 

        /* create our first bitmap pattern */
        unsigned short i = leftMostBit;
        while( i < rightMostBit ) {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;                
                bitSet( i, iMax - i, &pattern1[0] );
                i = i + 2 * bmpPitch;
        }
        
        /* create our second bitmap pattern */
        i = leftMostBit + bmpPitch;
        while (i < rightMostBit)
        {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;               
                bitSet( i, iMax - i, &pattern2[0] );
                i = i + 2 * bmpPitch;
        }
                    
        unsigned char *pImage = getImageBuffer();

        unsigned long rowCount = offset;
        bool isPattern1 = true;
        /* copy the test patterns to the image buffer */
        while( rowCount < length ) {
            if( isPattern1 ) {
                    memcpy( ( pImage + rowCount * rowByteWidth ), &pattern1[0], rowByteWidth );
            } else {
                    memcpy(( pImage + rowCount * rowByteWidth ), &pattern2[0], rowByteWidth );
            }
            rowCount++;

            if ( ( rowCount % 32 ) == 0 ) {       //bmpPitch
                isPattern1 = !isPattern1;
            }
        }

        /* clear our pattern buffers */
        memset( &pattern1[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        memset( &pattern2[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
    } else {
        PRINTF("createCheckerBoardLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }
#endif    
}

/******************************************************************************/
/*!   \fn createVerticalLinesLabel(  unsigned char offset, unsigned long length )
                             
      \brief
        This function blits the vertical lines test pattern into the 
        printer image buffer.


      \author
          Aaron Swift
*******************************************************************************/
void createVerticalLinesLabel(  unsigned char offset, unsigned long length )
{	
    unsigned short pattern[8] = { 0xC0, 0xE0, 0xC0, 0xE0, 0xC0, 0xE0, 0xC0, 0xE0 };
    unsigned char *pImage = getImageBuffer();
    unsigned short centeringOffset = 0;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
        
    unsigned long rowByteWidth = PRINTER_HEAD_SIZE_72MM;
    
    unsigned long leftMarginByte =  ( centeringOffset + EDGE_MARGIN ) / 8;
    unsigned long rightMarginByte = ( centeringOffset + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN ) / 8;

    /* our buffer is only big enough for a 4" label*/
    if( length <= STEPS_PER_LENGTH4_00 ) {

        unsigned long rowCount = offset;
        while( rowCount < length ) {
                
                unsigned long startByte = leftMarginByte + ( rowCount * rowByteWidth );
                unsigned long lastByte = rightMarginByte + ( rowCount * rowByteWidth );

                for( unsigned long i = startByte; i < lastByte; i++ ) {
                    pImage[i] = (unsigned char)pattern[i % 8];
                }
                rowCount++;
        }
    } else {
        PRINTF("createVerticalLinesLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }
}

/******************************************************************************/
/*!   \fn createSingleVerticalLineLabel(  unsigned char offset, unsigned long length )
                             
      \brief
        This function blits the vertical lines test pattern into the 
        printer image buffer.


      \author
          Aaron Swift
*******************************************************************************/
void createSingleVerticalLineLabel(  unsigned char offset, unsigned long length )
{	
   
    unsigned char *pImage = getImageBuffer();
    unsigned long rowCount = offset;
    /* our buffer is only big enough for a 4" label*/
    if( length <= STEPS_PER_LENGTH4_00 ) {

        //unsigned long rowCount = offset;
        while( rowCount < length ) {
                
                //unsigned long startByte = leftMarginByte + ( rowCount * rowByteWidth );
                //unsigned long lastByte = rightMarginByte + ( rowCount * rowByteWidth );

                //for( unsigned long i = startByte; i < lastByte; i++ ) {
                    pImage[ rowCount * PRINTER_HEAD_SIZE_72MM ] = 0x10;
                //}
                rowCount++;
        }
    } else {
        PRINTF("createVerticalLinesLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }
}

/******************************************************************************/
/*!   \fn createHorizontalLinesLabel( unsigned long length )
                             
      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void createHorizontalLinesLabel( unsigned char offset, unsigned long length )
{
    unsigned short centeringOffset = 0;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    unsigned short leftMostBit = centeringOffset + EDGE_MARGIN;
    unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
    
    /* right most bit position cannot be bigger then the print head */
    if( rightMostBit > getHeadStyleSize() ) {
        rightMostBit = getHeadStyleSize();
    }

    unsigned long barHeight = BAR_HEIGHT;
    unsigned long blankLineHeight = BAR_HEIGHT * 2;
    unsigned char *pImage = getImageBuffer();
    unsigned long lineWidthBytes = PRINTER_HEAD_SIZE_72MM;
    unsigned long rowCount = offset ;
    /* our buffer is only big enough for a 4" label*/
    if( length <= STEPS_PER_LENGTH4_00 ) {
    
        while( rowCount < length ) {
            for( unsigned long j = rowCount; ( (j < ( rowCount + barHeight ) ) && ( j < (length) ) ); j++ ) {
                    bitSet( leftMostBit, rightMostBit - leftMostBit, &pImage[j * lineWidthBytes] );
            }
            rowCount += ( barHeight + blankLineHeight );

            /* increase the size of the white space as we move down the label */
            blankLineHeight++;
            /* increase the size of the black bar as we move down the label */
            barHeight += 2;
        }  
    } else {
        PRINTF("createHorizontalLinesLabel(): Warning: label length too long for test pattern!\r\n" );     
    }
}

/******************************************************************************/
/*!   \fn void bitSet( unsigned short startBit, unsigned short numBits, 
                                                unsigned char *pBuffer )

      \brief
        This function sets bits in a buffer 

      \author
          Aaron Swift
*******************************************************************************/
void bitSet( unsigned short startBit, unsigned short numBits, unsigned char *pBuffer )
{
    unsigned char  mask = 0x80;
    unsigned char  bitsPerMask = 8;

     
    unsigned short firstFullByte, lastPartialByte;
    unsigned short firstFullBit , lastPartialBit;
    
    /* determine first full byte in the line */
    firstFullByte = startBit / bitsPerMask;

    if (startBit % bitsPerMask)
        firstFullByte += 1;
    
    /* very first bit in the very first full byte */
    firstFullBit = firstFullByte * bitsPerMask;	
 
    lastPartialByte = (startBit + numBits) / bitsPerMask;
    lastPartialBit = lastPartialByte * bitsPerMask;

    /* loop through the first dangling bits. there will be anywhere from 1 to 8 */
    for( unsigned short ii = startBit; ii < firstFullBit; ii++ ) {
        pBuffer[ii / bitsPerMask] |= ( mask >> ( ii % bitsPerMask ) );
    }

    if( firstFullByte < lastPartialByte ) {
        /* do all the full bytes one at a time. */
        for( unsigned short ii = firstFullByte; ii < lastPartialByte; ii++ ) {
            pBuffer[ii] = 0xFF;
        }

        /* Do the last few dangling bits. Might also be 1 - 8 of these */
        for( unsigned short ii = lastPartialBit; ii < (startBit + numBits); ii++ ) {
            pBuffer[ii / bitsPerMask] |= ( mask >> ( ii % bitsPerMask ) );
        }
    }  
}

/******************************************************************************/
/*!   \fn void delay( unsigned long )

      \brief
        This function delays in uSeconds. Please use this function when no 
        other method is ( oneshot timer )available.
      \author
          Aaron Swift
*******************************************************************************/
void delay( unsigned long time )
{
    unsigned long i, j, x;
    for( i = 0; i < time; i++ ) {
        for( j = 0; j < USEC_DELAY_COUNT; j++ ) {
            x += 1;
        }  
    }
}

/******************************************************************************/
/*!   \fn void idleOp( void )

      \brief
        This function handles the teach table idle command operation.

      \author
          Aaron Swift
*******************************************************************************/
void idleOp( void )
{  
    if((getGapCalStatus() == true || getTUCalStatus() == true) && getTUSlip() == false )
    {
        powerOnMotorsDuringCal();
    }
    else
    {
        powerOffMotors();
    }
  
    /* if this is the first time thru, do some initialization.*/
    if( currentStatus.state != ENGINE_IDLE )
    {   
       // PRINTF("currentStatus.state: Entering ENGINE_IDLE\r\n" );
      
        currentStatus.state = ENGINE_IDLE;
        /*previous command is complete. */
        currentStatus.command |= COMMAND_COMPLETE;
        /* clear our flag if set */
        currentStatus.sensor2 &= ~OUT_OF_DATA_BUFFERS; 
        
        setPrintingStatus(false);
        
        /* sample the shoot through sensor 100 times, if 90% of the samples are less than 
           OUT_OF_MEDIA_THRESHOLD, OUT_OF_MEDIA is set*/
        checkForOutOfMedia();
        
        /* notify the host the printer is idle. */        
        compareStatus( &currentStatus, &prevStatus );
        
        engine.steps = 0;
        engine.stepsOffset = 0;
        setHeadTimings();
        powerOffMotors();		
		
    } 
    else 
    {    
        //PRINTF("LT: %d\r\n", getLabelTaken());
      
        if(getLabelQueuePaused() == true)
        {
            labelPauseTimeout++;
            
            if(labelPauseTimeout >= 1000)
            {
                labelPauseTimeout = 0;
                setLabelQueuePaused(false);
                setLabelPauseBackwindPending(true);
                setStartOfQueue(true);
                
                setHalfStepMode(_MAIN_STEPPER);
                setHalfStepMode(_TAKEUP_STEPPER);
                if(getTakingUpPaper() == false)
                {
                    if(getUsingContinuous() == true || getCutterInstalled_() == true)
                    {
                        __NOP();
                    }
                    else if(getLargeGapFlag() == true)
                    {
                        stepToNextLabel(210 + getIndirectData(4), 1000);
                    }
                    else
                    {
                        stepToNextLabel(160 + getIndirectData(4), 1000);
                    }
                } 
            }
        }
        
        
        if(getBackwindAfterSizing() == true && getLabelTaken() <= LABEL_TAKEN_THRESHOLD_NO_LABEL && getTUCalStatus() == false)
        {          
            setHalfStepMode(_MAIN_STEPPER);
            setHalfStepMode(_TAKEUP_STEPPER);
          
            setTakeUpMotorDirection( FORWARDM_ ); 
            setMainMotorDirection( BACKWARDM_ );
          
            setBackwindAfterSizing(false);
  
            if(getCutterInstalled_() == true)
            {
                __NOP();                
            }
            else
            {
                if(getUsingContinuous() == true)
                {
                    if(getTakingUpPaper() == true)
                    {
                        backwindStock(100, 1000);
                    }
                }
                else
                {
                    backwindStock(calculateSizingBackwindSteps(), 1000);
                }              
            }
        }
        
        
        /* send requested changes in printer status to the controller. */
        compareStatus( &currentStatus, &prevStatus );

        if( pCmdQHandler_ != NULL ) {
            /* check for commands which may have been issued by the system controller. */
            PrCommand cmdMsg;
            
            int numMgs = uxQueueMessagesWaitingFromISR( pCmdQHandler_ );
            if( numMgs ) {
                if( xQueueReceiveFromISR( pCmdQHandler_, &cmdMsg, 0 ) ) {             
                    /* perform some initialization. */
                    currentStatus.command &= ~COMMAND_COMPLETE;

                    /* set the commad data */
                    setIndirectData( (CMD_DATA_IDS)cmdMsg.data_item, cmdMsg.value );
                    /* intialize the command list */
                    
                    initializeCmdSequence( cmdMsg.identifier, &currentStatus ); 

                } else {
                    /* timing values change with contrast setting and temperature */
                    setHeadTimings();
                }
            }
        } else { 
            PRINTF("idleOp(): Error: pCmdQHandler_ is null!\r\n" );  
        }
    }
}

/******************************************************************************/
/*!   \fn void ( CmdOp *pOperation )

      \brief
        This function handles the teach table print command operation.


      \author
          Aaron Swift
*******************************************************************************/
void printOp( CmdOp *pOperation )
{
    /* added to debug drift and correction */
    static bool onceGap_ = false; 
    static bool onceTaken_ = false;
    static bool ltOnce_ = false;
    static bool expelDist_ = false;
    static bool roll_      = false;
    static int cnt_ = 0, rollTicks_ = 0;
    static bool rcrd_ = false;
    static unsigned long printLines = 0;
    
    unsigned long half_image_buffer_line_count = 0;
    
    unsigned short expelSteps = (getIndirectData( 4 ));
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        printLines						= N_PRINTER_LINES_72MM;
		half_image_buffer_line_count 	= HALF_IMAGE_BUFFER_LINE_COUNT_72MM;
    }
    else
    {
        printLines						= N_PRINTER_LINES_80MM;
		half_image_buffer_line_count 	= HALF_IMAGE_BUFFER_LINE_COUNT_80MM;
    }
   
    /* clear if there was an error */
    currentStatus.error &= ~OUT_OF_PRINTDATA; 
    
        
    if( currentStatus.state != ENGINE_PRINTING ) 
    {
        //PRINTF("currentStatus.state: Entering ENGINE_PRINTING!\r\n" );
        
        powerOnStepper();
        
        if( engine.headType == UNKNOWN_HEAD ) 
        {
            currentStatus.error |= UNKNOWN_PH;
        } 
        else
        {
            currentStatus.error &= ~UNKNOWN_PH;
        }
        
        if(getTakeupBusy() == true)
        {
            takeupBusyTimeout++;

            if(takeupBusyTimeout >= 1500)
            {
                //PRINTF("\r\ntakeup busy timeout hit in print op");
              
                setTakeupBusy(false);
                takeupBusyTimeout = 0;
            }
        }

        if( currentStatus.error == NO_ERROR && getTakeupBusy() == false && backwindAfterSizing == true) 
        {
            takeupBusyTimeout = 0;

            setHalfStepMode(_MAIN_STEPPER);
            setHalfStepMode(_TAKEUP_STEPPER);

            if(getCutterInstalled_() == false)
            {
                backwindStock(calculateSizingBackwindSteps(), 1000);
            }
            
            backwindAfterSizing = false;
        }

        if(getTakingUpPaper() == true)
        {
            if( currentStatus.error == NO_ERROR && getTakeupBusy() == false && getLabelTaken() <= LABEL_TAKEN_THRESHOLD_NO_LABEL ) 
            {
                currentStatus.state = ENGINE_PRINTING;
                
                shootCounts = getShootThroughBuffer();
                
                takeupBusyTimeout = 0;

                engine.numPrintLines = getOpData( pOperation->print.type, pOperation->print.data );
                engine.direction = ( engine.numPrintLines < 0 ) ? BACKWARD_ : FORWARD_;
                engine.numPrintLines = abs( engine.numPrintLines );
                engine.totalLinesToPrint = engine.numPrintLines;
                engine.labelOrientation = pOperation->print.orientation;
                engine.steps = 0;

                startPrintEngine();
            }
        }
        else
        {
            if( currentStatus.error == NO_ERROR && getTakeupBusy() == false ) 
            {
                currentStatus.state = ENGINE_PRINTING;
                
                shootCounts = getShootThroughBuffer();
                
                takeupBusyTimeout = 0;

                engine.numPrintLines = getOpData( pOperation->print.type, pOperation->print.data );
                engine.direction = ( engine.numPrintLines < 0 ) ? BACKWARD_ : FORWARD_;
                engine.numPrintLines = abs( engine.numPrintLines );
                engine.totalLinesToPrint = engine.numPrintLines;
                engine.labelOrientation = pOperation->print.orientation;
                engine.steps = 0;
                
                startPrintEngine();
            }
        }
   } 
   else 
   {
        if(getLabelSizeInQuarterSteps() <= 750 /*&& getLargeGapFlag() == false*/)
        {
            if(getReadyToRecordShootVal() == true && shootIndex < 300)
            {
                setReadyToRecordShootVal(false);
                
                updateLowLabelStatus();

                if(shootIndex < SHOOT_COUNT_ARRAY_SIZE)
                {
                    shootCounts[shootIndex] = pollMediaCounts();
                    
                    
                    if(shootCounts[shootIndex] > (config_.backingAndlabel * 1.08))
                    {
                        shootCounts[shootIndex] = (config_.backingAndlabel * 1.08);
                    }
                }
                
                shootIndex++;
            }
        }
        else
        {
            if(getReadyToRecordShootVal() == true)
            {
                setReadyToRecordShootVal(false);
                
                updateLowLabelStatus();

                if(shootIndex < SHOOT_COUNT_ARRAY_SIZE)
                {
                    shootCounts[shootIndex] = pollMediaCounts();
                    
                    
                    if(shootCounts[shootIndex] > (config_.backingAndlabel * 1.08))
                    {
                        shootCounts[shootIndex] = (config_.backingAndlabel * 1.08);
                    }
                }

                shootIndex++;
            }
        }
        
        /* are we printing a label bigger than our image buffer? */
        if( engine.totalLinesToPrint > printLines ) 
        {
  
            /* Once we use the first half of the image buffer tell the host so we can get more image data */
			if( ( engine.lineCounter == half_image_buffer_line_count && !paused_ ) ) 
                        {   
                          
                //PRINTF("\r\nout of buffers 1\r\n");       
                
                /* notify the host that we're finished with the first half of the print buffer */
                currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS; 
                compareStatus( &currentStatus, &prevStatus );
                paused_ = true;
#if 0
				GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
#endif	
				
            }

			/*
			*	Have we used the second half of the image buffer?... before we ask for more data,
			*	make sure that we actually need it.
			*/
            if( engine.lineCounter == printLines && (engine.numPrintLines > half_image_buffer_line_count) )  
            {
                
				// notify host that we want image data to fill the second half (Buffer2)
				currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS; 
				currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS2; 

                                //PRINTF("\r\nout of buffers 2\r\n"); 
                                
				//myEngineLineCounterAtRollover++;
#if 0
				GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true);
#endif	
					
				compareStatus( &currentStatus, &prevStatus );
                engine.lineCounter = 0;

            }  
            
        }
		
        
        if( engine.linePrintDone == true && expelDone == true && getTPHIntrDone() == true && getTakeupBusy() == false) //print complete
        {                         
            paused_ = false;
            resetPrintDataLine();
            
            firstPrint = false;
            
            
           
            if( continuousStock_ )  //continuous stock print complete
            {

                if(getCutterInstalled_() == true)
                {
                    ICMessages msg1;
                    ICMessages msg2;
                    
                    msg1.generic.msgType = _I_CUTTER_CUT_CMD;
                    msg2.generic.msgType = _I_CUTTER_HOME_CMD;
                    
                    //takeupDelay();

                    handleInternalMessage(&msg1);
                }
                
                compareStatus( &currentStatus, &prevStatus );
                setNextOperation(&currentStatus);
            }
            else //die cut print complete  
            {        
                if(getTakingUpPaper() == true)
                {
                    /*
                    short* shoots = getShootThroughBuffer();
            
                    PRINTF("\r\n");
                    PRINTF("shoot through counts post filter:");
                    PRINTF("\r\n");
                    
                    for(uint16_t ind = 0; ind < shootIndex; ind++)
                    {
                        PRINTF("%d,", shoots[ind]);
                        takeupDelayShort();
                    }
                    
                    PRINTF("\r\n");
                    */

                    compareStatus( &currentStatus, &prevStatus );
                    setNextOperation(&currentStatus);
                } 
                else
                {      
                    //find gap
                    short* shoots = getShootThroughBuffer();
                  
                    int startFilterIdx = 0;
                    int endFilterIdx = 24;
                    
                    while(endFilterIdx < (shootIndex))
                    {
                        averageAndStore(shoots, startFilterIdx, endFilterIdx);
                        startFilterIdx++;
                        endFilterIdx++;
                    }
                  
                    double desiredPercentage;
                    
                    desiredPercentage = 85;
                    
                    double result = find_percentage_of_average(shoots, shootIndex, desiredPercentage);

                    find_lowest_points_lowest(shoots, shootIndex, result);
                    
                    TPHStepsPastGapThisPrint = ( getTPHStepsThisPrint() - getPrintDip() );
                    
                    if(TPHStepsPastGapThisPrint >= 600)
                    {
                        TPHStepsPastGapThisPrint = 600;
                    }
                    
                    if(getPrintDip() < 10)
                    {
                        TPHStepsPastGapThisPrint = 400;
                    }
                    
                    /*
                    PRINTF("\r\n");
                    PRINTF("shoot through counts post filter:");
                    PRINTF("\r\n");
                    
                    for(uint16_t ind = 0; ind < shootIndex; ind++)
                    {
                        PRINTF("%d,", shoots[ind]);
                        takeupDelayShort();
                    }
                    
                    PRINTF("\r\n");
                    */
                    
                    shootIndex = 0;
                    memset(shoots, 0, sizeof(&shoots));
                    
                    setPrintingStatus(false);
                    
                    //if the command option == 2 goto waitUntil, else goto idle
                    if(getWaitForLabelTaken() == false)
                    {
                        int calcSteps = getTPHStepsPastGapThisPrint(); 
                  
                        int modifier = 0;
                        
                        if(getLargeGapFlag() == true)
                        {
                            modifier = 330;
                        }
                        else
                        {
                            modifier = 345;
                        }
                        
                        if(calcSteps != 0)
                        {
                            if(calcSteps > modifier) 
                            {
                                calcSteps = calcSteps - modifier;
                            }
                            else
                            {
                                calcSteps = modifier - calcSteps;
                            }
                        }
                        
                        setStreamingLeadInMod(calcSteps);
                      
                        skipNextOperation( &currentStatus ); 
                        skipNextOperation( &currentStatus ); 
                        skipNextOperation( &currentStatus ); 
                        skipNextOperation( &currentStatus );
                    }
                    else
                    {                        
                        setStreamingLeadInMod(0);
                        
                        setStartOfQueue(true);

                        setNextOperation(&currentStatus);
                    }
                } 
            }   
        } 
    }    
}

/******************************************************************************/
/*!   \fn void printDotWear( CmdOp *pOperation )

      \brief
        This function handles the teach table Dot Wear test command operation.
        Starts the print engine but does not step.

      \author
          Aaron Swift
*******************************************************************************/
void printDotWearOp( CmdOp *pOperation )
{
    if( currentStatus.state != ENGINE_PRINTING ) {
        PRINTF("currentStatus.state: Entering ENGINE_DOT_PRINTING!\r\n" );  
        
        currentStatus.state = ENGINE_PRINTING;
        /* initialize the printer control parameters. */      
        engine.direction = FORWARD_;
        /* set a line for each dot in the head */
        engine.numPrintLines = getHeadStyleSize() + 1; // we print the first dot twice
        engine.totalLinesToPrint = engine.numPrintLines;
        engine.labelOrientation = pOperation->print.orientation;
        
       if( currentStatus.error == NO_ERROR ) {
            startPrintEngine();
       }
   } else {
        engine.outOfMediaCnt = 0;
        /* we are printing */
        if( currentStatus.error != 0 ) {
            /* return to the idle state */
            setOperation( IDLE_DIRECTIVE, &currentStatus );
        }
        else if( engine.linePrintDone == true ) {
            currentStatus.command = _TBL_PRINT;
            resetPrintDataLine();            
            setNextOperation( &currentStatus );            
        }
    }    
}

/******************************************************************************/
/*!   \fn void historyAdjacency( void )
      \brief
        This function generates the history and adjacency lines for the next
        print line.
        
      \author
          Aaron Swift
*******************************************************************************/
void historyAdjacency( void )
{    
    unsigned char *pCurrentLine = getCurrentPrintDataLine();  
    
    if( pCurrentLine != NULL ) {
        //PRINTF("historyAdjacency() pCurrentLine != null\r\n");
        history( pCurrentLine );
    } else {
        PRINTF("historyAdjacency(): Warning: Out of print head data!\r\n" );            
    }        
}

/******************************************************************************/
/*!   \fn void stepOp( StepOperation *pOperation )
      \brief
        This function handles the teach table step command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepOp( StepOperation *pOperation )
{
    setOperation( IDLE_DIRECTIVE, &currentStatus );
}

/******************************************************************************/
/*!   \fn void stepUntilOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table step until command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepUntilOp( StepUntilOperation *pOperation )
{
    /* static int index_ = 0; */

    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING_UNTIL!\r\n" );

        /* added for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {        

            if( testCondition( &currentStatus, pOperation->operator, pOperation->bits, 
                      pOperation->result) == true)
            {
                PRINTF( "Here!! steps: %d\r\n",  engine.steps );
                
                /* if we are going to skip stepping then we need to clear the user 
                   status because we do not set label present and taken status bits.
                   when the test is ran (sizing) then a missing label error is generated 
                   in the backend which is not handled correctly and causes the backend to 
                   stop printing labels. This condition is recreated after x number of reboots.
                */
                skipMissingLabel_ = true;
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                setNextOperation( &currentStatus );
            }
        }

        /* freestanding scale        
        index_ = 0; */

        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        

        engine.steps = 0;    
        powerOnStepper();

        /*if we are sizing a label, slow sizing speed to 3ips (motor stall ) -- ats */ 
        if( ( pOperation->bits == MOTOR_FORWARD | SYNC_BAR | SYNC_BAR_EDGE ) && ( ( engine.headType == KYOCERA753_OHM ) ||
            ( engine.headType == KYOCERA800_OHM ) || ( engine.headType == KYOCERA849_OHM ) ) ) {
            currentStatus.sensor |= MOTOR_FORWARD;
            //currentStatus.sensor &= ~LABEL_TAKEN;
            
            setPrintEngineTimer( getSltSizingTime( engine.headType ) );
        } else {
            setPrintEngineTimerSlt();
	}
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps );
        /* initializeStepper( engine.direction ); */
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            
            if(getTakingUpPaper() == true)
            {
                //halfStepMotor(); 
            }
            else
            {
                //stepMainMotor();
            }    
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() == RT_GLOBAL_FSS ) {
            /* need to leave this in to accurately size labels. */   
            int mediaValue = pollMediaCounts();//getMediaCounts();
            
            if( mediaValue > MEDIA_SYNC_BAR_THRESHOLD ) {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR_EDGE;               
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            } else {
                engine.outOfMediaCnt = 0;    
                if ( ( currentStatus.sensor & SYNC_BAR ) == SYNC_BAR )
                    currentStatus.sensor |= SYNC_BAR_EDGE;    
                else
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;  
                currentStatus.sensor &= ~SYNC_BAR;           
            }
        } else {
            int mediaValue = pollMediaCounts();//getMediaCounts();            
            if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
                
                if( ( currentStatus.sensor & MOTOR_FORWARD ) == MOTOR_FORWARD ){
                    pCurrentLabel = pCurrentLabel->next;
                    currentStatus.sensor |= SYNCHRONIZED;    
                } else {
                    if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                        currentStatus.sensor |= SYNC_BAR;  
                        currentStatus.sensor &= ~SYNCHRONIZED;
                        pCurrentLabel->next->position = 0;
                    } else {
                        currentStatus.sensor &= ~SYNC_BAR_EDGE;
                    }
                    currentStatus.sensor |= SYNC_BAR;             
                }
            } else { 
                
                engine.outOfMediaCnt = 0; 
                if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                    currentStatus.sensor &= ~SYNCHRONIZED;
                }
            }        
        }
        /* added for freestanding scale */
        if(  getMyModel() == RT_GLOBAL_FSS  ) {
            if( pCurrentLabel->position == labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;            
            } else if( pCurrentLabel->position > labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor &= ~SYNCHRONIZED;
            } else {
                currentStatus.sensor &= ~SYNCHRONIZED;
            }
        }
        if( engine.outOfMediaCnt < engine.maxMediaCount ) {
            currentStatus.sensor &= ~OUT_OF_MEDIA;
            currentStatus.error &= ~MEDIA_SHUTDOWN;
        } else {
            if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                currentStatus.error |= MEDIA_SHUTDOWN;
            else        
                currentStatus.error &= ~MEDIA_SHUTDOWN;
        }
        
        /* added for freestanding scale */
        if( getMyModel() == RT_GLOBAL_FSS ) {        
            /* read label taken sensor */
            if( GPIO_ReadPinInput( LABEL_TAKEN_SENSOR_GPIO, LABEL_TAKEN_SENSOR_PIN ) ) { 
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;
            }
        } else {        
            /* read label taken sensor */
            if( !readLabelTakenSensor() ) {
                currentStatus.sensor |= LABEL_TAKEN;                
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;
            }
        }


        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( ( testCondition( &currentStatus, 
                  pOperation->operator, 
                  pOperation->bits, 
                  pOperation->result) == false) 
                  &&  ( --engine.numSteps > 0 ) ) {    

            int even = 0;
            even = engine.numSteps & 0x0001;

            if( even == 0 ) {
                /* two half steps per print line */            
                motorStep( engine.direction, &currentStatus );
            }
            
            engine.steps++;
            
            if(getTakingUpPaper() == true)
            {
                //halfStepMotor(); 
            }
            else
            {
                //stepMainMotor();
            }    
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {
                        
            PRINTF( "steps: %d\r\n",  engine.steps );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper();
                                    
            setNextOperation( &currentStatus );            
        }
    }	
}

/******************************************************************************/
/*!   \fn void stepGapOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table find the label gap command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepGapOp( StepUntilOperation *pOperation )
{   
    setStreamingLeadInMod(0);
    setStartOfQueue(false);
    
    firstPrint = true;
    
    setLabelQueuePaused(false);
    setLabelPauseBackwindPending(false);
    setLabelPauseTimeout(0);

  
    if(getCanceledSizingFlag() == true)
    {
        sizingState = GO_TO_IDLE;   
    }
  
    if( getTUCalStatus() == false && getGapCalStatus() == false && getHeadUp() == false)
    {
        streamingLabelBackwind = 0;
        setGapCalStatus(false);
        setTUCalStatus(false);
        
        setStreamingLabelBackwind( 0 );
        setTPHStepsPastGapThisPrint( 0 );
        setTPHStepsThisPrint( 0 );
        setTakeupBusy(true);
        
        currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
        currentStatus.sensor &= ~OUT_OF_MEDIA;
        
        /*
        typedef enum SizingState
        {
           PRE_SIZE_TIGHTEN,                    //0
           STEP_TO_LABEL_TAKEN,                 //1
           SIZE,                                //2
           STEP_TO_NEXT,                        //3
           GO_TO_IDLE,                          //4
           TAKEUP_BUSY                          //5
        }   SizingState_t;
        */

        
        if(getCutterInstalled_() == false && ( currentStatus.sensor & HEAD_UP ) != HEAD_UP)
        { 
            switch(sizingState)
            {
                case PRE_SIZE_TIGHTEN:
                {
                    PRINTF("PRE SIZE TIGHTEN\r\n");

                    checkForPaper(((float)config_.takeup_sensor_max_tension_counts * 0.90), 820);
                  
                    #if 0 /* TFink - "check paper" tightens the stock. tighten stock leads to pauses in the takeup motor steps, which
                             could lead to motor stalls */
                    if(getTakingUpPaper() == true)
                    {
                      tightenStock(((float)config_.takeup_sensor_min_tension_counts * 1.0), 920, false, HALF_STEP);
                    }
                    #endif
                  
                    sizingState++;
                    
                    break;
                }
              
                case STEP_TO_LABEL_TAKEN:
                {            
                    PRINTF("STEP_TO_LABEL_TAKEN\r\n");

                    currentStatus.sensor2 &= ~JAMMED_LABEL;
                    //currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
                    //currentStatus.sensor &= ~OUT_OF_MEDIA;
                    
                    sizingLabels = true;
                  
                    currentStatus.counter = 0;
                    engine.steps = 0;
                  
                    uint16_t stepsToLt = 4999;
                    stepToLt(stepsToLt, 575);
                    
                    sizingState = TAKEUP_BUSY;
                    break;
                }
                
                case SIZE:
                { 
                    PRINTF("SIZE\r\n");
                    
                    uint16_t stepsToSize = 10000;
                    sizeLabels(stepsToSize, 575);
                    
                    sizingState = TAKEUP_BUSY;
                    break;
                }
                
                case STEP_TO_NEXT:
                {            
                    PRINTF("STEP_TO_NEXT\r\n");
                    

                    if(getLargeGapFlag() == true)
                    {
                        if(getTakingUpPaper() == true)
                        {
                            stepToNextLabel(GAP_SENSOR_TO_PEEL_BAR_HT - getStepsBackToGap(), 575);
                        }
                        else
                        {
                            stepToNextLabel(GAP_SENSOR_TO_TEAR_BAR_HT - getStepsBackToGap(), 575);
                        }
                    }
                    else
                    {
                        if(getTakingUpPaper() == true)
                        {
                            stepToNextLabel(GAP_SENSOR_TO_PEEL_BAR_GT - getStepsBackToGap(), 575);
                        }
                        else
                        {
                            stepToNextLabel(GAP_SENSOR_TO_TEAR_BAR_GT - getStepsBackToGap(), 575);
                        }
                    }
                    
                    backwindAfterSizing = true;
                    
                    sizingState = TAKEUP_BUSY;
                    break;
                }
                
                case GO_TO_IDLE:
                {          
                    //PRINTF("GO_TO_IDLE\r\n");
                    
                    setSizingStatus(false);
                  
                    streamingLabelBackwind = 0;
                    
                    sizingState = PRE_SIZE_TIGHTEN;
                    
                    setBackwindAfterSizingDone(false);
      
                    setNextOperation( &currentStatus );
                    break;
                }
                
                case TAKEUP_BUSY:
                {
                    //PRINTF("sizingState = %d = busy\r\n", sizingState);
                    break;
                }
            }
        }
        else if( getCutterInstalled_() == true && ( currentStatus.sensor & HEAD_UP ) != HEAD_UP)
        {
            PRINTF("\r\n\r\nCUTTER SIZING INITIATED\r\n\r\n");
            
            ICMessages msg1;
            ICMessages msg2;
            
            msg1.generic.msgType = _I_CUTTER_CUT_CMD;
            msg2.generic.msgType = _I_CUTTER_HOME_CMD;
            
            if(getCutterHome() == false)
            {
                PRINTF("\r\ncutter not home on size\r\n");
                handleInternalMessage(&msg2);
            }
            
            checkForPaper(((float)config_.takeup_sensor_max_tension_counts * 0.90), 820);
          
            if(getTakingUpPaper() == true)
            {
                tightenStock(((float)config_.takeup_sensor_min_tension_counts * 1.0), 920, false, HALF_STEP);
            }
            
            while(getTakeupBusy() == true)
            {
                __NOP();
            }
            
            stepToNextLabel(1000, 862);
            
            while(getTakeupBusy() == true)
            {
                __NOP();
            }
            
            handleInternalMessage(&msg1);
                    
            setNextOperation( &currentStatus );
        }
        else
        {
            PRINTF("NO SIZING - OUT_OF_MEDIA OR HEAD UP\r\n");
            
            if(( currentStatus.sensor & OUT_OF_MEDIA ) == OUT_OF_MEDIA)
            {
                PRINTF("OUT_OF_MEDIA\r\n");
            }
            
            if(( currentStatus.sensor & HEAD_UP ) == HEAD_UP)
            {
                PRINTF("HEAD_UP\r\n");
            }
            
            setTakeupBusy(false);
            setLabelSizeInQuarterSteps(4999);
            
            setSizingStatus(false);
            
            setNextOperation( &currentStatus );
        }
    }
    else
    {   
        PRINTF("NO SIZING\r\n");     
      
        if( getTUCalStatus() == true )
        {
            PRINTF("TU CAL status true\r\n");
        }
        
        if( getGapCalStatus() == true )
        {
            PRINTF("GAP CAL status true\r\n");
        }
          
        if(( currentStatus.sensor & OUT_OF_MEDIA ) == OUT_OF_MEDIA)
        {
            PRINTF("OUT_OF_MEDIA\r\n");
        }
        
        if(( currentStatus.sensor & HEAD_UP ) == HEAD_UP)
        {
            PRINTF("HEAD_UP\r\n");
        }
        
        if(getHeadUp() == true)
        {
            PRINTF("getHeadUp() == true\r\n");
        }
        
        if(getOutOfMedia() == true)
        {
            PRINTF("getOutOfMedia() == true\r\n");
        }
        
        setTakeupBusy(false);
        setLabelSizeInQuarterSteps(4999);

        setSizingStatus(false);
        
        setNextOperation( &currentStatus );
    }
}

/******************************************************************************/
/*!   \fn void detectionOp( StepUntilOperation *pOperation )
      \brief
        This function handles the setting up the label stock to a known position
        before trying to size.
        
      \author
          Aaron Swift
*******************************************************************************/
void detectionOp( StepUntilOperation *pOperation )
{
    static bool findGap_ = false;
    
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_DETECTION_OP!\r\n" );
        
        /* pause the adc thread and control manual 
        pauseResumeConversions( true );*/
        
        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        
        mainMotorStopped_ = false;
        engine.steps = 0;
           
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );

        /* determine if we are at rest on gap or label */
        int mediaValue = getMediaCounts(); 
        if( mediaValue > BACKING_PAPER_THRESHOLD  ) {
            findGap_ = true;
            /* have the sensor ISR check and stop motor on the gap */
            setMotorStopOnGap();
            PRINTF( "detectionOp() drive to gap.\r\n");
        } else {
            /* have the sensor ISR check and stop motor on edge */
            setMotorStopOnEdge();
            PRINTF( "detectionOp() drive to label edge.\r\n");
        }
        
        
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        setStepperDirection( engine.direction );
        
    } else {
        int mediaValue = getMediaCounts();
        int even = 0;
        
        if( findGap_ ) {
            /* keep stepping until the isr sets the flag or we run out of steps */
            if( ( !mainMotorStopped_ ) &&  ( --engine.numSteps > 0 ) ){
                even = engine.numSteps & 0x0001;
                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                engine.steps++;
                if( getTakingUpPaper() == true ) {
                    halfStepMotor(); 
                } else {
                    stepMainMotor();
                }                    
            } else {                
                clrMotorStopOnGap();
                PRINTF( "detectionOp() steps: %d\r\n", engine.steps );
                /* have we detected continuous stock while sizing */
                if( engine.steps > CONTINUOUS_STOCK_MIN ) {
                    /* jump to the end of sizing since we know the stock size already 
                      don't set continuous bit until size message from host */
                    jumpToOperation( &currentStatus,  10 );                
                } else {
                    skipNextOperation( &currentStatus ); 
                }          
            }   
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus );                                         
        } else {
            /* keep stepping until the isr sets the flag or we run out of steps */
            if( ( !mainMotorStopped_ ) &&  ( --engine.numSteps > 0 ) ){
                even = engine.numSteps & 0x0001;
                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                engine.steps++;
                if( getTakingUpPaper() == true ) {
                    halfStepMotor(); 
                } else {
                    stepMainMotor();
                }                    
            } else {
              
                clrMotorStopOnGap();
                /* stepping complete */
                resetPrintEngineTimer();
                /* have we detected continuous stock while sizing */
                if( engine.steps > CONTINUOUS_STOCK_MIN ) {
                    /* jump to the end of sizing since we know the stock size already 
                      don't set continuous bit until size message from host */
                    jumpToOperation( &currentStatus,  10 );                
                } else {
                    skipNextOperation( &currentStatus ); 
                }          
            }   
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus );                                                 
        }
    }        
}


/******************************************************************************/
/*!   \fn void stepEdgeOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table find the label edge command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepEdgeOp( StepUntilOperation *pOperation )
{
#if 0
    int even = 0;
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single STEP_EDGE_DIRECTIVE!\r\n" );
        
        /* only for freestanding scale */
        if( getMyModel() == RT_GLOBAL_FSS ) {        
                skipMissingLabel_ = true;
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                setNextOperation( &currentStatus );
        }

        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        /* have the sensor ISR check and stop motor on the label edge */
        setMotorStopOnEdge();

        engine.numSteps = 0;
        engine.steps = 0;    
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
        currentStatus.sensor &= ~SYNCHRONIZED;
        
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps ); */        
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        int mediaValue = pollMediaCounts();//getMediaCounts();
        
        if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
            
            if( ( currentStatus.sensor & MOTOR_FORWARD ) == MOTOR_FORWARD ){
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;    
            } else {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR;  
                    currentStatus.sensor &= ~SYNCHRONIZED;
                    pCurrentLabel->next->position = 0;
                } else {
                    if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                        currentStatus.sensor |= SYNC_BAR;  
                        currentStatus.sensor &= ~SYNCHRONIZED;
                        pCurrentLabel->next->position = 0;
                    } else {
                        currentStatus.sensor &= ~SYNC_BAR_EDGE;
                    }
                    currentStatus.sensor |= SYNC_BAR;             
                }
            } else { 
                
                engine.outOfMediaCnt = 0; 
                if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                    currentStatus.sensor &= ~SYNCHRONIZED;
                    PRINTF("stepEdgeOp(): clear SYNCHRONIZED bit\r\n" );
                    PRINTF("stepEdgeOp(): engine.steps %d\r\n", engine.steps ); 
                }
            }        
            
            if( engine.outOfMediaCnt < engine.maxMediaCount ) {
                currentStatus.sensor &= ~OUT_OF_MEDIA;
                currentStatus.error &= ~MEDIA_SHUTDOWN;
            } else {
                if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                    currentStatus.error |= MEDIA_SHUTDOWN;
                else        
                    currentStatus.error &= ~MEDIA_SHUTDOWN;
            }
            
            /* read label taken sensor */
            if( !readLabelTakenSensor() ) {
                currentStatus.sensor |= LABEL_TAKEN;                
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;
            }
        
            /* check for error conditions. */
            if( currentStatus.error != NO_ERROR ) {
                /* return the printer to the idle state */    
                powerOffStepper();
                setOperation( IDLE_DIRECTIVE, &currentStatus ); 
            } else if ( ( testCondition( &currentStatus, 
                      pOperation->operator, 
                      pOperation->bits, 
                      pOperation->result) == false) 
                      &&  ( --engine.numSteps > 0 ) ) {    

                even = engine.numSteps & 0x0001;

                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                
                engine.steps++;
                if(getTakingUpPaper() == true)
                {
                    halfStepMotor(); 
                }
                else
                {
                    stepMainMotor();
                }    
                
                /* notify host of any status change 
                compareStatus( &currentStatus, &prevStatus ); */
            } else {
                
                clrMotorStopOnEdge();
                PRINTF( "stepEdgeOp() mediaValue: %d\r\n",  mediaValue );
                PRINTF( "stepEdgeOp() pOperation->operator: %d\r\n",  pOperation->operator );
                PRINTF( "stepEdgeOp() pOperation->bits: %d\r\n",  pOperation->bits );
                PRINTF( "stepEdgeOp() pOperation->result: %d\r\n",  pOperation->result );

                PRINTF( "steps: %d\r\n",  engine.steps );
                PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                                        
                setNextOperation( &currentStatus );            
            }
        } else {
            even = engine.numSteps & 0x0001;

            if( even == 0 ) {
                /* two half steps per print line */            
                motorStep( engine.direction, &currentStatus );
            }
            
            engine.steps++;
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }            
        }
    }	
#endif
}

void stepTakeupTightenOp( StepOperation *pOperation ) 
{
    if (currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Tighten ENGINE_STEPPING!\r\n" );  
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        powerOnStepper();
        
        /* sync print line with the step rate */
        setPrintEngineTimerSlt();
        
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;
                
        setTakeUpMotorDirection( BACKWARDM_ );
        
        /* sync print line with the step rate */ 
        resetPrintEngineTimer();
        engine.steps++;
        if(getTakingUpPaper() == true)
        {  
            //stepTakeUpMotor();                            
        }
    } else {   
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if( ( engine.numSteps-- <= 0 ) ) {
           
            if( getTakingUpPaper() == true ) {
                tightenStock(2200, 500, false, HALF_STEP);// this function blocks within it now, so does loosen and backwind
            }
            
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();
            setNextOperation( &currentStatus );
        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                if(getTakingUpPaper() == true) {
                    //stepTakeUpMotor();                            
                }
            }
            engine.steps++;
        }
    }
  
}

/******************************************************************************/
/*!   \fn void testForSyncOp( StepOperation *pOperation )
      \brief
        This function tests whether we are at a sync bar or label edge.  
        The function will drive out 1/2" looking for a gap meaning we have 
        encountered a sync bar and have driven passed. The function will then 
        proceed to the label edge before exiting. If the function does not 
        encounter a gap then we have driven 1/2" into the label which we will 
        save the 1/2" distance to be used for our sizing measurement and the 
        exits.  
         
      \author
          Aaron Swift
*******************************************************************************/
void testForSyncOp( StepOperation *pOperation )
{
    static bool syncFound_ = false;
    static int index_ = 0;
    /* not supported in global scale. global scale stock has no sync bars */
    setNextOperation( &currentStatus ); 
#if 0    
    if (currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Single TEST_FOR_SYNC!\r\n" );  
                
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        engine.steps = 0;    
        powerOnStepper();
        
        /* freestanding scale */       
        index_ = 0;

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
                     
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);

        /* save steps value to be added back to total steps if sync bar not present */
        engine.stepsOffset = engine.numSteps; 
        
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;

        /* have the sensor ISR check and stop motor on the gap */
        setMotorStopOnGap();
   
        setStepperDirection( FORWARD_ );
        powerOnStepper();
         
    } else {   

        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() == RT_GLOBAL_FSS ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus );            
        } else {           
            int mediaValue = pollMediaCounts();//getMediaCounts();
            if( !syncFound_ ) {
                if( mediaValue <= BACKING_PAPER_THRESHOLD ) {    
                    /* sync bar found need to drive to edge */
                    syncFound_ = true;
                    /* clear our offset sync bar has been found */
                    engine.stepsOffset = 0;
                    /* add more distance so that we find the label edge */
                    engine.numSteps += 300; /* little over an inch should do it */
                    /* clear flag */
                    clrMotorStopOnGap();
                    /* set flag to stop motor on label edge*/
                    setMotorStopOnEdge();
                    /* turn power back on */
                    powerOnStepper();
                    PRINTF("testForSyncOp(): sync bar found!\r\n" ); 
                }
            }
            /* if we encountered the sync bar then look for the label edge */
            if( syncFound_ ) {
                if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                    /* clear our flag */                   
                    clrMotorStopOnEdge();
                    /* set our steps to zero */
                    engine.numSteps = 0;
                    PRINTF("testForSyncOp(): label edge found after sync bar!\r\n" ); 
                }
            }
        }

        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( engine.numSteps-- <= 0 ) {
            powerOffStepper();	
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();                       
                       
            if( syncFound_ ) {
                syncFound_ = false;
                /* at the label edge */
                setNextOperation( &currentStatus ); 
            } else {
                PRINTF("testForSyncOp(): no sync bar, label edge found!\r\n" ); 
                PRINTF("testForSyncOp(): engine.steps %d\r\n", engine.steps );
                /* skip next operation and continue with following operation. */
                skipNextOperation( &currentStatus );            
            }
        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                motorStep( engine.direction, &currentStatus );
            }
            engine.steps++;
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }
#endif    
}

/******************************************************************************/
/*!   \fn void testForLabelOp( StepOperation *pOperation )
      \brief
        The function will drive out 1/2" looking for the label taken sensor 
        to be blocked. The function will then 
        proceed to the label edge before exiting. If the function does not 
        encounter a gap then we have driven 1/2" into the label which we will 
        save the 1/2" distance to be used for our sizing measurement and then 
        exits.  
         
      \author
          Aaron Swift
*******************************************************************************/
void testForLabelOp( StepOperation *pOperation )
{
    
    static bool labelFound_ = false;
    
    if( currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Single TEST_FOR_LABEL!\r\n" );  
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        powerOnStepper();
        
        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
                     
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);

        /* save steps value to be added back to total steps if sync bar not present */
        engine.stepsOffset = engine.numSteps; 
        
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;

        /* have the sensor ISR check and stop motor on the gap */
        setMotorStopOnGap();
   
        setStepperDirection( FORWARD_ );
        powerOnStepper();
         
    } else {   
        
        if( readLabelTakenSensor() ) {
            if( !labelFound_ ) {
                labelFound_ = true;
                /* add more distance to peel the label */
                engine.numSteps += 410; /* 510 little over 2 1/2" should do it */
                /* turn power back on */
                powerOnStepper();
                PRINTF("testForLabelOp(): label at sensor!\r\n" );             
            }
        }
        /* check to see if we have a very short label 1.75" */
        int mediaValue = pollMediaCounts();//getMediaCounts();   
        if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
            engine.numSteps = 0;
        }        
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( engine.numSteps-- <= 0 ) {
            powerOffStepper();	
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();                       
                       
            if( labelFound_ ) {
                labelFound_ = false;
                /* label ready to be taken */
                setNextOperation( &currentStatus ); 
            } else {
                PRINTF("testForLabelOp(): no label taken trip!\r\n" ); 
                PRINTF("testForLabelOp(): engine.steps %d\r\n", engine.steps );
                /* skip next operation and continue with following operation. */
                skipNextOperation( &currentStatus );            
            }
        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                motorStep( engine.direction, &currentStatus );
            }
            engine.steps++;
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }     
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }    
}

/******************************************************************************/
/*!   \fn void testForContinuous( StepOperation *pOperation )
      \brief
        The function will test for continuous stock bit and if continuous then 
        step to the expel distance. Otherwise go to the next function.
         
      \author
          Aaron Swift
*******************************************************************************/
void testForContinuous( StepOperation *pOperation ) 
{
    if( continuousStock_ ) {
        if( currentStatus.state != ENGINE_STEPPING ) {    
            PRINTF("currentStatus.state: Entering Single TEST_FOR_CONTINUOUS!\r\n" );  
        
            /* set the state and enable the stepper motor*/       
            currentStatus.state = ENGINE_STEPPING;
            
            powerOnStepper();
            
            /* get my expel position */             
            engine.numSteps = getOpData( pOperation->type, pOperation->data );
            engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
            engine.numSteps = abs(engine.numSteps);
            engine.numSteps = engine.numSteps << 1;
           
            setStepperDirection( engine.direction );
        } else {
            if( currentStatus.error != NO_ERROR ) {
                /* return the printer to the idle state */    
                powerOffStepper();
                setOperation( IDLE_DIRECTIVE, &currentStatus ); 
            } else if ( engine.numSteps-- <= 0 ) {
                /* stepping complete */
                //resetPrintEngineTimer();
                powerOffStepper();
                                        
                setNextOperation( &currentStatus );                        
            } else {
                int even = engine.numSteps & 0x0001;
                if( even == 0 ) {
                    /* Two half steps per printline */  
                    motorStep( engine.direction, &currentStatus );
                }
                engine.steps++;
                
                if(getTakingUpPaper() == true)
                {
                    halfStepMotor(); 
                }
                else
                {
                    stepMainMotor();
                }    
                
                /* notify host of any status change */
                compareStatus( &currentStatus, &prevStatus );                     
            }
        }      
    } else {
        PRINTF("currentStatus.state: Skip TEST_FOR_CONTINUOUS!\r\n" ); 
        setNextOperation( &currentStatus ); 
    }
}

/******************************************************************************/
/*!   \fn void waitOp( WaitOperation *pOperation )
      \brief
        This function waits a specified amount of time (in mS) until executing
        the next operation. Note: FlexTimer runs at a rate of 400uS even timing 
        values will be off (under) by 400uS. 
      \author
          Aaron Swift
*******************************************************************************/
void waitOp( WaitOperation *pOperation )
{  
    static int waitCount_ = 0;
    if( currentStatus.state != ENGINE_WAITING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_WAITING!\r\n" );
      
        currentStatus.state = ENGINE_WAITING;
        /* get the wait time (mS) */
        waitCount_ = getOpData(pOperation->type, pOperation->data);               
        
        /* adjust count for 400uS timer rate */
        if( waitCount_ >= 10 ) {
            if( waitCount_ / 2 )
                waitCount_ = ( waitCount_ * 2 ) + 4;
            else 
              waitCount_ = ( waitCount_ * 2 ) - 4; 
        } else {
            if( waitCount_ / 2 )
                waitCount_ = ( waitCount_ * 2 ) + 1;                        
            else 
                waitCount_ = ( waitCount_ * 2 ) - 1;                        
        }         
    } else {           
        /* check for error conditions. */
        if ( currentStatus.error != NO_ERROR ) {
            powerOffStepper();            
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else  {  
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus );             
            if( waitCount_ <= 0 ) {
              setNextOperation(  &currentStatus );         
            } else {
                waitCount_--;
            }   
        }
    }
}

/******************************************************************************/
/*!   \fn void waitUntilOp(  WaitUntilOperation *pOperation  )
      \brief
        This function handles the teach table wait until event operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void waitUntilOp(  WaitUntilOperation *pOperation  )
{   
  
    if(currentStatus.state != ENGINE_WAITING)
    {
        currentStatus.state = ENGINE_WAITING;
        //PRINTF("Entering waitUntilOp()\r\n");
        
        setPrintingStatus(false);
        
        /* sample the shoot through sensor 100 times, if 90% of the samples are less than 
           OUT_OF_MEDIA_THRESHOLD, OUT_OF_MEDIA is set*/
        checkForOutOfMedia();
    }
    
    
    if(getLabelTaken() <= LABEL_TAKEN_THRESHOLD_NO_LABEL)
    { 
        setOperation( IDLE_DIRECTIVE, &currentStatus );
    }
}

/******************************************************************************/
/*!   \fn void waitUntilSizingOp(  WaitUntilOperation *pOperation  )
      \brief
        This function handles the teach table wait until event operation during
        sizing for FSSS.
        
      \author
          Aaron Swift
*******************************************************************************/
void waitUntilSizingOp(  WaitUntilOperation *pOperation  )
{
    PRINTF("currentStatus.state: Entering ENGINE_WAITING_UNTIL_SIZING!\r\n" );
    
    setNextOperation( &currentStatus );
}

/******************************************************************************/
/*!   \fn void testOp( TestOperation *pOperation )
      \brief
        This function handles the teach table test command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void testOp( TestOperation *pOperation )
{
    PRINTF("currentStatus.state: Entering ENGINE_TEST!\r\n" );

    if( testCondition( &currentStatus,
                    pOperation->operator, 
                    pOperation->bits, 
                    pOperation->result ) == true ) {
                      
        if( !skipMissingLabel_ ) {              
            currentStatus.user |= pOperation->true_bits; 
            /* check to make sure we are not setting a ghost missing label! */
            if( ( ( currentStatus.user & MISSING_LABEL ) == MISSING_LABEL ) && getGhostMCntr( ) != 0 ) {
                currentStatus.user &= ~MISSING_LABEL;
            }
        } else {
            PRINTF("*********************Skip setting missing label bit -- true ********************\r\n" );
            currentStatus.user = 0;
            skipMissingLabel_ = false;
        } 
        
        sendPrStatus( &currentStatus, true );
        
        if( pOperation->true_action == CONTINUE_EXECUTION) {
            setNextOperation( &currentStatus );
        } else {
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        }
        
    } else {
      
        if( !skipMissingLabel_ ) {              
            currentStatus.user |= pOperation->false_bits;
        } else {
            PRINTF("*********************Skip setting missing label bit -- false ********************\r\n" );
            currentStatus.user = 0;
            skipMissingLabel_ = false;
        }
        
        if( pOperation->false_action == CONTINUE_EXECUTION ) {
            setNextOperation( &currentStatus );
        } else {
            setOperation( IDLE_DIRECTIVE, &currentStatus );
            
            engine.steps = SHRT_MAX;
        }
    }
}

/******************************************************************************/
/*!   \fn void statusOp( StatusOperation *pOperation )
      \brief
        This function handles the teach table status command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void statusOp( StatusOperation *pOperation )
{   
    //PRINTF("currentStatus.state: Entering ENGINE_STATUS!\r\n" );
    if( pOperation->operator == SET_BITS ) {
        currentStatus.user |= pOperation->bits;
        if( ( currentStatus.user & TAKE_LABEL ) == TAKE_LABEL ) {
          PRINTF("statusOp(): setting take label bit\r\n" );
        } else if( ( currentStatus.user & MISSING_LABEL ) == MISSING_LABEL ) {
            PRINTF("statusOp(): setting missing label bit\r\n" );
            currentStatus.user = 0; 
        } else if( ( currentStatus.user & JAMMED_LABEL ) == JAMMED_LABEL ) {
            PRINTF("statusOp(): setting jammed label bit\r\n" );
        }
    }
    else if( pOperation->operator == CLEAR_BITS ) {
        currentStatus.user &= ~pOperation->bits;
    }
    setNextOperation( &currentStatus );
}

/******************************************************************************/
/*!   \fn void counterOp( CounterOperation *pOperation )
      \brief
        This function handles the teach table counter command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void counterOp( CounterOperation *pOperation )
{    
   // PRINTF("currentStatus.state: Entering ENGINE_COUNTER!\r\n" );
    
    switch( pOperation->operator )
    {
        default: 
        {
            PRINTF("counterOp() - labelSizeInQuarterSteps = %d\r\n", getLabelSizeInQuarterSteps());
            /* we are keeping track of half steps and sizing needs whole steps. */
            engine.steps = getLabelSizeInQuarterSteps();

            currentStatus.counter = (  engine.steps / 2 );

            stepCounterEnabled_ = false;
           
            break;
        }
    }
    setNextOperation( &currentStatus );
}

/******************************************************************************/
/*!   \fn void calibrateOp( CmdOp *pOperation )
      \brief
        This function start with the PWM set to maximum current (99.6%) and  
        slowly reduce the PWM to the minimum current (4.0%) or until the average 
        slope is within the desired tolerance.
      \author
          Aaron Swift
*******************************************************************************/
void calibrateOp( CmdOp *pOperation )
{
#if 0 /* TO DO: convert to LP5521 if needed for media sensor reflective */    
    static unsigned char calDutyCycle = DC_97_PERCENT; 
    FPMBLC3Checksums    checkSums;
    
    if( currentStatus.state != ENGINE_CALIBRATING ) {
      
        calDutyCycle = DC_97_PERCENT;     
        currentStatus.state = ENGINE_CALIBRATING;      
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 

        setSamplesCntMedia( 10 );
        
        setDutyCycleIsrDS1050( DS1050_MS_DEVICE_ADDR, calDutyCycle );
    } else {
      
        /* notify host of any status changes */
        compareStatus( &currentStatus, &prevStatus ); 

        /* wait for our sample to be ready */
        if(  isMediaSampleReady() ) {
           
            unsigned long cnts = pollMediaCounts();//getMediaCounts();
            /* media sensor threshold 1.0v */
            if( cnts >= MEDIA_CAL_VOLTAGE ) {   
                /* set to just above saturation */  
                calDutyCycle += 1; 
        
                PRINTF("calibrateOp(): cal duty cycle: %fKhz \r\n", calDutyCycle * DUTYCYCLE_RESOLUTION ); 
                PRINTF("calibrateOp(): cal count: %d \r\n", calDutyCycle ); 
                PRINTF("calibrateOp(): cal voltage: %fV \r\n", cnts * AD_RESOLUTION );
                PRINTF("calibrateOp(): cal counts: %d \r\n", cnts );
              
                /* save the media sensor adjustment to configuration */
                config_.media_sensor_adjustment = calDutyCycle;
                setSerialPrConfiguration( &config_ );
                
                /* read the checksum section to maintain weigher checksum */
                getPageChecksums( &checkSums );
                
                /* calculate new checksum for printer section */
                checkSums.prConfigSum = calculateChecksum ( &config_, sizeof (Pr_Config) );
                setPageChecksums(&checkSums);
                
                /* turn off sampling */
                clearSampleCntMedia();
                
                /* return to idle */
                setOperation( IDLE_DIRECTIVE, &currentStatus );               
            } else {
                /* adjust the PWM duty cycle by 3.125% and retest */
                calDutyCycle--;                            
                setSamplesCntMedia( 10 );
                setDutyCycleIsrDS1050( DS1050_MS_DEVICE_ADDR, calDutyCycle );
            }
            /* break out on error */
            if( calDutyCycle == PWM_0_PRECENT_DUTY) {
                /* return to idle */
                setOperation( IDLE_DIRECTIVE, &currentStatus );                           
            }
        }
    }
#endif    
}

void freePrinterCalBuffers(void)
{
	if( engine.TUCalMaxTensionVals != NULL )
	{
		vPortFree( engine.TUCalMaxTensionVals );
		engine.TUCalMaxTensionVals = NULL;
	}
	
	if( engine.TUCalMinTensionVals != NULL )
	{
		vPortFree( engine.TUCalMinTensionVals );
		engine.TUCalMinTensionVals = NULL;
	}
	
	if( engine.TUCalDeltaTensionVals != NULL )
	{
		vPortFree( engine.TUCalDeltaTensionVals );
		engine.TUCalDeltaTensionVals = NULL;
	}
	
	freeTUMotorCalArray();
	
}

/******************************************************************************/
/*!   \fn static void sendToPrinterTaskFromISR( void )

      \brief
        Function sends PR_TU_CAL_DONE message to Printer Task.
         
      \author
          Carlos Guzman
*******************************************************************************/          
static void sendTUCalDoneToPrinterTaskFromISR( BaseType_t *xHigherPriorityTaskWoken )
{
    PrMessage 		prMsg;
	QueueHandle_t 	q_handle = NULL;
    
	prMsg.generic.msgType 	= PR_TU_CAL_DONE;
    
	q_handle = getPrinterQueueHandle();
	
    if( q_handle ) 
	{
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBackFromISR( q_handle, (void *)&prMsg, xHigherPriorityTaskWoken );
    
		if( result != pdTRUE ) 
		{
            PRINTF("sendTUCalDoneToPrinterTaskFromISR(): PR Message Queue full! \r\n");  
        }             
    }
	else
	{
        PRINTF("sendTUCalDoneToPrinterTaskFromISR(): PR Message Queue handle null! \r\n");  
    }   
	
}

/******************************************************************************/
/*!   \fn static void calibrateTUOp( CmdOp *pOperation )

      \brief
        Print Engine Takeup Motor calibration state machine 
         
      \author
          Carlos Guzman
*******************************************************************************/      
void calibrateTUOp( CmdOp *pOperation )
{
	TakeupMotor tu_motor;
	unsigned short current_tension;
	PrGapCalStatus calStatus;
	
	
   /*increment 1mS cycle counter */
	engine.cycleCounter++;
		
    if( currentStatus.state != ENGINE_CALIBRATING ) {
      	
		PRINTF("currentStatus.state: Entering ENGINE_TU_CALIBRATION!\r\n" );

		currentStatus.state = ENGINE_CALIBRATING;      
		
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
		
		setTakeupFiltering();      			
		
		/* Make sure motor is not at reset */
		releaseFromReset( _TAKEUP_STEPPER );
		
		/* clear real time keeper, ONE_mS_ENGINE_TIME_BASE every cycle */
		engine.cycleCounter = 0;
				
		/* To aide in time keeping, adjust engine time period to run at 1ms time base */
		setPrintEngineTimer( ONE_mS_ENGINE_TIME_BASE );
		
		/* Apply a working Takeup current, value is not critical at this point */
		if( setPaperTakeupCurrent( CC_SEVEN_POINT_FIVE ) == true )
		{
			/* remember initial current */
			engine.TUCalEmittermACurrent = CC_SEVEN_POINT_FIVE;

			/* Verify that initial mA current does not saturate detector sensor  */
			engine.TUCalState = VERIFY_INITIAL_TAKEUP_CURRENT;
			
			/* record time */
			engine.cycleCounterRel = engine.cycleCounter;	
		
		}
		else
		{
			PRINTF("calibrateTUOp: Failed to set emitter mA current! \r\n");
			
			/* Update Host application */
			memset(&calStatus, 0, sizeof(calStatus) );
			calStatus.state 				= _TakeupCalFailure;
			calStatus.deflectionVoltage		= 0;
			
			sendPrTUCalStatusFromISR( &calStatus ); 
			
			
			/* Bail, could not set emitter current */
			engine.TUCalState = IDLE_TU_CAL;		
		}
		
//debug,  setting pin low before cal starts
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
		

    } else {
      
        /* notify host of any status changes */
        compareStatus( &currentStatus, &prevStatus ); 

		switch( engine.TUCalState )
		{
			case VERIFY_INITIAL_TAKEUP_CURRENT:
			{
				if( engine.cycleCounter >= engine.cycleCounterRel+TEN_mS )
				{
					unsigned short tension_val;
					
					tension_val = getPaperTakeUp();
					
					/*
					*	The idea here is to make sure that the Takeup sensor is not saturated with too much current.
					*	If the tension counts are pegged low this is an indication that the sensor is saturated
					* 	and that we need to lower the current on the emitter.
					*/
					if( tension_val < MIN_TENSION_VALUE_AT_REST )
					{
						/* lower the current until we exit saturation and enter the sensor's active range */
						engine.TUCalEmittermACurrent -= 2;
						
						if( engine.TUCalEmittermACurrent < CC_TWO_POINT_ZERO )
						{
#if 1
							PRINTF("VERIFY_INITIAL_TAKEUP_CURRENT(): Failed to set initial current, mA = %d\tTension = %d\r\n", engine.TUCalEmittermACurrent, tension_val );		
#endif			
							/* Update Host application */
							memset(&calStatus, 0, sizeof(calStatus) );
							calStatus.state 				= _TakeupCalFindMaxTensionFailure;
							calStatus.deflectionVoltage		= (unsigned short)VERIFY_INITIAL_TAKEUP_CURRENT;//debug;

							sendPrTUCalStatusFromISR( &calStatus ); 
							
							/* Keep config value */
							setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
							
							/* Enable motor again */
							releaseFromReset( _TAKEUP_STEPPER );
								
							/* Exit Cal */
							engine.TUCalState = IDLE_TU_CAL;	
						}
						else
						{
							setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
							
							/* record time */
							engine.cycleCounterRel = engine.cycleCounter;
						}

					}
					else
					{
						/* Ok, we seem to have the sensor in its active range. Set the next state */
						engine.TUCalState = PREPARE_MEDIA_TU_CAL_START;
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;
					}
					
				}
				break;
			}
			
			case PREPARE_MEDIA_TU_CAL_START:
			{				
				/* 
				*	Let a few milliseconds go by since 
				*	we set current to Emitter at startup. One engine cycle == 1mS
				*/
				if( engine.cycleCounter >= engine.cycleCounterRel+TEN_mS ) 
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);					

					/* clear out motor struct */
					memset( &tu_motor, 0, sizeof(tu_motor) );
					
					/*
					*	Let's run TU motor to remove any slack in the media.
					*/
					tu_motor.speed 					= 1500; //speedInUs
					tu_motor.torqueThreshold 		= getPaperTakeUp() + TU_CAL_INIT_TENSION;//desired tension threshold
					tu_motor.torqueThresholdCalCntr	= 20;//min of 20steps at Threshold to call media tight
					
					/*record initial tension */
					engine.TUCalInitialTension			= getPaperTakeUp();	
					
					/* record when we started motor */
					engine.cycleCounterRel				= engine.cycleCounter;
						
					/* start TU motor ISR */
					prepMediaTUCal( &tu_motor );
						
					/* change state to check for end */
					engine.TUCalState = PREPARE_MEDIA_TU_CAL_END;
				}
				break;
			}
			
			case PREPARE_MEDIA_TU_CAL_END:
			{
				/* Is motor done or we've reached a timeout */
				if( getTakeupBusy() == false )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);

					/* grab a copy of the Take Up motor driver */
					getTUMotor( &tu_motor );
					
					/* validate tension counts */
					current_tension = getPaperTakeUp();
					
					/* achieved tension should be equal or greater than the desired */
					if( current_tension >= tu_motor.torqueThreshold  || current_tension >= TU_CAL_INIT_TENSION)
					{
#if 1						
						PRINTF("PREPARE_MEDIA_TU_CAL_END: Passed \r\nInitial Tension =\t%d\r\nTarget Tension = \t%d\r\nEnd Tension = \t%d\r\nSteps = \t%d\r\n", 
							   engine.TUCalInitialTension, tu_motor.torqueThreshold, current_tension, tu_motor.steps );
#endif						
						
						/* 
						*	Turn off TU motor to release media tension and 
						*	let spring retract 
						*/
						resetMotor(  _TAKEUP_STEPPER );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;
						
						/* go to relax period */
						engine.TUCalState = RELAX1_MEDIA_TU_CAL;
					}
					else
					{
						PRINTF("PREPARE_MEDIA_TU_CAL_END: FAILED \r\nInitial Tension =\t%d\r\nTarget Tension = \t%d\r\nEnd Tension = \t%d\r\n", 
							   engine.TUCalInitialTension, tu_motor.torqueThreshold, current_tension );
						
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalFindInitTensionFailure;
						calStatus.deflectionVoltage		= (unsigned short)PREPARE_MEDIA_TU_CAL_END;//debug

						sendPrTUCalStatusFromISR( &calStatus ); 
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
							
						/* Exit Cal */
						engine.TUCalState = IDLE_TU_CAL;
						
					}
					
				}
				else if( engine.cycleCounter > (engine.cycleCounterRel+TU_CAL_TIMEOUT) )
				{
					PRINTF("PREPARE_MEDIA_TU_CAL_END: failed to set initial tension, TIMEOUT reached \r\n");					
										
					/* Update Host application */
					memset(&calStatus, 0, sizeof(calStatus) );
					calStatus.state 				= _TakeupCalTimeoutFailure;
					calStatus.deflectionVoltage		= (unsigned short)PREPARE_MEDIA_TU_CAL_END;//debug;

					sendPrTUCalStatusFromISR( &calStatus ); 
					
					/* Keep config value */
					setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
					
					/* Stop Takeup Motor */
					stopTakeupIntr();
										
					/* Exit Cal, TIME OUT reached */
					engine.TUCalState = IDLE_TU_CAL;
				}
				
				break;
			}
			
			case RELAX1_MEDIA_TU_CAL:
			{
				/* Wait for half second for spring to retract */
				if( engine.cycleCounter >= engine.cycleCounterRel+HALF_SECOND_IN_mS )
				{
					/* Enable motor again */
					releaseFromReset( _TAKEUP_STEPPER );
					
					/* clear out motor struct */
					memset( &tu_motor, 0, sizeof(tu_motor) );
					
					/*
					*	Set motor to low speed(high torque) to force a stall.
					*/
					tu_motor.speed	= 3000; //speedInUs
					
					/* Start TU Motor ISR */
					forceMotorStallTUCal(&tu_motor);
					
					/* record time */
					engine.cycleCounterRel = engine.cycleCounter;
					
					/* set next state */
					engine.TUCalState = DETECT_MOTOR_STALL_TU_CAL;

				}
				
				break;	
			}	
			
			case DETECT_MOTOR_STALL_TU_CAL:
			{
				/* Wait for TU motor to stall */
				if( getTakeupBusy() == false )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);

					/* grab a copy of the Take Up motor driver to see results */
					getTUMotor( &tu_motor );
					
					/* validate the max tension reached */
					if( tu_motor.maxTensionTUCal >= MIN_TU_CAL_TENSION )
					{	
#if 1						
						PRINTF("DETECT_MOTOR_STALL_TU_CAL(): Passed, value = %d\t steps = %d\r\n", tu_motor.maxTensionTUCal, tu_motor.steps );
#endif			
						/* save max tension */
						engine.TUCalFinalTension = tu_motor.maxTensionTUCal;
							
						/* Turn off TU motor to let spring retract */
						resetMotor(  _TAKEUP_STEPPER );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;
						
						/* go to relax period */
						engine.TUCalState = RELAX2_MEDIA_TU_CAL;
		
					}
					else
					{
#if 1
						PRINTF("DETECT_MOTOR_STALL_TU_CAL(): Failed, Min Tension = %d\t Actual Tension = %d\t steps = %d\r\n", \
							   							MIN_TU_CAL_TENSION, tu_motor.maxTensionTUCal, tu_motor.steps );		
#endif			
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalFindMaxTensionFailure;
						calStatus.deflectionVoltage		= (unsigned short)DETECT_MOTOR_STALL_TU_CAL;//debug;

						sendPrTUCalStatusFromISR( &calStatus ); 
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
							
						/* Exit Cal */
						engine.TUCalState = IDLE_TU_CAL;	
					}
					
				}
				else if( engine.cycleCounter > (engine.cycleCounterRel+TU_CAL_TIMEOUT) )
				{
					PRINTF("DETECT_MOTOR_STALL_TU_CAL: TIMEOUT reached \r\n");					
										
					/* Update Host application */
					memset(&calStatus, 0, sizeof(calStatus) );
					calStatus.state 				= _TakeupCalTimeoutFailure;
					calStatus.deflectionVoltage		= (unsigned short)DETECT_MOTOR_STALL_TU_CAL;//debug;;
					
					sendPrTUCalStatusFromISR( &calStatus ); 
					
					/* Keep config value */
					setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
					
					/* Exit Cal, TIME OUT reached */
					engine.TUCalState = IDLE_TU_CAL;
				}
				break;
			}
			
			case RELAX2_MEDIA_TU_CAL:
			{
				/* 
				*	Wait for half second to let spring retract and then
				* 	start motor to go to max tension.
				*/
				if( engine.cycleCounter >= engine.cycleCounterRel+HALF_SECOND_IN_mS )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);					
		
					/* renable motor */
					releaseFromReset( _TAKEUP_STEPPER );
					
					/* clear out motor struct */
					memset( &tu_motor, 0, sizeof(tu_motor) );
					
					/*
					*	Set motor params to go to Max tension minus a little 
					*	to avoid a stall
					*/
					tu_motor.speed					= 3000; //speedInUs
					tu_motor.torqueThreshold		= engine.TUCalFinalTension-MAX_CAL_SETPOINT_ADJ;//minus a little so we don't stall
					tu_motor.torqueThresholdCalCntr = 1;
					
					/*record initial tension */
					engine.TUCalInitialTension	= getPaperTakeUp();	
					
					/* record when we started motor */
					engine.cycleCounterRel		= engine.cycleCounter;
					
					/* start tu motor isr   */
					prepMediaTUCal( &tu_motor );
					
					/* set next state */
					engine.TUCalState = DETECT_MAX_TENSION_TU_CAL;
				
				}
				break;
			}
			
			case DETECT_MAX_TENSION_TU_CAL:
			{	
				/* wait for TU motor to finish */
				if( getTakeupBusy() == false )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);

					/* grab a copy of the Take Up motor driver */
					getTUMotor( &tu_motor );
					
					current_tension = getPaperTakeUp();
					
					/* validate current tension against the desired max tension torqueThreshold */
					if( abs(current_tension - tu_motor.torqueThreshold) <= MAX_CAL_THRESH_TOL )
					{	
#if 1						
						PRINTF("DETECT_MAX_TENSION_TU_CAL(): Passed, Current tension is = %d\t steps = %d\r\n", current_tension, tu_motor.steps );
#endif			
						/* Prepare for next step which is to collect cal data points at max spring */
						memset(engine.TUCalMaxTensionVals, 0, CC_TWENTY_FIVE_POINT_FIVE*sizeof(unsigned short) );
						
						/* set takeup bias current to min */
						engine.TUCalEmittermACurrent = CC_Zero; 
						setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;
						
						/* go to collect data points */
						engine.TUCalState = COLLECT_MAX_TENSION_DATA_TU_CAL;
		
					}
					else
					{
#if 1
						PRINTF("DETECT_MAX_TENSION_TU_CAL(): Failed, value = %d\t steps = %d\r\n", current_tension, tu_motor.steps );		
#endif			
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalHoldMaxTensionFailure;
						calStatus.deflectionVoltage		= (unsigned short)DETECT_MAX_TENSION_TU_CAL;//debug;;

						sendPrTUCalStatusFromISR( &calStatus ); 
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
						
						/* Exit Cal */
						engine.TUCalState = IDLE_TU_CAL;
					}
					
				}
				else if( engine.cycleCounter > (engine.cycleCounterRel+TU_CAL_TIMEOUT) )
				{
					PRINTF("DETECT_MAX_TENSION_TU_CAL: TIMEOUT reached \r\n");					
										
					/* Update Host application */
					memset(&calStatus, 0, sizeof(calStatus) );
					calStatus.state 				= _TakeupCalTimeoutFailure;
					calStatus.deflectionVoltage		= (unsigned short)DETECT_MAX_TENSION_TU_CAL;//debug;;;

					sendPrTUCalStatusFromISR( &calStatus ); 
					
					/* Keep config value */
					setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
							
					/* Exit Cal, TIME OUT reached */
					engine.TUCalState = IDLE_TU_CAL;
				}
				break;
			}
			
			case COLLECT_MAX_TENSION_DATA_TU_CAL:
			{
				/* 
				*	At this point, spring is at max extension,
				*	keep motors (On) while we're gathering cal data 
				*/
#if 1
				powerOnMotorsDuringCal();
#else	
    			powerOnMotors();
#endif
				
				/* 
				*	If for some reason the Cassette opens during this process
				*	abort calibration.
				*/
				if( (currentStatus.error & HEAD_UP ) == HEAD_UP )
				{
#if 1					
					PRINTF("COLLECT_MAX_TENSION_DATA_TU_CAL(): Failed, Cassette Open!" );							
#endif					
					/* Update Host application */
					memset(&calStatus, 0, sizeof(calStatus) );
					calStatus.state 				= _TakeupCalFailure;
					calStatus.deflectionVoltage		= (unsigned short)COLLECT_MAX_TENSION_DATA_TU_CAL;//debug;;;

					sendPrTUCalStatusFromISR( &calStatus ); 
					
					/* Keep config value */
					setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
					
					/* Exit Cal */
					engine.TUCalState = IDLE_TU_CAL;
					break;
				}
				

				/* Collect cal data, let about 10mS pass by everytime we take a reading */
				if( engine.cycleCounter >= engine.cycleCounterRel+TEN_mS )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);					

					/* Record TU sensor reading at every mA emitter current */
					if( engine.TUCalMaxTensionVals != NULL )
					{
						engine.TUCalMaxTensionVals[engine.TUCalEmittermACurrent] = getPaperTakeUp();
#if 0						
						PRINTF("engine.TUCalMaxTensionVals[%d] =\t%d \r\n", engine.TUCalEmittermACurrent, engine.TUCalMaxTensionVals[engine.TUCalEmittermACurrent] );		
#endif						
					}
					else
					{
#if 1
						PRINTF("COLLECT_MAX_TENSION_DATA_TU_CAL(): Failed, engine.TUCalMaxTensionVals = NULL" );		
#endif			
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalFailure;
						calStatus.deflectionVoltage		= (unsigned short)COLLECT_MAX_TENSION_DATA_TU_CAL;//debug;;;

						sendPrTUCalStatusFromISR( &calStatus ); 
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
							
						/* Exit Cal */
						engine.TUCalState = IDLE_TU_CAL;
						break;
					}
					
					/* Have we scanned through all mA currents */
					if( engine.TUCalEmittermACurrent < CC_TWENTY_FIVE_POINT_FIVE )
					{
						/* increment current */
						engine.TUCalEmittermACurrent ++;
						setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;	
//debug						
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);						

					}
					else
					{
						/* Ok, All done, set current back to min */						
						engine.TUCalEmittermACurrent = CC_Zero; 
            			setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
												
						/* Turn off TU motor to let spring retract */
						resetMotor(  _TAKEUP_STEPPER );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;
						
						/* go to relax period */
						engine.TUCalState = RELAX3_MEDIA_TU_CAL;

//debug						
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);												
		
					}
					
				}
				
				break;
			}
			
			case RELAX3_MEDIA_TU_CAL:
			{	
				/* Wait for half second for spring to retract */
				if( engine.cycleCounter >= engine.cycleCounterRel+HALF_SECOND_IN_mS )
				{
					/* renable motor */
					releaseFromReset( _TAKEUP_STEPPER );
					
					/* Prepare for next step which is to collect cal data points at min spring extension */
					memset(engine.TUCalMinTensionVals, 0, CC_TWENTY_FIVE_POINT_FIVE*sizeof(unsigned short));
					
					/* set takeup bias current to min */
					engine.TUCalEmittermACurrent = CC_Zero; 
					setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
					
					/* record time */
					engine.cycleCounterRel = engine.cycleCounter;
					
					/* go to collect data points */
					engine.TUCalState = COLLECT_MIN_TENSION_DATA_TU_CAL;
				}				
				break;
			}
			
			case COLLECT_MIN_TENSION_DATA_TU_CAL:
			{
				/* 
				*	Spring is now relaxed at min extension, keep motors (On)
				*	at this position while we gathering cal data 
				*/
#if 1
				powerOnMotorsDuringCal();
#else	
    			powerOnMotors();
#endif
				
				/* 
				*	If for some reason the Cassette opens during this process
				*	abort calibration.
				*/
				if( (currentStatus.error & HEAD_UP ) == HEAD_UP )
				{
#if 1					
					PRINTF("COLLECT_MIN_TENSION_DATA_TU_CAL(): Failed, Cassette Open!" );												
#endif
					/* Update Host application */
					memset(&calStatus, 0, sizeof(calStatus) );
					calStatus.state 				= _TakeupCalFailure;
					calStatus.deflectionVoltage		= (unsigned short)COLLECT_MIN_TENSION_DATA_TU_CAL;//debug;;;

					sendPrTUCalStatusFromISR( &calStatus ); 
					
					/* Keep config value */
					setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
							
					/* Exit Cal */
					engine.TUCalState = IDLE_TU_CAL;
					break;
				}
				
				
				/* Collect Cal data, let about 10mS pass by everytime we take a reading */
				if( engine.cycleCounter >= engine.cycleCounterRel+TEN_mS )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);

					/* record TU sensor reading at every mA emitter current */
					if( engine.TUCalMinTensionVals != NULL )
					{
						engine.TUCalMinTensionVals[engine.TUCalEmittermACurrent] = getPaperTakeUp();
#if 0
						PRINTF("engine.TUCalMinTensionVals[%d] =\t%d \r\n", engine.TUCalEmittermACurrent, engine.TUCalMinTensionVals[engine.TUCalEmittermACurrent] );								
#endif						
					}
					else
					{
#if 1
						PRINTF("COLLECT_MIN_TENSION_DATA_TU_CAL(): Failed, engine.TUCalMinTensionVals = NULL" );		
#endif			
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalFailure;
						calStatus.deflectionVoltage		= (unsigned short)COLLECT_MIN_TENSION_DATA_TU_CAL;//debug;;;;
						
                    	sendPrTUCalStatusFromISR( &calStatus ); 
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
						
						/* Exit Cal */
						engine.TUCalState = IDLE_TU_CAL;
						break;						
					}
					
					/* Have we parsed throuh all mA currents */
					if( engine.TUCalEmittermACurrent < CC_TWENTY_FIVE_POINT_FIVE )
					{
						/* increment current */
						engine.TUCalEmittermACurrent ++;
						setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;						
//debug						
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
					}
					else
					{
						/* Ok, All done, set current back to min */						
						engine.TUCalEmittermACurrent = CC_Zero; 
            			setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
						
						/* record time */
						engine.cycleCounterRel = engine.cycleCounter;
						
						/* go to calculate calibration */
						engine.TUCalState = CALCULATE_TU_CAL_DELTAS;
//debug						
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
					}
					
				}		
				break;
			}
			
			case CALCULATE_TU_CAL_DELTAS:
			{
				/*
				*	Calculate the deflections between min and max readings
				*/
				if( engine.cycleCounter >= engine.cycleCounterRel+TEN_mS )
				{
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);					
					
					if( engine.TUCalDeltaTensionVals != NULL )
					{
						/* fill delta array with differences between min/max */
						for(int index=0; index<=CC_TWENTY_FIVE_POINT_FIVE; index++)
						{
							engine.TUCalDeltaTensionVals[index] = abs( engine.TUCalMaxTensionVals[index] - 
																	   engine.TUCalMinTensionVals[index] );			
						}
					}
					else
					{
#if 1
						PRINTF("CALCULATE_TU_CAL_DELTAS(): Failed, engine.TUCalDeltaTensionVals = NULL" );		
#endif			
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalTimeoutFailure;
						calStatus.deflectionVoltage		= (unsigned short)CALCULATE_TU_CAL_DELTAS;//debug;;;;

						sendPrTUCalStatusFromISR( &calStatus ); 
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
						
						/* Exit Cal */
						engine.TUCalState = IDLE_TU_CAL;
						break;						
					}
					
					/* record time */
					engine.cycleCounterRel = engine.cycleCounter;
					
					/* go to calc set point */
					engine.TUCalState = CALCULATE_TU_CAL_SETPOINT;
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);							
				}
				break;
			}
			
			case CALCULATE_TU_CAL_SETPOINT:
			{
				unsigned short max_delta;
				
				if( engine.cycleCounter >= engine.cycleCounterRel+TEN_mS )
				{
					/*
					*	Figure out mA current at point of max deflection
					*/
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);		
					
					max_delta 						= engine.TUCalDeltaTensionVals[0];
					engine.TUCalEmittermACurrent 	= CC_Zero;
					
					/* parse for max deflection and record mA at that point */
					for(int index=0; index<=CC_TWENTY_FIVE_POINT_FIVE; index++)
					{
						if( engine.TUCalDeltaTensionVals[index] > max_delta )
						{
							max_delta 						= engine.TUCalDeltaTensionVals[index];
							engine.TUCalEmittermACurrent 	= index;
						}
					}
					
					/* Bounds check our emitter current. Above 10mA the sensor saturates */
					if( ( engine.TUCalEmittermACurrent >= CC_THREE_POINT_ZERO ) 	&& 
					    ( engine.TUCalEmittermACurrent <= CC_NINE_POINT_ZERO) 		&&
						  max_delta						> MIN_TU_CAL_DELTA_CNTS	) 
					{
						//set min/max sensor values
#if 0//Toms method						
						unsigned short max_sensor_value = engine.TUCalMaxTensionVals[ engine.TUCalEmittermACurrent ];
						unsigned short min_sensor_value = engine.TUCalMinTensionVals[ engine.TUCalEmittermACurrent ];
#else
						unsigned short max_sensor_value = (int)(0.72 * (float)engine.TUCalMaxTensionVals[ engine.TUCalEmittermACurrent ]);
						unsigned short min_sensor_value = (int)(0.70 * (float)engine.TUCalMaxTensionVals[ engine.TUCalEmittermACurrent ]);											
#endif						
						BaseType_t xHigherPriorityTaskWoken = pdFALSE;
						
						setPaperTakeupCurrent( engine.TUCalEmittermACurrent );
						
						/* Update global config struct */
						config_.takeup_sensor_drive_current			= engine.TUCalEmittermACurrent;    
    					config_.takeup_sensor_max_tension_counts 	= max_sensor_value;
    					config_.takeup_sensor_min_tension_counts 	= min_sensor_value;

						/* 
						*	Notify Printer Task that we're done. 
						*	When Printer task gets notification, it will save 
						* 	Config to serial EEP.
						*/
						sendTUCalDoneToPrinterTaskFromISR( &xHigherPriorityTaskWoken );
				
				
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _Done;
						calStatus.deflectionVoltage 	= engine.TUCalEmittermACurrent;//Confusing.... but backend uses deflectionVoltage to fill TUCal current
						calStatus.driveCurrent 			= config_.media_sensor_adjustment;
						sendPrTUCalStatusFromISR( &calStatus );
#if 1						
						PRINTF("\r\ntakeupSensorCal(): struct values: \r\n");
						PRINTF("calStatus.msgType: %d\r\n", calStatus.msgType );
						PRINTF("calStatus.state: %d\r\n", calStatus.state );
						PRINTF("calStatus.TUSensorDriveCurrent: %d\r\n", calStatus.TUSensorDriveCurrent );
						PRINTF("calStatus.deflectionVoltage(TU Drive mA): %d\r\n", calStatus.deflectionVoltage );
						PRINTF("calStatus.driveCurrent: %d\r\n", calStatus.driveCurrent );
						PRINTF("calStatus.backingVoltage: %d\r\n", calStatus.backingVoltage );
						PRINTF("calStatus.labelBackVoltage: %d\r\n", calStatus.labelBackVoltage );					
						PRINTF("A/D Max Counts: %d\r\n", engine.TUCalMaxTensionVals[ engine.TUCalEmittermACurrent ] );
						PRINTF("A/D Max Counts Calibrated: %d\r\n", max_sensor_value );	
						PRINTF("A/D Min Counts Calibrated: %d\r\n", min_sensor_value );	
						
#endif

						if( xHigherPriorityTaskWoken == pdTRUE )
							taskYIELD ();

					} 
					else 
					{
						/* Update Host application */
						memset(&calStatus, 0, sizeof(calStatus) );
						calStatus.state 				= _TakeupCalFailure;
						calStatus.deflectionVoltage		= (unsigned short)CALCULATE_TU_CAL_SETPOINT;

						sendPrTUCalStatusFromISR( &calStatus ); 
						
						
						/* Keep config value */
						setPaperTakeupCurrent( config_.takeup_sensor_drive_current );
#if 1						
						PRINTF("takeupSensorCal(): _TakeupCalFailure: %d\r\n\r\n", max_delta );
						PRINTF("takeupSensorCal(): bias point: %d\r\n", engine.TUCalEmittermACurrent );
						PRINTF("takeupSensorCal(): deflection: %d\r\n\r\n", max_delta );
#endif						
					}
					
					/* Done with Calibration, go to idle */
					engine.TUCalState = IDLE_TU_CAL;
//debug					
//GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);		
				}
				
				
				break;
			}
			case IDLE_TU_CAL:
			{
				/*
				*	Return print engine ISR time to default
				*/
				setPrintEngineTimer( DEFAULT_ENGINE_COUNT );
				
				/* disable ADC averaging */
				clrTakeupFiltering();
				
				/* Set to return to idle */
                setOperation( IDLE_DIRECTIVE, &currentStatus );     				
				
				break;
			}
			
			default:
			break;
		}
    }

}

void dotCheckerPwrEnable() 
{
    GPIO_WritePinOutput(GPIO2, 5U, true); //dot checker power pin enable
}

void dotCheckerPwrDisable() 
{
    GPIO_WritePinOutput(GPIO2, 5U, false); //dot checker power pin disable
}

void activateStrobeCheckStart() 
{
    GPIO_PinWrite(GPIO3, 22U, true); //activate strobe, pull strobe high
    GPIO_PinWrite(GPIO3, 6U, true);
}

void activateStrobeCheckStop() 
{
    GPIO_PinWrite(GPIO3, 22U, false); //deactivate strobe, pull strobe low
    GPIO_PinWrite(GPIO3, 6U, false);
}

void LPSPI_MasterUserCallbackPrint(LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData)
{
    dotChecker.isTransferCompleted = true;
}

void initSpiPrinthead() 
{    
    LPSPI_MasterGetDefaultConfig( &dotChecker.masterConfig );
    LPSPI_MasterTransferCreateHandle(LPSPI4, &dotChecker.masterHandle, LPSPI_MasterUserCallbackPrint, NULL);
    
    dotChecker.masterConfig.baudRate                      = (1000000U * 14); //~12Mhz
    dotChecker.masterConfig.bitsPerFrame                  = 8U;
    dotChecker.masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;
    dotChecker.masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
    dotChecker.masterConfig.direction                     = kLPSPI_MsbFirst;
    dotChecker.masterConfig.whichPcs                      = kLPSPI_Pcs0;
    dotChecker.masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
    dotChecker.masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;
    dotChecker.masterConfig.betweenTransferDelayInNanoSec   = 10000U; 
    
    LPSPI_MasterInit(LPSPI4, &dotChecker.masterConfig, CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (7U + 1U));
    
    NVIC_SetPriority( LPSPI4_IRQn, 1 );
    
    CLOCK_SetMux(kCLOCK_LpspiMux, 1U);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, 7U);
}

void writePrintheadFrame(uint8_t * data) 
{
    dotChecker.isTransferCompleted = false;
    lpspi_transfer_t masterXfer;

    for(int i = 0; i < DATA_LENGTH_IN_BYTES; i++) 
    {
        dotChecker.txData[i] = data[i];
    }
    
    masterXfer.txData      = dotChecker.txData;
    masterXfer.rxData      = dotChecker.rxData;
    masterXfer.dataSize    = DATA_LENGTH_IN_BYTES; 
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    LPSPI_MasterTransferNonBlocking(LPSPI4, &dotChecker.masterHandle, &masterXfer);
}

void latchSetupTimer(uint32_t length) 
{
    GPT_StartTimer(GPT2);
    uint32_t timeA = GPT_GetCurrentTimerCount(GPT2);
    uint32_t timeB = timeA + length; //6000U ~25ns
    
    while(GPT_GetCurrentTimerCount(GPT2) < timeB) {};
    GPT_StopTimer(GPT2);
}

void latchHoldTimer(uint32_t length) 
{
    GPT_StartTimer(GPT2);
    uint32_t timeA = GPT_GetCurrentTimerCount(GPT2);
    uint32_t timeB = timeA + length; //6000U ~25ns
    
    while(GPT_GetCurrentTimerCount(GPT2) < timeB) {};
    GPT_StopTimer(GPT2);
}

void strobeHoldTimer(uint32_t length) 
{
    GPT_StartTimer(GPT2);
    uint64_t timeA = GPT_GetCurrentTimerCount(GPT2);
    uint64_t timeB = timeA + length; //6000U ~25ns
    
    while(GPT_GetCurrentTimerCount(GPT2) < timeB) {};
    GPT_StopTimer(GPT2);
}

void activateLatch(uint32_t setupTime, uint32_t holdTime) 
{  
    //latchSetupTimer(setupTime); 
    GPIO_PinWrite(GPIO2, 0U, false); //activate latch, pull latch low
    //latchHoldTimer(holdTime);
    delay_uS(25);
    GPIO_PinWrite(GPIO2, 0U, true); //deactivate latch, pull latch high
}

void startGPT2DotCheckTimer( uint32_t timerLength )
{ 
    //PRINTF("\r\nstartGPT2DotCheckTimer\r\n");
    
    setGPT2IntrType(DOT_CHECK_TIMER);

    /* initialize timer compare interrupts */    
    gpt_config_t gptConfig;
    
    /* obtain default configuration for general purpose timer module */
    GPT_GetDefaultConfig( &gptConfig );
   
    gptConfig.enableRunInWait = true; 
    gptConfig.enableRunInStop = true;
    gptConfig.enableRunInDoze = true;  
    gptConfig.enableRunInDbg = true;   
    gptConfig.enableFreeRun = false;

    /* initialize gpt module */
    GPT_Init( GPT2, &gptConfig );    
    GPT_SetClockDivider( GPT2, 32 );    
    
    /* load the compare registers with their initial values and setup the output 
       compare mode to toggle output on a match*/

    /* set channel 1 time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, timerLength ); 
    
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );

    /* enable channel interrupt flags. */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );     
    
    NVIC_SetPriority( GPT2_IRQn, 1 );
    EnableIRQ( GPT2_IRQn );
    
    GPT_StartTimer( GPT2 );
}

/******************************************************************************/
/*!    \fn unsigned char getHeadWearDotStatus( int x )

        \brief
        This function calculates if a dot is good bad or marginal.

        \author
        Chris King
*******************************************************************************/
HeadDotStatus getHeadWearDotStatus( int x )
{
    uint32_t dotStatus = getHeadWearDot(x);
    dotCheckerCalibratedAverage = config_.printheadResistance; 
    
    uint16_t dotResistanceBadHigh = dotCheckerCalibratedAverage + ((30.0f / 100.0f) * (dotCheckerCalibratedAverage));
    uint16_t dotResistanceBadLow = dotCheckerCalibratedAverage - ((30.0f / 100.0f) * (dotCheckerCalibratedAverage));
    uint16_t dotResistanceMarginalHigh = dotCheckerCalibratedAverage + ((20.0f / 100.0f) * (dotCheckerCalibratedAverage));
    uint16_t dotResistanceMarginalLow = dotCheckerCalibratedAverage - ((20.0f / 100.0f) * (dotCheckerCalibratedAverage));
    
    if(dotCheckerCalibratedAverage == 0)
    {
        dotResistanceBadHigh =  DOT_RESISTANCE_EXPECTED + ((30.0f / 100.0f) * (DOT_RESISTANCE_EXPECTED));
        dotResistanceBadLow =  DOT_RESISTANCE_EXPECTED - ((30.0f / 100.0f) * ( DOT_RESISTANCE_EXPECTED));
        dotResistanceMarginalHigh =  DOT_RESISTANCE_EXPECTED + ((20.0f / 100.0f) * (DOT_RESISTANCE_EXPECTED));
        dotResistanceMarginalLow =  DOT_RESISTANCE_EXPECTED - ((20.0f / 100.0f) * ( DOT_RESISTANCE_EXPECTED));
    }
    else
    {
        dotResistanceBadHigh = dotCheckerCalibratedAverage + ((30.0f / 100.0f) * (dotCheckerCalibratedAverage));
        dotResistanceBadLow = dotCheckerCalibratedAverage - ((30.0f / 100.0f) * (dotCheckerCalibratedAverage));
        dotResistanceMarginalHigh = dotCheckerCalibratedAverage + ((20.0f / 100.0f) * (dotCheckerCalibratedAverage));
        dotResistanceMarginalLow = dotCheckerCalibratedAverage - ((20.0f / 100.0f) * (dotCheckerCalibratedAverage));
    }
    
    
    
    //Higher counts is lower resistance
    if( dotStatus >= dotResistanceBadHigh || dotStatus <= dotResistanceBadLow )
    {
	return DOT_BAD;
    }
    else if( dotStatus >= dotResistanceMarginalHigh || dotStatus <= dotResistanceMarginalLow )
    {
	return DOT_MARGINAL;
    }
    else 
    {
	return DOT_GOOD;
    }
}

/******************************************************************************/
/*!     \fn unsigned char getHeadWearDot( int x )

        \brief
        This function returns the average value for the dot at the index X.

        \author
        Chris King
*******************************************************************************/
uint32_t getHeadWearDot( int x )
{  
    return dotChecker.dotResistanceValues[x];
}

/******************************************************************************/
/*!   \fn void dotWearOp( CmdOp *pOperation )
      \brief
        This function handles the dot wear command operation.
        
      \author
          Chris King
*******************************************************************************/              
void dotWearOp( CmdOp *pOperation )
{      
    if ( currentStatus.state != ENGINE_DOT_WEAR) 
    {
        //timing
        //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
        
        PRINTF("currentStatus.state: Entering DOT_WEAR!\r\n");    
        
        currentStatus.state = ENGINE_DOT_WEAR;    

        dotChecker.state = DOT_WEAR_INIT;
        
        PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 30U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );
    }
    else
    {
        switch( dotChecker.state ) 
        {
            case DOT_WEAR_INIT: 
            {   
                //PRINTF("DOT_WEAR_INIT\r\n");

                bool dotCheckerDataAvailable = false;
                if(dotChecker.readyToSend == true)
                {
                    dotCheckerDataAvailable = true;
                }
                
                memset( &dotChecker, 0, sizeof(dotChecker) );

                initSpiPrinthead();
                
                dotChecker.waitType =                   DOT_WEAR_PH_TRANSFER_WAIT;
                dotChecker.strobeWaitTime =             DOT_CHECKER_STROBE_WAIT_TIME;
                dotChecker.setupWaitTime =              DOT_CHECKER_SETUP_WAIT_TIME;
                dotChecker.dotCount =                   0;
                dotChecker.bitCount =                   0;
                dotChecker.byteCount =                  0;
                dotChecker.runCount =                   0;
                dotChecker.dataLengthInBytes =          DATA_LENGTH_IN_BYTES;
                dotChecker.isTransferCompleted =        false;
                dotChecker.isStrobeCompleted =          false;
                dotChecker.isSetupCompleted =           false;
               
                if(dotCheckerDataAvailable)
                {
                    dotChecker.readyToSend = true;
                }
                else
                {
                    dotChecker.readyToSend = false;
                }
                
                dotChecker.state = DOT_WEAR_WRITE_DOT_TO_PH;
                
                break;
            }
            case DOT_WEAR_WRITE_DOT_TO_PH: 
            {
                //PRINTF("DOT_WEAR_WRITE_DOT_TO_PH %d %d\r\n", dotChecker.byteCount, dotChecker.runCount);

                if(dotChecker.byteCount >= DATA_LENGTH_IN_BYTES)
                {
                    memset(dotChecker.printheadBuffer, 0, sizeof(dotChecker.printheadBuffer));

                    dotChecker.state = DOT_WEAR_RESET;
                }
                else
                {
                    //if we have written each bit in a byte
                    //reset the bit number and increment the byte count
                    if(dotChecker.bitCount == 8)
                    {
                        dotChecker.bitCount = 0;
                        dotChecker.byteCount++;
                    }
              
                    switch(dotChecker.bitCount)
                    {
                        case 0:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT0;
                            break;
                        case 1:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT1; 
                            break;
                        case 2:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT2; 
                            break;
                        case 3:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT3; 
                            break;
                        case 4:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT4; 
                            break;
                        case 5:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT5; 
                            break;
                        case 6:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT6; 
                            break;
                        case 7:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT7; 
                            break; 
                    }
                    
                    
                    
                    //increment the bit we are writing to the printhead
                    dotChecker.bitCount++;
                    
                    //increment the dot we are checking
                    dotChecker.dotCount++;
                    
                    
                    
                    
                    //clear SPI tx/rx buffers
                    memset(dotChecker.rxData, 0, sizeof(dotChecker.rxData));
                    memset(dotChecker.txData, 0, sizeof(dotChecker.txData));
                    
                    //write the printhead frame/buffer
                    writePrintheadFrame(dotChecker.printheadBuffer); 
                    
                    //goto dot checker wait state
                    dotChecker.waitType = DOT_WEAR_PH_TRANSFER_WAIT;
                    dotChecker.state = DOT_WEAR_WAIT;
                }

                break;
            }
            case DOT_WEAR_LATCH_PH: 
            {
                //PRINTF("DOT_WEAR_LATCH_PH\r\n");
                
                //cycle printhead latch ~10 microseconds
                activateLatch(LATCH_SETUP_TIME, LATCH_HOLD_TIME);
                
                dotChecker.state = DOT_WEAR_STROBE_PH;

                break;
            }
            case DOT_WEAR_STROBE_PH: 
            {
                //PRINTF("DOT_WEAR_STROBE_PH\r\n");
                
                //PRINTF("\r\nADC buff\r\n");
              
                //timing, 344 uS
                GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
              
                //turn on strobe
                dotCheckerPwrEnable();
                
                activateStrobeCheckStart();
                
                PIT_ClearStatusFlags( PIT, kPIT_Chnl_0, kPIT_TimerFlag );
                PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 30U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );

                //start dot checker timer for strobe length
                startGPT2DotCheckTimer( DOT_CHECKER_STROBE_WAIT_TIME );
                
                //goto dot checker wait state
                dotChecker.waitType = DOT_WEAR_STROBE_WAIT;
                dotChecker.state = DOT_WEAR_WAIT;
                
                break;
            }
            case DOT_WEAR_CALCULATE_DOT_RESISTANCE: 
            {
                //PRINTF("DOT_WEAR_CALCULATE_DOT_RESISTANCE");
                
                if(dotChecker.byteCount >= DATA_LENGTH_IN_BYTES)
                {
                    memset(dotChecker.printheadBuffer, 0, sizeof(dotChecker.printheadBuffer));
                    
                    activateStrobeCheckStop();
                    
                    dotCheckerPwrDisable();
                  
                    dotChecker.state = DOT_WEAR_RESET;
                }
                else
                {
                    //get printhead voltage ADC value
                    dotChecker.ADCBuffer = 0;
                    uint32_t buff = 0;
                    
                    dotChecker.ADCBuffer = getHeadVoltage();//ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 1U);
                    //buff = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 1U);
                    
                    //PRINTF("%d          %d\r\n", dotChecker.ADCBuffer, buff);
                    
                    //stop the printhead strobe
                    activateStrobeCheckStop();
                    
                    dotCheckerPwrDisable();
                    
                    //timing
                    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
                    
                    //calculate ADCBuff->dot resistance in ohms
                    uint32_t dotInOhms = -2.115 * dotChecker.ADCBuffer / (0.00045 * dotChecker.ADCBuffer - 1);
                    
                    //populate checkedDots array with current dot resistance
                    dotChecker.dotResistanceValues[(dotChecker.dotCount - 1)] += dotInOhms;
                    //dotChecker.dotResistanceValues[(dotChecker.dotCount - 1)] = dotInOhms;
                    
                    //clear dot checker printhead buffer 
                    memset(dotChecker.printheadBuffer, 0, sizeof(dotChecker.printheadBuffer));
                    
                    //goto dot checker wait state
                    dotChecker.waitType = DOT_WEAR_SETUP_WAIT;
                    dotChecker.state = DOT_WEAR_WAIT;
                    
                    //start dot checker timer for next dot start
                    startGPT2DotCheckTimer( DOT_CHECKER_SETUP_WAIT_TIME );
                }
              
                break;
            }
            case DOT_WEAR_RESET: 
            {
                //PRINTF("DOT_WEAR_RESET\r\n");
                
                dotChecker.runCount++;
                
                if(dotChecker.runCount == 1)
                {
                    memset( dotChecker.dotResistanceValues, 0, sizeof(dotChecker.dotResistanceValues) );
                }

                if(dotChecker.runCount >= DOT_CHECKER_SAMPLES_TO_TAKE)
                {
                    //dot checker is done, average our dot checker samples
                    for(uint16_t checkedDotsIndex = 0; checkedDotsIndex < getHeadStyleSize(); checkedDotsIndex++ )
                    {
                        dotChecker.dotResistanceValues[checkedDotsIndex] = ((dotChecker.dotResistanceValues[checkedDotsIndex] / (DOT_CHECKER_SAMPLES_TO_TAKE - 1)));
                        PRINTF("%d, ", dotChecker.dotResistanceValues[checkedDotsIndex]);
                        delay_uS(10);
                    }

                    sendDotWear(PRINTER_HEAD_SIZE_80MM);
                    
                    dotChecker.readyToSend = true; 
                
                    dotChecker.state = DOT_WEAR_EXIT;
                }
                else
                {
                    //reset dot checker for next sample
                    dotChecker.dotCount =                   0;
                    dotChecker.bitCount =                   0;
                    dotChecker.byteCount =                  0;
                    dotChecker.isTransferCompleted =        false;
                    dotChecker.isStrobeCompleted =          false;
                    dotChecker.isSetupCompleted =           false;
                    
                    dotChecker.state = DOT_WEAR_WRITE_DOT_TO_PH;
                }
                
                break;
            }
            case DOT_WEAR_EXIT: 
            {
                //PRINTF("DOT_WEAR_EXIT\r\n");
               
                dotCheckerPwrDisable();
                 
                PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 100U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );
                
                intializePrintHead( config_.contrast_adjustment, RT_SERVICE_80MM );
                initializePrintEngine( config_.contrast_adjustment, config_.out_of_media_count, getPrCommandQueueHandle() );

                //timing
                //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
                
                setOperation( IDLE_DIRECTIVE, &currentStatus );
         
                break;
            }
            case DOT_WEAR_WAIT: 
            {
                switch(dotChecker.waitType)
                {
                    case DOT_WEAR_PH_TRANSFER_WAIT:
                    {
                        //PRINTF("DOT_WEAR_PH_TRANSFER_WAIT\r\n");
                        
                        dotChecker.state = DOT_WEAR_WAIT;
                        
                        if(dotChecker.isTransferCompleted == true)
                        {
                            dotChecker.state = DOT_WEAR_LATCH_PH;
                        }
                        
                        break;
                    }
                    case DOT_WEAR_STROBE_WAIT:
                    {
                        //PRINTF("DOT_WEAR_STROBE_WAIT\r\n");
                        
                        dotChecker.state = DOT_WEAR_WAIT;
                        
                        if(dotChecker.isStrobeCompleted == true /* && dotChecker.dotRunCount >= 9*/)
                        {
                            //PRINTF("\r\n");
                          
                            dotChecker.state = DOT_WEAR_CALCULATE_DOT_RESISTANCE;
                        }
                        /*
                        else if(dotChecker.isStrobeCompleted == true && dotChecker.dotRunCount < 9)
                        {
                            dotChecker.ADCBuffer = getHeadVoltage();
                        }
                        */
                        
                        break;
                    }
                    case DOT_WEAR_SETUP_WAIT:
                    {
                        //PRINTF("DOT_WEAR_SETUP_WAIT\r\n");
                        
                        dotChecker.state = DOT_WEAR_WAIT;
                        
                        if(dotChecker.byteCount < DATA_LENGTH_IN_BYTES)
                        {
                            if(dotChecker.isSetupCompleted == true)
                            {
                                dotChecker.state = DOT_WEAR_WRITE_DOT_TO_PH;
                            }
                        }
                        else
                        {
                            if(dotChecker.isSetupCompleted == true)
                            {
                                dotChecker.state = DOT_WEAR_RESET;
                            }
                        }
                         
                        break;
                    }
                    default:
                    {
                        //PRINTF("UNKNOWN DOT WEAR WAIT STATE\r\n");
                        break;
                    }
                }
            }
            default:
            {
                //PRINTF("UNKNOWN DOT WEAR STATE %d\r\n", dotChecker.state);
                break;
            }
        }
    }
}

/******************************************************************************/
/*!   \fn void dotWearOp( CmdOp *pOperation )
      \brief
        This function handles the dot wear cal command operation.
        
      \author
          Chris King
*******************************************************************************/              
void dotWearCalOp( CmdOp *pOperation )
{      

    if ( currentStatus.state != ENGINE_DOT_WEAR) 
    {
        //timing
        //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
        
        PRINTF("currentStatus.state: Entering DOT_WEAR_CAL!\r\n");    
        
        //PRINTF("\r\n\r\n");
        
        PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 30U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );
        
        currentStatus.state = ENGINE_DOT_WEAR;    

        dotChecker.state = DOT_WEAR_INIT;
    }
    else
    {
        switch( dotChecker.state ) 
        {
            case DOT_WEAR_INIT: 
            {   
                //PRINTF("DOT_WEAR_INIT\r\n");

                bool dotCheckerDataAvailable = false;
                if(dotChecker.readyToSend == true)
                {
                    dotCheckerDataAvailable = true;
                }
                
                memset( &dotChecker, 0, sizeof(dotChecker) );
                
                initSpiPrinthead();
                
                dotChecker.waitType =                   DOT_WEAR_PH_TRANSFER_WAIT;
                dotChecker.strobeWaitTime =             DOT_CHECKER_STROBE_WAIT_TIME;
                dotChecker.setupWaitTime =              DOT_CHECKER_SETUP_WAIT_TIME;
                dotChecker.dotCount =                   0;
                dotChecker.bitCount =                   0;
                dotChecker.byteCount =                  0;
                dotChecker.runCount =                   0;
                dotChecker.dataLengthInBytes =          DATA_LENGTH_IN_BYTES;
                dotChecker.isTransferCompleted =        false;
                dotChecker.isStrobeCompleted =          false;
                dotChecker.isSetupCompleted =           false;
               
                if(dotCheckerDataAvailable)
                {
                    dotChecker.readyToSend = true;
                }
                else
                {
                    dotChecker.readyToSend = false;
                }
                
                dotChecker.state = DOT_WEAR_WRITE_DOT_TO_PH;
                
                break;
            }
            case DOT_WEAR_WRITE_DOT_TO_PH: 
            {
                //PRINTF("DOT_WEAR_WRITE_DOT_TO_PH %d %d\r\n", dotChecker.byteCount, dotChecker.runCount);

                if(dotChecker.byteCount >= DATA_LENGTH_IN_BYTES)
                {
                    memset(dotChecker.printheadBuffer, 0, sizeof(dotChecker.printheadBuffer));

                    dotChecker.state = DOT_WEAR_RESET;
                }
                else
                {
                    //if we have written each bit in a byte
                    //reset the bit number and increment the byte count
                    if(dotChecker.bitCount == 8)
                    {
                        dotChecker.bitCount = 0;
                        dotChecker.byteCount++;
                    }
              
                    switch(dotChecker.bitCount)
                    {
                        case 0:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT0;
                            break;
                        case 1:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT1; 
                            break;
                        case 2:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT2; 
                            break;
                        case 3:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT3; 
                            break;
                        case 4:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT4; 
                            break;
                        case 5:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT5; 
                            break;
                        case 6:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT6; 
                            break;
                        case 7:
                            dotChecker.printheadBuffer[dotChecker.byteCount] = BIT7; 
                            break; 
                    }
                    
                    //increment the bit we are writing to the printhead
                    dotChecker.bitCount++;
                    
                    //increment the dot we are checking
                    dotChecker.dotCount++;
                    
                    //clear SPI tx/rx buffers
                    memset(dotChecker.rxData, 0, sizeof(dotChecker.rxData));
                    memset(dotChecker.txData, 0, sizeof(dotChecker.txData));
                    
                    //write the printhead frame/buffer
                    writePrintheadFrame(dotChecker.printheadBuffer); 
                    
                    //goto dot checker wait state
                    dotChecker.waitType = DOT_WEAR_PH_TRANSFER_WAIT;
                    dotChecker.state = DOT_WEAR_WAIT;
                }

                break;
            }
            case DOT_WEAR_LATCH_PH: 
            {
                //PRINTF("DOT_WEAR_LATCH_PH\r\n");
                
                //cycle printhead latch ~10 microseconds
                activateLatch(LATCH_SETUP_TIME, LATCH_HOLD_TIME);
                
                dotChecker.state = DOT_WEAR_STROBE_PH;

                break;
            }
            case DOT_WEAR_STROBE_PH: 
            {
                //PRINTF("DOT_WEAR_STROBE_PH\r\n");
                
                //timing, 344 uS
                //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
              
                //turn on strobe
                dotCheckerPwrEnable();
                
                activateStrobeCheckStart();
                
                PIT_ClearStatusFlags( PIT, kPIT_Chnl_0, kPIT_TimerFlag );
                PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 30U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );

                //start dot checker timer for strobe length
                startGPT2DotCheckTimer( DOT_CHECKER_STROBE_WAIT_TIME );
                
                //goto dot checker wait state
                dotChecker.waitType = DOT_WEAR_STROBE_WAIT;
                dotChecker.state = DOT_WEAR_WAIT;
                
                break;
            }
            case DOT_WEAR_CALCULATE_DOT_RESISTANCE: 
            {
                //PRINTF("DOT_WEAR_CALCULATE_DOT_RESISTANCE");
                
                if(dotChecker.byteCount >= DATA_LENGTH_IN_BYTES)
                {
                    memset(dotChecker.printheadBuffer, 0, sizeof(dotChecker.printheadBuffer));
                    
                    activateStrobeCheckStop();
                    
                    dotCheckerPwrDisable();
                  
                    dotChecker.state = DOT_WEAR_RESET;
                }
                else
                {
                    //get printhead voltage ADC value
                    dotChecker.ADCBuffer = 0;
                    uint32_t buff = 0;
                    
                    dotChecker.ADCBuffer = getHeadVoltage();//ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 1U);
                    //buff = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 1U);
                    
                    //PRINTF("%d          %d\r\n", dotChecker.ADCBuffer, buff);
                    
                    //stop the printhead strobe
                    activateStrobeCheckStop();
                    
                    dotCheckerPwrDisable();
                    
                    //timing
                    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
                    
                    //calculate ADCBuff->dot resistance in ohms
                    uint32_t dotInOhms = -2.115 * dotChecker.ADCBuffer / (0.00045 * dotChecker.ADCBuffer - 1);
                    
                    //populate checkedDots array with current dot resistance
                    dotChecker.dotResistanceValues[(dotChecker.dotCount - 1)] += dotInOhms;
                    //dotChecker.dotResistanceValues[(dotChecker.dotCount - 1)] = dotInOhms;
                    
                    //clear dot checker printhead buffer 
                    memset(dotChecker.printheadBuffer, 0, sizeof(dotChecker.printheadBuffer));
                    
                    //goto dot checker wait state
                    dotChecker.waitType = DOT_WEAR_SETUP_WAIT;
                    dotChecker.state = DOT_WEAR_WAIT;
                    
                    //start dot checker timer for next dot start
                    startGPT2DotCheckTimer( DOT_CHECKER_SETUP_WAIT_TIME );
                }
              
                break;
            }
            case DOT_WEAR_RESET: 
            {
                //PRINTF("DOT_WEAR_RESET\r\n");
                
                dotChecker.runCount++;
                
                if(dotChecker.runCount == 1)
                {
                    memset( dotChecker.dotResistanceValues, 0, sizeof(dotChecker.dotResistanceValues) );
                }

                if(dotChecker.runCount >= DOT_CHECKER_SAMPLES_TO_TAKE)
                {
                    //dot checker is done, average our dot checker samples
                    for(uint16_t checkedDotsIndex = 0; checkedDotsIndex < getHeadStyleSize(); checkedDotsIndex++ )
                    {
                        dotChecker.dotResistanceValues[checkedDotsIndex] = ((dotChecker.dotResistanceValues[checkedDotsIndex] / (DOT_CHECKER_SAMPLES_TO_TAKE - 1)));
                        PRINTF("%d, ", dotChecker.dotResistanceValues[checkedDotsIndex]);
                        delay_uS(10);
                    }
                    
                    PRINTF("\r\n\r\n");
                    
                    dotCheckerCalibratedAverage = 0;
                    
                    for(uint16_t checkedDotsIndex = 0; checkedDotsIndex < getHeadStyleSize(); checkedDotsIndex++ )
                    {
                        dotCheckerCalibratedAverage += dotChecker.dotResistanceValues[checkedDotsIndex];
                        //PRINTF("%d, ", dotCheckerCalibratedAverage);
                        //delay_uS(10);
                    }
                    
                    dotCheckerCalibratedAverage = (dotCheckerCalibratedAverage / (uint16_t)getHeadStyleSize());
                    
                    PRINTF("dotCheckerCalibratedAverage = %d\r\n\r\n", dotCheckerCalibratedAverage);
                    
                    config_.printheadResistance = (unsigned short)dotCheckerCalibratedAverage;                    
                    
                    response.msgType = PR_CAL_PRINTHEAD_RESISTANCE_RESPONSE;
                    response.calValue = (short)dotCheckerCalibratedAverage;
                    
                    sendPrHeadCalResponse( &response );
                    
                    dotChecker.readyToSend = true; 
                    
                    PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 100U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );
                
                    dotChecker.state = DOT_WEAR_EXIT;
                }
                else
                {
                    //reset dot checker for next sample
                    dotChecker.dotCount =                   0;
                    dotChecker.bitCount =                   0;
                    dotChecker.byteCount =                  0;
                    dotChecker.isTransferCompleted =        false;
                    dotChecker.isStrobeCompleted =          false;
                    dotChecker.isSetupCompleted =           false;
                    
                    dotChecker.state = DOT_WEAR_WRITE_DOT_TO_PH;
                }
                
                break;
            }
            case DOT_WEAR_EXIT: 
            {
                //PRINTF("DOT_WEAR_EXIT\r\n");
               
                dotCheckerPwrDisable();
              
                intializePrintHead( config_.contrast_adjustment, RT_SERVICE_80MM );
                initializePrintEngine( config_.contrast_adjustment, config_.out_of_media_count, getPrCommandQueueHandle() );

                //timing
                //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
                
                setOperation( IDLE_DIRECTIVE, &currentStatus );
         
                break;
            }
            case DOT_WEAR_WAIT: 
            {
                switch(dotChecker.waitType)
                {
                    case DOT_WEAR_PH_TRANSFER_WAIT:
                    {
                        //PRINTF("DOT_WEAR_PH_TRANSFER_WAIT\r\n");
                        
                        dotChecker.state = DOT_WEAR_WAIT;
                        
                        if(dotChecker.isTransferCompleted == true)
                        {
                            dotChecker.state = DOT_WEAR_LATCH_PH;
                        }
                        
                        break;
                    }
                    case DOT_WEAR_STROBE_WAIT:
                    {
                        //PRINTF("DOT_WEAR_STROBE_WAIT\r\n");
                        
                        dotChecker.state = DOT_WEAR_WAIT;
                        
                        if(dotChecker.isStrobeCompleted == true)
                        {
                            dotChecker.state = DOT_WEAR_CALCULATE_DOT_RESISTANCE;
                        }
                        
                        break;
                    }
                    case DOT_WEAR_SETUP_WAIT:
                    {
                        //PRINTF("DOT_WEAR_SETUP_WAIT\r\n");
                        
                        dotChecker.state = DOT_WEAR_WAIT;
                        
                        if(dotChecker.byteCount < DATA_LENGTH_IN_BYTES)
                        {
                            if(dotChecker.isSetupCompleted == true)
                            {
                                dotChecker.state = DOT_WEAR_WRITE_DOT_TO_PH;
                            }
                        }
                        else
                        {
                            if(dotChecker.isSetupCompleted == true)
                            {
                                dotChecker.state = DOT_WEAR_RESET;
                            }
                        }
                         
                        break;
                    }
                    default:
                    {
                        //PRINTF("UNKNOWN DOT WEAR WAIT STATE\r\n");
                        break;
                    }
                }
            }
            default:
            {
                //PRINTF("UNKNOWN DOT WEAR STATE %d\r\n", dotChecker.state);
                break;
            }
        }
    }
}

/******************************************************************************/
/*!   \fn void disableOp( CmdOp *pOperation )
      \brief
        This function handles the teach table disable command operation.
        
      \author
          Aaron Swift
*******************************************************************************/              
void disableOp( CmdOp *pOperation )
{
              
    if ( currentStatus.state != ENGINE_DISABLED) {
        PRINTF("currentStatus.state: Entering ENGINE_DISABLE!\r\n" );
      /* Set the status bits to show that the printer has      
           just returned to the idle state after the completion  
           one of the commands. The initial command bits that    
           made the printer leave printer idle are still set at  
           this time to indicate what the command was that was   
           just completed. */            
        currentStatus.state = ENGINE_DISABLED;           
        currentStatus.command |= COMMAND_COMPLETE;
        
        /* notify host the printer is disabled. */
        compareStatus( &currentStatus, &prevStatus );
        
        /* perform some initialization. */        
        currentStatus.command &= ~COMMAND_COMPLETE;
        currentStatus.user    = 0;
        
        shutdownPrintEngine();
    }
    
    /* clear the command queue */
    clearCmdQueue();
}

/******************************************************************************/
/*!   \fn void cutOp( WaitOperation *pOperation )
      \brief
        This function signals the cutter to cut the label
      \author
          Eric Landes
*******************************************************************************/
void cutOp( GenericOperation *pOperation )
{  
    if( currentStatus.state != ENGINE_CUTTING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_CUTTING!\r\n" );
      
        currentStatus.state = ENGINE_CUTTING;
        /* tell the Printer task to cut */
        if(pCutSemaphore != NULL){
            /* make sure the semaphore is empty */
            xSemaphoreTakeFromISR(pCutDoneSemaphore,pdFALSE);
            /* called from an ISR so user the ISR give */
            if(xSemaphoreGiveFromISR( pCutSemaphore, pdFALSE) != pdTRUE){
                PRINTF("cutOp(): Failed to give cutter Semaphore");
            }
        }     
    } else {           
        /* wait until the cutter is done */
        if(xSemaphoreTakeFromISR(pCutDoneSemaphore,pdFALSE) == pdTRUE){
            setNextOperation( &currentStatus );
        }
    }
}



/******************************************************************************/
/*!   \fn void stepTakeupOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table test of paper takeup command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepTakeupOp( StepUntilOperation *pOperation )
{
    static int index_ = 0;
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING_UNTIL_TAKEUP!\r\n" );
                
        currentStatus.state = ENGINE_STEPPING;
                  
        powerOnStepper();

        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        
        engine.direction = BACKWARD_;
        /* 10" of label travel into steps 
        engine.numSteps = 10 * 203; */
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps ); */
        /* initializeStepper( engine.direction ); */
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
              
            if(getTakingUpPaper() == true)
            {   
                //stepTakeUpMotor();         	 
            }
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        unsigned short torque = getTakeUpTorque();
        PRINTF( "torque: %d\r\n",  torque );
        currentStatus.error = NO_ERROR;
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if( torque <= 2500 ) {
        /*
        else if ( ( testCondition( &currentStatus, 
                  pOperation->operator, 
                  pOperation->bits, 
                  pOperation->result) == false) 
                  &&  ( --engine.numSteps > 0 ) ) {    */

            int even = 0;
            even = engine.numSteps & 0x0001;
            if(getTakingUpPaper() == true)
            {
                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                
                engine.steps++;
                //stepTakeUpMotor(); 
            }
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {                                  
            PRINTF( "torque: %d\r\n",  torque );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper();

            setNextOperation( &currentStatus ); 
        }
    }	
}

/******************************************************************************/
/*!   \fn void clearCmdQueue( void )
      \brief
        This function clears the printer command queue.
        
      \author
          Aaron Swift
*******************************************************************************/              
void clearCmdQueue( void )
{
    /* clear the command queue */
    xQueueReset( pCmdQHandler_ );  
}

/******************************************************************************/
/*!   \fn void clearLabelImageBuffer( void )
      \brief
        This function clears the printer label image buffer.
        
      \author
          Aaron Swift
*******************************************************************************/              
void clearLabelImageBuffer( void )
{
    unsigned long buffSize = 0;
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        buffSize = PRINTER_BUFFER_SIZE_72MM;
    }
    else
    {
        buffSize = PRINTER_BUFFER_SIZE_80MM;
    }
  
  
    unsigned char *pBuffer = getImageBuffer();
    if( pBuffer != NULL) {
        memset( pBuffer, 0, buffSize );    
    } else {
        PRINTF("clearLabelImageBuffer(): Error: pBuffer is null!\r\n" );       
    }
}

/******************************************************************************/
/*!   \fn void setContinuousStock( void )
      \brief
        This function set the flag for continuous stock. This flag signals use 
        when to use tracking for adjusting for die cut labels slip and stalling
        which is not important on continuous stock.

      \author
          Aaron Swift
*******************************************************************************/              
void setContinuousStock( void )
{
    continuousStock_ = true; 
    //sizingLabels = false;
}

/******************************************************************************/
/*!   \fn void clrContinuousStock( void )
      \brief
        This function clears the flag for continuous stock. This flag signals use 
        when to use tracking for adjusting for die cut labels slip and stalling
        which is not important on continuous stock.

      \author
          Aaron Swift
*******************************************************************************/              
void clrContinuousStock( void )
{
    continuousStock_ = false;
    //sizingLabels = true;
}

void printerTests()
{
    static int testCnt_ = 1; 
    
    /****************** test print engine command operation *******************/
    #if 1
    QueueHandle_t pQHandler_ = NULL; //= getPrCommandQueueHandle();
    if( pQHandler_ != 0 ) {
        switch( testCnt_ ) 
        {
            /* test sizing label */
            case 1: {
                if( engine.currentCmd.generic.directive == NOOPERATION_DIRECTIVE ) {
                    PrCommand cmdMsg;
                    cmdMsg.identifier = _TBL_SIZE_LABELS;  /* size command */
                    cmdMsg.options = 0;             /* no security label */ 
                    cmdMsg.data_item = 0;           /* no indirection */ 
                    cmdMsg.value = 0;               

                    BaseType_t result = xQueueSendToBack( pQHandler_, (void *)&cmdMsg, 0 );
                    if( result == pdTRUE ) {
                        testCnt_++;
                    } else {
                        PRINTF("printerTests(): PR Command Queue full! \r\n");  
                    }
                } else {
                    PRINTF("printerTests(): PR Engine busy! \r\n");  
                }
                break;
            }
            case 2: {
                /* wait for sizing to complete */
                if( currentStatus.state == ENGINE_IDLE ) {
                    testCnt_++;
                }
                break;
            }
            case 3: {
                break;
            }
            default : {
                break;
            }
        }
    } else {
        PRINTF("printerTests(): command queque null!\r\n" );  
    } 
    #endif
}


int getOutOfMediaCounts( void ){
    return engine.outOfMediaCnt;
}


void setOutOfMediaCounts(int val){
    engine.outOfMediaCnt = val;
}


int getMaxOutOfMediaCounts( void ){
    return engine.maxMediaCount;
}


void setHistoryEnabled(bool enabled){
    historyEnabled_ = enabled;
}


bool getUsingContinuous( void )
{
    if(continuousStock_ == true)
    {
        return true;
    }
    else 
    {
        return false;
    }
}

bool getSizingLabels( void )
{
    if(sizingLabels == true)
    {
        return true;
    }
    else 
    {
        return false;
    }
}


void setSizingLabels( bool sizing )
{
    if(sizing == true)
    {
        sizingLabels = true;
    }
    else 
    {
        sizingLabels = false;
    }
}


bool getIDF2( void )
{
    if(GPIO_PinRead(GPIO1, 8U) == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
}


bool getBackwindAfterSizing( void )
{
    return backwindAfterSizing;
}


void setBackwindAfterSizing(bool backwind)
{
    backwindAfterSizing = backwind;
}


bool getBackwindAfterSizingDone( void )
{
    return backwindAfterSizingDone;
}


void setBackwindAfterSizingDone(bool backwind)
{
    backwindAfterSizingDone = backwind;
}


uint32_t getLTWaitCount_( void )
{
    return LTWaitCount_;
}


void setLTWaitCount_( uint32_t waitCount )
{
    LTWaitCount_ = waitCount;
}


bool getExpelDone( void )
{
    return expelDone;
}


int getShootIndex( void )
{
    return shootIndex;
}


void setShootIndex( int index )
{
    shootIndex = index;
}


uint16_t calculateSizingBackwindSteps( void )
{
    if(getTakingUpPaper() == false)
    {
        if(getLargeGapFlag() == true && getSyncBarFlag() == false)
        {
            //PRINTF("HT STOCK SIZING BACKWIND - NO PAPER\r\n");
            return (161);
        }
        else if(getSyncBarFlag() == true)
        {
            //PRINTF("HT SYNC BAR STOCK SIZING BACKWIND - NO PAPER\r\n");
            return (161);
        }
        else
        { 
            //PRINTF("GT STOCK SIZING BACKWIND - NO PAPER \r\n");
            return (153);
        }
    }
    else
    {
        if(getLargeGapFlag() == true && getSyncBarFlag() == false)
        {            
            //PRINTF("HT STOCK SIZING BACKWIND \r\n");
            return (212 - getIndirectData( 4 ));
        }
        else if(getSyncBarFlag() == true)
        {
            //PRINTF("HT SYNC BAR STOCK SIZING BACKWIND\r\n");
            return (212 - getIndirectData( 4 ));
        }
        else
        { 
            if(getLabelSizeInQuarterSteps() <= 740 && getLabelSizeInQuarterSteps() >= 680) 
            {
                //PRINTF("FIRST BACKWIND FOR 1.75 inch label no paper\r\n");
                return (175 - getIndirectData( 4 ));
            }
            else
            {
                //PRINTF("GT STOCK SIZING BACKWIND\r\n");
                return (175 - getIndirectData( 4 ));
            }
        }
    }  
}

uint16_t calculateStreamingBackwindSteps( void )
{   
    if(TPHStepsPastGapThisPrint > 0 )
    {
       if(getLargeGapFlag() == true) //HT backwind
        {
            if(TPHStepsPastGapThisPrint >= 330)
            {
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 330); 
                return streamingLabelBackwind;
            }
            else
            {
                streamingLabelBackwind = (330 - TPHStepsPastGapThisPrint); 
                return streamingLabelBackwind;
            }   
        }
        else //GT backwind
        {
            if(TPHStepsPastGapThisPrint >= 340) 
            {
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 340);
                return streamingLabelBackwind;
            }
            else
            {
                streamingLabelBackwind = (340 - TPHStepsPastGapThisPrint);
                return streamingLabelBackwind;
            }
        }
    }
    else
    {
        return 0; 
    }
}

uint16_t calculatePeelingBackwindSteps( void )
{          
    if(getLargeGapFlag() == true) //HT backwind
    {
        if(TPHStepsPastGapThisPrint >= 248) 
        {
            if(getIndirectData( 4 ) >= 34)
            {
                return (TPHStepsPastGapThisPrint - 248) - (getIndirectData( 4 ) - 34);
            }
            else
            {
                return (TPHStepsPastGapThisPrint - 248) + (34 - getIndirectData( 4 ));
            }
        }
        else
        {
            if(getIndirectData( 4 ) >= 34)
            {
                return (248 - TPHStepsPastGapThisPrint) - (getIndirectData( 4 ) - 34);
            }
            else
            {
                return (248 - TPHStepsPastGapThisPrint) + (34 - getIndirectData( 4 ));
            }
        }
    }
    else //GT backwind
    {
        if(TPHStepsPastGapThisPrint >= 316)
        {
            if(getIndirectData( 4 ) >= 34)
            { 
                return (TPHStepsPastGapThisPrint - 316) - (getIndirectData( 4 ) - 34);
            }
            else
            { 
                return (TPHStepsPastGapThisPrint - 316) + (34 - getIndirectData( 4 ));
            }
        }
        else
        {
            if(getIndirectData( 4 ) >= 34)
            { 
                return (316 - TPHStepsPastGapThisPrint) - (getIndirectData( 4 ) - 34);
            }
            else
            {
                return (316 - TPHStepsPastGapThisPrint) + (34 - getIndirectData( 4 ));
            }
        }
    }
}


uint16_t calculateHTLeadInTarget(void) 
{  
    static const LabelRange HTlabelRanges[] = 
    {
        {680,  900,  20,  121},
        {901,  1000, 36,  126},
        {1001, 1299, 40,  131},
        {1300, 1499, 44,  136},
        {1500, 1699, 46,  142},
        {1700, 1900, 48,  146},
        {1901, 2060, 50,  151},
        {2061, 2260, 56,  151},
        {2261, 2500, 60,  154},
        
        //6.5
        {2501, 2680, 62,  156},
        
        {2681, 2860, 66, 159},
        {2861, 3070, 70, 162},
        {3071, 3300, 74, 164},
        {3301, 3490, 76, 166},
        {3491, 3690, 80, 170},
        {3691, 3890, 82, 172},
        {3891, 5000, 82, 172},
    };

    uint16_t labelSize = getLabelSizeInQuarterSteps();
    bool isTakingPaper = getTakingUpPaper();

    for (size_t i = 0; i < sizeof(HTlabelRanges) / sizeof(LabelRange); ++i) 
    {
        if (labelSize >= HTlabelRanges[i].min && labelSize <= HTlabelRanges[i].max) 
        {
            //PRINTF("\r\nHT LEAD IN - paper %d - no paper %d\r\n", HTlabelRanges[i].paperValue, HTlabelRanges[i].noPaperValue);
          
            return isTakingPaper ? HTlabelRanges[i].paperValue : HTlabelRanges[i].noPaperValue;
        }
    }

    // Default values for unmatched label length
    return isTakingPaper ? 110 : 74;
}


uint16_t calculateGTLeadInTarget( void )
{
    static const LabelRange GTlabelRanges[] = 
     {
        {680,  900,  0,   36},
        {901,  1000, 2,   40},
        {1001, 1299, 4,   42},
        {1300, 1499, 10,   42},
        {1500, 1699, 14,   44},
        {1700, 1900, 16,   48},
        {1901, 2060, 20,   52},
        {2061, 2260, 24,   56},
        {2261, 2500, 28,   60},
        {2501, 2680, 30,   64},
        {2681, 2860, 34,   64},
        {2861, 3070, 38,   64},
        {3071, 3300, 44,   72},
        {3301, 3490, 48,   72},
        {3491, 3690, 52,   72},
        {3691, 3890, 52,   76},
        {3891, 5000, 58,   78},
    };

    uint16_t labelSize = getLabelSizeInQuarterSteps();
    bool isTakingPaper = getTakingUpPaper();

    for (size_t i = 0; i < sizeof(GTlabelRanges) / sizeof(LabelRange); ++i) 
    {
        if (labelSize >= GTlabelRanges[i].min && labelSize <= GTlabelRanges[i].max) 
        {
            //PRINTF("\r\nGT LEAD IN - paper %d - no paper %d\r\n", GTlabelRanges[i].paperValue, GTlabelRanges[i].noPaperValue);
          
            return isTakingPaper ? GTlabelRanges[i].paperValue : GTlabelRanges[i].noPaperValue;
        }
    }

    // Default values for unmatched label length
    return isTakingPaper ? 60 : 26;
}

/******************************************************************************/
/*!   \fn 
        
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void setStreamingLabelBackwind( uint16_t steps )
{
    streamingLabelBackwind = steps;
    
    //PRINTF("setStreamingLabelBackwind() - steps = %d\r\n", steps);
}


/******************************************************************************/
/*!   \fn 
        
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void setTPHStepsPastGapThisPrint(uint16_t steps)
{
    TPHStepsPastGapThisPrint = steps;
    
    //PRINTF("setTPHStepsPastGapThisPrint() - steps = %d\r\n", steps);
}


/******************************************************************************/
/*!   \fn 
        
      \brief  
        
      \author
          Chris King
*******************************************************************************/
uint16_t getTPHStepsPastGapThisPrint( void )
{
    //PRINTF("getTPHStepsPastGapThisPrint() - steps = %d\r\n", TPHStepsPastGapThisPrint);
    
    return TPHStepsPastGapThisPrint;
}


/******************************************************************************/
/*!   \fn 
        
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void setSizingState( char state )
{
    sizingState = state;
}


/******************************************************************************/
/*!   \fn 
        
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void setGapCurrentToSeventyFivePercent( bool status )
{
    if(status == true)
    {   
        if(config_.media_sensor_adjustment == 0)
        { 
            //PRINTF("setGapCurrentToSeventyFivePercent() - adjustment == 0 - setting current to %d\r\n", 20);
          
            setGapCurrent(20);
        }
        else
        {
            //PRINTF("setGapCurrentToSeventyFivePercent() - current == %d, setting current to %d\r\n", config_.media_sensor_adjustment, (char)(config_.media_sensor_adjustment * 0.75));
            
            setGapCurrent((char)(config_.media_sensor_adjustment * 0.75));
        }
    }
    else
    {
        //PRINTF("setGapCurrentToSeventyFivePercent() - setting current to %d for printing/sizing\r\n", (config_.media_sensor_adjustment));
      
        setGapCurrent(config_.media_sensor_adjustment);
    }
}


void setTUSlip(bool status)
{
    TUSlip = status;
}


bool getTUSlip( void )
{
    return TUSlip;
}


void setPrintingStatus(bool status)
{
    printingStatus = status;
    
    //PRINTF("setPrintingStatus() == %d\r\n", status);
}


bool getPrintingStatus( void )
{
    return printingStatus;
}


void setSizingStatus(bool status)
{
    sizingStatus = status;
    
    //PRINTF("setSizingStatus() == %d\r\n", status);
}


bool getSizingStatus( void )
{
    return sizingStatus;
}


void setCanceledSizingFlag(bool status)
{
    canceledSizingFlag = status;
    
    PRINTF("setCanceledSizingFlag() == %d\r\n", status);
}


bool getCanceledSizingFlag( void )
{
    return canceledSizingFlag;
}

/******************************************************************************/
/*!   \fn bool getLeadInDone( void )
        
      \brief  
          Getter function
      \author
          Tom Fink
*******************************************************************************/
bool getLeadInDone( void )
{
    return leadInDone;
}

/******************************************************************************/
/*!   \fn int getTempAtStart( void )
        
      \brief  
          Getter function
      \author
          Tom Fink
*******************************************************************************/
int getTempAtStart( void )
{
    return tempAtStart;
}


bool checkForOutOfMedia( void )
{
    setGapCurrentToSeventyFivePercent(true);
        
    if(getGapCalStatus() == false && getPrintingStatus() == false && getSizingStatus() == false)
    {        
        delay_uS(110); //adcEtc0HandleIsr() runs every 100uS 
        
        char OutOfMediaCount = 0; 
        
        for(unsigned int i = 0; i < 100; i++) 
        {
           delay_uS(110);
          
           if(adcManager.value[CHANNEL_SHOOT] < OUT_OF_MEDIA_THRESHOLD)
           {
              OutOfMediaCount++;
           }
           
           
           //PRINTF("CHANNEL_SHOOT == %d\r\n", adcManager.value[CHANNEL_SHOOT]);
        }  
        
        if(OutOfMediaCount >= 90)
        {
            OutOfMedia = true;
        }
        else
        {
            OutOfMedia = false;
        }
    }
    else
    {
        OutOfMedia = false;
    }
    
    if(OutOfMedia == true)  
    {
        if((currentStatus.sensor & OUT_OF_MEDIA) != OUT_OF_MEDIA && getHeadUp() == false)
        {
            //PRINTF("OOM status SET in ENTERING idle OP\r\n");
        }
        
        if(getHeadUp() == false)
        {
            //OOS, set low label min
            if(lowLabelStatus.averagedSensorFeedback != 0)
            {
                if(getTakingUpPaper() == true)
                {
                    if(lowLabelStatus.averagedSensorFeedback >= LOW_LABEL_MIN_PEELING_DEFAULT_MIN)
                    {
                        lowLabelStatus.segmentLengthMinPeeling = lowLabelStatus.averagedSensorFeedback;
                  
                        sendLowLabelMinMax(lowLabelStatus.segmentLengthMinPeeling, lowLabelStatus.segmentLengthMaxPeeling, lowLabelStatus.segmentLengthMinStreaming, lowLabelStatus.segmentLengthMaxStreaming);
                        setLowLabelPeelingMinFromHost(lowLabelStatus.segmentLengthMinPeeling);
                    }
                    else
                    {
                        lowLabelStatus.segmentLengthMinPeeling = LOW_LABEL_MIN_PEELING_DEFAULT_MIN;
                  
                        sendLowLabelMinMax(lowLabelStatus.segmentLengthMinPeeling, lowLabelStatus.segmentLengthMaxPeeling, lowLabelStatus.segmentLengthMinStreaming, lowLabelStatus.segmentLengthMaxStreaming);
                        setLowLabelPeelingMinFromHost(lowLabelStatus.segmentLengthMinPeeling);
                    }
                }
                else
                {
                    if(lowLabelStatus.averagedSensorFeedback >= LOW_LABEL_MIN_STREAMING_DEFAULT_MIN)
                    {
                        lowLabelStatus.segmentLengthMinStreaming = lowLabelStatus.averagedSensorFeedback;
                  
                        sendLowLabelMinMax(lowLabelStatus.segmentLengthMinPeeling, lowLabelStatus.segmentLengthMaxPeeling, lowLabelStatus.segmentLengthMinStreaming, lowLabelStatus.segmentLengthMaxStreaming);
                        setLowLabelStreamingMinFromHost(lowLabelStatus.segmentLengthMinStreaming);
                    }
                    else
                    {
                        lowLabelStatus.segmentLengthMinStreaming = LOW_LABEL_MIN_STREAMING_DEFAULT_MIN;
                  
                        sendLowLabelMinMax(lowLabelStatus.segmentLengthMinPeeling, lowLabelStatus.segmentLengthMaxPeeling, lowLabelStatus.segmentLengthMinStreaming, lowLabelStatus.segmentLengthMaxStreaming);
                        setLowLabelStreamingMinFromHost(lowLabelStatus.segmentLengthMinStreaming);
                    }
                }
            }
            
            currentStatus.sensor |= OUT_OF_MEDIA;
            setHeadPower( false );           
        }
        else
        {
            currentStatus.sensor &= ~OUT_OF_MEDIA;
        }    
    }
    else
    {
        if((currentStatus.sensor & OUT_OF_MEDIA) == OUT_OF_MEDIA && getHeadUp() == false)
        {
            //PRINTF("OOM status REMOVED in ENTERING idle OP\r\n");
        }

        currentStatus.sensor &= ~OUT_OF_MEDIA;
    }
    
    setGapCurrentToSeventyFivePercent(false);
    
    compareStatus( &currentStatus, &prevStatus );
    
    //PRINTF("OOM = %d\r\n", OutOfMedia);
    
    return OutOfMedia;
}


void setStartOfQueue( bool start )
{
    startOfQueue = start;
}


bool getStartOfQueue( void )
{
    return startOfQueue;
}


void setStreamingLeadInMod( uint16_t steps )
{
    streamingLeadInMod = steps;
}


uint16_t getStreamingLeadInMod( void )
{
    return streamingLeadInMod;
}


void setStreamingExpelMod( uint16_t steps )
{
    streamingExpelMod = steps;
    
    if( getTakingUpPaper() == true )
    {
        if(getLargeGapFlag() == true || getCutterInstalled_() == true)
        {
            streamingExpelMod += 210;
        }
        else
        {
            streamingExpelMod += 160;
        }  
    }
    else
    {
        if(getLargeGapFlag() == true || getCutterInstalled_() == true)
        {
            streamingExpelMod += 210;
        }
        else
        {
            streamingExpelMod += 160;
        }  
    }
}


uint16_t getStreamingExpelMod( void )
{
    return streamingExpelMod;
}


bool getFirstPrint( void )
{
    return firstPrint;
}

uint16_t getLabelPauseTimeout( void )
{
    return labelPauseTimeout;
}

void setLabelPauseTimeout( uint16_t timeout )
{
    labelPauseTimeout = timeout;
}

void updateLowLabelStatus( void )
{    
    //get sensor feedback
    lowLabelStatus.ADCSample = getLowStockSensor();
  
    int sensorFeedbackDelta = lowLabelStatus.sensorFeedbackMax - lowLabelStatus.sensorFeedbackMin;
  
    if(lowLabelStatus.thresholdSet == false && sensorFeedbackDelta >= 400 && lowLabelStatus.motorSteps > 1400)
    {
        lowLabelStatus.samplePointIndex = 0;
        lowLabelStatus.lastMotorSteps = 0;
        lowLabelStatus.motorSteps = 0;
        lowLabelStatus.consecutiveSamplesAboveThreshold = 0;
        lowLabelStatus.consecutiveSamplesBelowThreshold = 0;
        
        lowLabelStatus.samplesCollected = 0;
        
        lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = 0;
        lowLabelStatus.averagedSensorFeedback = 0;
        
        currentStatus.labelLowPercentage = 0;
        
        lowLabelStatus.estimatedLabelCount = 750;
        
        lowLabelStatus.numberOfLabelsOnFullRoll = 750;
        lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining = 0;

        for(char i = 0; i < (LABEL_LOW_SAMPLE_COUNT - 1); i++)
        {
            lowLabelStatus.samplePoint[i] = 0;
        }
        
        setLabelLowThreshold((lowLabelStatus.sensorFeedbackMin + lowLabelStatus.sensorFeedbackMax) / 2);
      
        lowLabelStatus.thresholdSet = true;
    }

    if(lowLabelStatus.ADCSample < lowLabelStatus.sensorFeedbackMin && lowLabelStatus.ADCSample > 0)
    {
        lowLabelStatus.sensorFeedbackMin = lowLabelStatus.ADCSample;
        
        setLabelLowThreshold((lowLabelStatus.sensorFeedbackMin + lowLabelStatus.sensorFeedbackMax) / 2);
    }
    
    if(lowLabelStatus.ADCSample > lowLabelStatus.sensorFeedbackMax)
    {
        lowLabelStatus.sensorFeedbackMax = lowLabelStatus.ADCSample;
        
        setLabelLowThreshold((lowLabelStatus.sensorFeedbackMin + lowLabelStatus.sensorFeedbackMax) / 2);
    }
    
    //increment motor step counter
    lowLabelStatus.motorSteps++;
  
    //we are below our threshold
    if( lowLabelStatus.ADCSample <= getLabelLowThreshold())
    {
        //reset samples above threshold counter
        lowLabelStatus.consecutiveSamplesAboveThreshold = 0;
        
        //keep counting until we are sure we are on the other side of the threshold
        if(lowLabelStatus.consecutiveSamplesBelowThreshold != LABEL_LOW_POINT_RECORD_THRESHOLD_CROSSED)
        {
            lowLabelStatus.consecutiveSamplesBelowThreshold++;
        }

        //we've gone far enough, record a sample point
        if(lowLabelStatus.consecutiveSamplesBelowThreshold == LABEL_LOW_POINT_RECORD_THRESHOLD)
        {
            //we have crossed the threshold
            lowLabelStatus.consecutiveSamplesBelowThreshold = LABEL_LOW_POINT_RECORD_THRESHOLD_CROSSED;

            //record a sample point
            lowLabelStatus.samplePoint[lowLabelStatus.samplePointIndex] = lowLabelStatus.motorSteps;

            //increment sample index
            lowLabelStatus.samplePointIndex++;
        }
    }
    else //we are above our threshold
    {
        
      
        //reset samples below threshold counter
        lowLabelStatus.consecutiveSamplesBelowThreshold = 0;

        //keep counting until we are sure we are on the other side of the threshold
        if(lowLabelStatus.consecutiveSamplesAboveThreshold != LABEL_LOW_POINT_RECORD_THRESHOLD_CROSSED)
        {
            lowLabelStatus.consecutiveSamplesAboveThreshold++;
        }

        //we've gone far enough, record a sample point
        if(lowLabelStatus.consecutiveSamplesAboveThreshold == LABEL_LOW_POINT_RECORD_THRESHOLD)
        {
             //we have crossed the threshold
            lowLabelStatus.consecutiveSamplesAboveThreshold = LABEL_LOW_POINT_RECORD_THRESHOLD_CROSSED;

            //record a sample point
            lowLabelStatus.samplePoint[lowLabelStatus.samplePointIndex] = lowLabelStatus.motorSteps;
            
            //increment sample index
            lowLabelStatus.samplePointIndex++; 
        }
    }
    
    
    
    //we have enough sample points collected to find the two largest segments (sample point to next sample point) to average together
    if(lowLabelStatus.samplePointIndex >= (LABEL_LOW_SAMPLE_COUNT - 1))
    {   
        uint16_t segmentA = 0;
        uint16_t segmentB = 0;
        uint16_t pointAStart = 0;
        uint16_t pointAEnd = 0;
        uint16_t pointBStart = 0;
        uint16_t pointBEnd = 0;
        lowLabelStatus.averagedSensorFeedback = 0;
       
        //find the two largest segments, starting from the second recorded threshold crossing 
        for (int i = 0; i < (LABEL_LOW_SAMPLE_COUNT - 1); i++) 
        {
            //calc segment length
            int distanceBetweenPoints = lowLabelStatus.samplePoint[i + 1] - lowLabelStatus.samplePoint[i] - 1;
            
            if(distanceBetweenPoints > segmentA) //biggest segment
            {
                // shift current largest into second largest
                segmentB = segmentA;
                pointBStart = pointAStart;
                pointBEnd = pointAEnd;

                // update largest
                segmentA = distanceBetweenPoints;
                pointAStart = lowLabelStatus.samplePoint[i];
                pointAEnd = lowLabelStatus.samplePoint[i + 1];
            }
            else if(distanceBetweenPoints > segmentB) //second largest segment
            {
                segmentB = distanceBetweenPoints;
                pointBStart = lowLabelStatus.samplePoint[i];
                pointBEnd = lowLabelStatus.samplePoint[i + 1];
            }
        }       
        
        for(int i = 0; i < (LABEL_LOW_SAMPLE_COUNT - 1); i++)
        {
          //PRINTF("segment: %d length: %d\r\n", i, lowLabelStatus.samplePoint[i + 1] - lowLabelStatus.samplePoint[i] - 1);
        }
        
        //if this is the first time we have calculated a low label sensor sample average
        if(lowLabelStatus.lastMotorSteps == 0)
        {
            //average the two largest segments 
            lowLabelStatus.averagedSensorFeedback = (segmentB + segmentA) / 2;
            //lowLabelStatus.averagedSensorFeedback = segmentA;
        }
        else //we already have a segment average
        {
            //average the two largest segments 
            int segmentAverage = (segmentB + segmentA) / 2;
            //int segmentAverage = segmentA;
            
            //average the last segment avergage with the current one
            lowLabelStatus.averagedSensorFeedback = (segmentAverage + lowLabelStatus.lastMotorSteps) / 2;
        }
        
        
        //if this is the first segment average or the new average is <= the old one
        if(lowLabelStatus.lastMotorSteps == 0 || lowLabelStatus.averagedSensorFeedback <= lowLabelStatus.lastMotorSteps )
        {
            //set last segment average
            lowLabelStatus.lastMotorSteps = lowLabelStatus.averagedSensorFeedback;
        }
        
        //if this is the first data set collected
        if(lowLabelStatus.samplesCollected == 0)
        { 
            if(getTakingUpPaper() == true)
            {
                if(getLowLabelPeelingMaxFromHost() == 0)
                {
                    //PRINTF("lowLabelStatus.segmentLengthMax = LOW_LABEL_MAX_PEELING_DEFAULT\r\n");
                  
                    lowLabelStatus.segmentLengthMaxPeeling = LOW_LABEL_MAX_PEELING_DEFAULT;
                }
                else
                {
                    //PRINTF("lowLabelStatus.segmentLengthMax = getLowLabelPeelingMaxFromHost()\r\n");
                  
                    lowLabelStatus.segmentLengthMaxPeeling = getLowLabelPeelingMaxFromHost();
                } 
                
                if(getLowLabelPeelingMinFromHost() == 0)
                {
                    //PRINTF("lowLabelStatus.segmentLengthMin = LOW_LABEL_MIN_PEELING_DEFAULT\r\n");
                  
                    lowLabelStatus.segmentLengthMinPeeling = LOW_LABEL_MIN_PEELING_DEFAULT;
                }
                else
                {
                    //PRINTF("lowLabelStatus.segmentLengthMin = getLowLabelPeelingMinFromHost()\r\n");
                  
                    lowLabelStatus.segmentLengthMinPeeling = getLowLabelPeelingMinFromHost();
                } 
            }
            else
            {
                if(getLowLabelStreamingMaxFromHost() == 0)
                {
                    //PRINTF("lowLabelStatus.segmentLengthMax = LOW_LABEL_MAX_STREAMING_DEFAULT\r\n");
                  
                    lowLabelStatus.segmentLengthMaxStreaming = LOW_LABEL_MAX_STREAMING_DEFAULT;
                }
                else
                {
                    //PRINTF("lowLabelStatus.segmentLengthMax = getLowLabelStreamingMaxFromHost()\r\n");
                  
                    lowLabelStatus.segmentLengthMaxStreaming = getLowLabelStreamingMaxFromHost();
                } 
                
                if(getLowLabelStreamingMinFromHost() == 0)
                {
                    //PRINTF("lowLabelStatus.segmentLengthMin = LOW_LABEL_MIN_STREAMING_DEFAULT\r\n");
                    
                    lowLabelStatus.segmentLengthMinStreaming = LOW_LABEL_MIN_STREAMING_DEFAULT;
                }
                else
                {
                    //PRINTF("lowLabelStatus.segmentLengthMin = getLowLabelStreamingMinFromHost()\r\n");
                    
                    lowLabelStatus.segmentLengthMinStreaming = getLowLabelStreamingMinFromHost();
                } 
            }
          
            //if our current segment average is larger than our historic max 
            if((lowLabelStatus.segmentLengthMaxPeeling) < lowLabelStatus.averagedSensorFeedback && lowLabelStatus.averagedSensorFeedback < LOW_LABEL_MAX_PEELING && getTakingUpPaper() == true)
            {
                //set a new segment average max (new roll)
                lowLabelStatus.segmentLengthMaxPeeling = lowLabelStatus.averagedSensorFeedback;

                sendLowLabelMinMax(lowLabelStatus.segmentLengthMinPeeling, lowLabelStatus.segmentLengthMaxPeeling, lowLabelStatus.segmentLengthMinStreaming, lowLabelStatus.segmentLengthMaxStreaming);
                setLowLabelPeelingMaxFromHost(lowLabelStatus.segmentLengthMaxPeeling);    
            }
            else if((lowLabelStatus.segmentLengthMaxStreaming) < lowLabelStatus.averagedSensorFeedback && lowLabelStatus.averagedSensorFeedback < LOW_LABEL_MAX_STREAMING && getTakingUpPaper() == false)
            {
                //set a new segment average max (new roll)
                lowLabelStatus.segmentLengthMaxStreaming = lowLabelStatus.averagedSensorFeedback;
                
                sendLowLabelMinMax(lowLabelStatus.segmentLengthMinPeeling, lowLabelStatus.segmentLengthMaxPeeling, lowLabelStatus.segmentLengthMinStreaming, lowLabelStatus.segmentLengthMaxStreaming);
                setLowLabelStreamingMaxFromHost(lowLabelStatus.segmentLengthMaxStreaming);
            }
              
            //if we are peeling labels
            if(getTakingUpPaper() == true)
            {
                //if we set our empty roll value higher than a sane default, reset
                if(lowLabelStatus.segmentLengthMinPeeling > LOW_LABEL_MIN_PEELING_DEFAULT_MAX || lowLabelStatus.segmentLengthMinPeeling < LOW_LABEL_MIN_PEELING_DEFAULT_MIN)
                {
                    lowLabelStatus.segmentLengthMinPeeling = LOW_LABEL_MIN_PEELING_DEFAULT;
                }
            }
            else //if we are streaming labels
            {
                //if we set our empty roll value higher than a sane default, reset
                if(lowLabelStatus.segmentLengthMinStreaming > LOW_LABEL_MIN_STREAMING_DEFAULT_MAX || lowLabelStatus.segmentLengthMinStreaming < LOW_LABEL_MIN_STREAMING_DEFAULT_MIN )
                {
                    lowLabelStatus.segmentLengthMinStreaming = LOW_LABEL_MIN_STREAMING_DEFAULT;
                }
            }
            
            //if we are peeling labels
            if(getTakingUpPaper() == true)
            {
                //if our low label max (full roll) is less than a sane default
                if(lowLabelStatus.segmentLengthMaxPeeling < (LOW_LABEL_MAX_PEELING_DEFAULT + 1))
                {
                    //set a sane default
                    lowLabelStatus.segmentLengthMaxPeeling = LOW_LABEL_MAX_PEELING_DEFAULT;
                }
                
                //if segment average is greater than or equal to a sane default
                if(lowLabelStatus.averagedSensorFeedback >= (LOW_LABEL_MAX_PEELING))
                {
                    //set our label count estimation to a full roll 
                    lowLabelStatus.estimatedLabelCount = lowLabelStatus.numberOfLabelsOnFullRoll;
                    
                    lowLabelStatus.segmentLengthMaxPeeling = LOW_LABEL_MAX_PEELING;
                }
                else
                {
                    //set our label count estimation to a percentage that matches our sensor feedback estimated roll completion percentage
                    lowLabelStatus.estimatedLabelCount = lowLabelStatus.numberOfLabelsOnFullRoll * (((((float)(lowLabelStatus.averagedSensorFeedback - lowLabelStatus.segmentLengthMinPeeling) / (lowLabelStatus.segmentLengthMaxPeeling - lowLabelStatus.segmentLengthMinPeeling)) * 100.0f)) / 100);
                }
            }
            else //if we are streaming labels
            {
                //if our low label max (full roll) is less than a sane default
                if(lowLabelStatus.segmentLengthMaxStreaming < (LOW_LABEL_MAX_STREAMING_DEFAULT + 1))
                {
                    //set a sane default
                    lowLabelStatus.segmentLengthMaxStreaming = LOW_LABEL_MAX_STREAMING_DEFAULT;
                }
                
                //if segment average is greater than or equal to a sane default
                if(lowLabelStatus.averagedSensorFeedback >= (LOW_LABEL_MAX_STREAMING))
                {                   
                    //set our label count estimation to a full roll
                    lowLabelStatus.estimatedLabelCount = lowLabelStatus.numberOfLabelsOnFullRoll;
                    
                    lowLabelStatus.segmentLengthMaxStreaming = LOW_LABEL_MAX_STREAMING;
                }
                else
                {
                    //set our label count estimation to a percentage that matches our sensor feedback estimated roll completion percentage
                    lowLabelStatus.estimatedLabelCount = lowLabelStatus.numberOfLabelsOnFullRoll * (((((float)(lowLabelStatus.averagedSensorFeedback - lowLabelStatus.segmentLengthMinStreaming) / (lowLabelStatus.segmentLengthMaxStreaming - lowLabelStatus.segmentLengthMinStreaming)) * 100.0f)) / 100);
                }
            }  
        }
        
        //increment the number of data sets we have collected
        lowLabelStatus.samplesCollected++;
        
        //if this sample is not the first one and is less than the previous one
        if( lowLabelStatus.averagedSensorFeedback <= lowLabelStatus.lastMotorSteps && lowLabelStatus.lastMotorSteps > 0)
        {   
            // Calculate the percentage
            if(getTakingUpPaper() == true)
            {
                if(lowLabelStatus.averagedSensorFeedback > lowLabelStatus.segmentLengthMinPeeling)
                {
                    lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = ((float)(lowLabelStatus.averagedSensorFeedback - lowLabelStatus.segmentLengthMinPeeling) / (lowLabelStatus.segmentLengthMaxPeeling - lowLabelStatus.segmentLengthMinPeeling)) * 100.0f;
                }
                else
                {
                    lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = 1;
                }
            }
            else
            {
                if(lowLabelStatus.averagedSensorFeedback > lowLabelStatus.segmentLengthMinStreaming)
                {
                    lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = ((float)(lowLabelStatus.averagedSensorFeedback - lowLabelStatus.segmentLengthMinStreaming) / (lowLabelStatus.segmentLengthMaxStreaming - lowLabelStatus.segmentLengthMinStreaming)) * 100.0f;
                }
                else
                {
                    lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = 1;
                }
            }
        }
        
        //reset sample point related variables
        lowLabelStatus.samplePointIndex = 0;
        lowLabelStatus.motorSteps = 0;
        lowLabelStatus.consecutiveSamplesAboveThreshold = 0;
        lowLabelStatus.consecutiveSamplesBelowThreshold = 0;

        for(char i = 0; i < (LABEL_LOW_SAMPLE_COUNT - 1); i++)
        {
            lowLabelStatus.samplePoint[i] = 0;
        }
    }
}

void setLabelLowSteps( short steps )
{
    lowLabelStatus.motorSteps = steps;
}

short getLabelLowSteps( void )
{
    return lowLabelStatus.motorSteps;
}

void resetLabelLowVars( void )
{    
    //PRINTF("resetLabelLowVars()\r\n");
    lowLabelStatus.sensorFeedbackMax = 0;
    lowLabelStatus.sensorFeedbackMin = 4999;
      
    lowLabelStatus.samplePointIndex = 0;
    lowLabelStatus.lastMotorSteps = 0;
    lowLabelStatus.motorSteps = 0;
    lowLabelStatus.consecutiveSamplesAboveThreshold = 0;
    lowLabelStatus.consecutiveSamplesBelowThreshold = 0;
    
    lowLabelStatus.samplesCollected = 0;
    
    lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = 0;
    lowLabelStatus.averagedSensorFeedback = 0;
    
    currentStatus.labelLowPercentage = 0;
    
    lowLabelStatus.estimatedLabelCount = 810;
    
    lowLabelStatus.numberOfLabelsOnFullRoll = 810;
    lowLabelStatus.ADCSample = 0;
    lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining = 0;

    for(char i = 0; i < (LABEL_LOW_SAMPLE_COUNT - 1); i++)
    {
        lowLabelStatus.samplePoint[i] = 0;
    }
}

void resetLabelLowSamples( void )
{    
    //PRINTF("resetLabelLowSamples()\r\n");
    //lowLabelStatus.sensorFeedbackMax = 0;
    //lowLabelStatus.sensorFeedbackMin = 4999;
      
    lowLabelStatus.samplePointIndex = 0;
    lowLabelStatus.lastMotorSteps = 0;
    //lowLabelStatus.motorSteps = 0;
    lowLabelStatus.consecutiveSamplesAboveThreshold = 0;
    lowLabelStatus.consecutiveSamplesBelowThreshold = 0;
    
    //lowLabelStatus.samplesCollected = 0;
    
    //lowLabelStatus.sensorEstimatedPercentageOfRollRemaining = 0;
    lowLabelStatus.averagedSensorFeedback = 0;
    
    //currentStatus.labelLowPercentage = 0;
    
    lowLabelStatus.estimatedLabelCount = 810;
    
    lowLabelStatus.numberOfLabelsOnFullRoll = 810;
    lowLabelStatus.ADCSample = 0;
    lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining = 0;

    for(char i = 0; i < (LABEL_LOW_SAMPLE_COUNT - 1); i++)
    {
        lowLabelStatus.samplePoint[i] = 0;
    }
}

void updateNumberOfLabelsOnRoll(void)
{
    uint16_t steps = engine.numPrintLines;
    bool hasLargeGap = getLargeGapFlag();

    const size_t tableSize = sizeof(labelTable) / sizeof(labelTable[0]);

    // Default to first entry
    size_t index = 0;

    // Find the largest entry where stepLimit <= steps
    for (size_t i = 0; i < tableSize; i++)
    {
        if (steps >= labelTable[i].stepLimit) 
        {
            index = i;
        }        
        else
        {
            break;
        }
    }

    // Clamp to last entry if numPrintLines exceeds all limits
    if (steps > labelTable[tableSize - 1].stepLimit)
    {
        index = tableSize - 1;
    }
        
    lowLabelStatus.numberOfLabelsOnFullRoll = hasLargeGap ?
        labelTable[index].gapCount : labelTable[index].noGapCount;
}

void updateRollCompletionPercentage( void )
{
    //if our estimated label count is greater than one, decrement
    if(lowLabelStatus.estimatedLabelCount > 1)
    {
        lowLabelStatus.estimatedLabelCount--;
    }
  
    //calculate estimated label count roll completion percentage
    lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining = ((float)(lowLabelStatus.estimatedLabelCount - 1) / (lowLabelStatus.numberOfLabelsOnFullRoll - 1)) * 100.0f;
  
    //if we have collected enough low label samples to have a segment average
    if(lowLabelStatus.averagedSensorFeedback > 0)
    {
        char lowLabelDelta = 0;
      
        //if our estimated label count percentage is greater than or equal to our segment average
        if((char)lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining >= (char)lowLabelStatus.sensorEstimatedPercentageOfRollRemaining)
        {
            //calculate the delta between our two percentages
            lowLabelDelta = (char)lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining - (char)lowLabelStatus.sensorEstimatedPercentageOfRollRemaining;
            
            //if the delta is larger than 5 and we think we have more than 11 labels left
            if(lowLabelDelta >= 3 && lowLabelStatus.estimatedLabelCount > 11)
            {
                //set our label count estimation 10 values lower
                lowLabelStatus.estimatedLabelCount -= 10;
            }
        }
        else //our estimated label count percentage is less than our segment average
        {
            lowLabelDelta = (char)lowLabelStatus.sensorEstimatedPercentageOfRollRemaining - (char)lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining;   
            
            //if the delta is larger than 5 
            if(lowLabelDelta >= 3)
            {
                //set our label count estimation 10 values higher
                lowLabelStatus.estimatedLabelCount += 10;
            }
        }
        
        
        //average our estimated label count percentage with our sensor derived roll completion percentage
        char reportedPercentageOfRollRemaining = /*(char)(((char)lowLabelStatus.estimatedLabelCountPercentageOfRollRemaining + */(char)lowLabelStatus.sensorEstimatedPercentageOfRollRemaining/*) / 2 )*/;
        
        //clamp to sane reporting values
        if(reportedPercentageOfRollRemaining > 100)
        {
            reportedPercentageOfRollRemaining = 100;
        }
        else if(reportedPercentageOfRollRemaining < 1)
        {
            reportedPercentageOfRollRemaining = 1;
        }
        
        //PRINTF("low label sensor max = %d\r\n", lowLabelStatus.sensorFeedbackMax);
        //PRINTF("low label sensor min = %d\r\n", lowLabelStatus.sensorFeedbackMin);
        //PRINTF("low label sensor feedback delta = %d\r\n", (lowLabelStatus.sensorFeedbackMax - lowLabelStatus.sensorFeedbackMin));
        //PRINTF("low label sensor threshold = %d\r\n", getLabelLowThreshold());
        //PRINTF("low label segment max peeling = %d\r\n", lowLabelStatus.segmentLengthMaxPeeling);
        //PRINTF("low label segment min peeling = %d\r\n", lowLabelStatus.segmentLengthMinPeeling);
        //PRINTF("low label segment max streaming = %d\r\n", lowLabelStatus.segmentLengthMaxStreaming);
        //PRINTF("low label segment min streaming = %d\r\n", lowLabelStatus.segmentLengthMinStreaming);
        //PRINTF("low label averaged sensor feedback = %d\r\n", lowLabelStatus.averagedSensorFeedback);
        //PRINTF("low label sensor derived estimated percentage = %d\r\n", (char)lowLabelStatus.sensorEstimatedPercentageOfRollRemaining);
        //PRINTF("low label sensor reported percentage = %d\r\n", (char)reportedPercentageOfRollRemaining);
        //PRINTF("low label reported nearest 25 combined percentage = %d\r\n", roundToNearest25((char)reportedPercentageOfRollRemaining));
    
        if(currentStatus.labelLowPercentage > (char)reportedPercentageOfRollRemaining || currentStatus.labelLowPercentage == 0)
        {
            if((char)reportedPercentageOfRollRemaining >= 25)
            {
                currentStatus.labelLowPercentage = roundToNearest25((char)reportedPercentageOfRollRemaining);
            }
            else
            {
                currentStatus.labelLowPercentage = roundToNearest5((char)reportedPercentageOfRollRemaining);
            } 
        }
    }
}

char roundToNearest5(char value)
{
    if (value < 1) value = 1;
    if (value > 100) value = 100;

    int remainder = value % 5;
    int rounded = value;

    if (remainder == 0)
    {
        rounded = value; // already a multiple of 5
    }
    else if (remainder <= 3)
    {
        rounded = value - remainder; // round down
    }
    else // remainder == 4
    {
        rounded = value + (5 - remainder); // round up
    }

    if (rounded < 1) rounded = 1;
    if (rounded > 100) rounded = 100;

    return (char)rounded;
}

char roundToNearest25(char value)
{
    if (value < 1) value = 1;
    if (value > 100) value = 100;

    int remainder = value % 25;
    int rounded = value;

    if (remainder == 0)
    {
        rounded = value; // already a multiple of 25
    }
    else if (remainder <= 5)
    {
        rounded = value - remainder; // round down to nearest 25
    }
    else
    {
        rounded = value + (25 - remainder); // round up to nearest 25
    }

    if (rounded < 1) rounded = 1;
    if (rounded > 100) rounded = 100;

    return (char)rounded;
}

LowLabelStatus* getLowLabelStatus( void )
{
    return &lowLabelStatus;
}

short getLowLabelPeelingMaxFromHost( void )
{
    return lowLabelPeelingMaxFromHost;
}

short getLowLabelStreamingMaxFromHost( void )
{
    return lowLabelStreamingMaxFromHost;
}

short getLowLabelPeelingMinFromHost( void )
{
    return lowLabelPeelingMinFromHost;
}

short getLowLabelStreamingMinFromHost( void )
{
    return lowLabelStreamingMinFromHost;
}

void setLowLabelPeelingMaxFromHost(short value)
{
    lowLabelPeelingMaxFromHost = value;
}

void setLowLabelStreamingMaxFromHost(short value)
{
    lowLabelStreamingMaxFromHost = value;
}

void setLowLabelPeelingMinFromHost(short value)
{
    lowLabelPeelingMinFromHost = value;
}

void setLowLabelStreamingMinFromHost(short value)
{
    lowLabelStreamingMinFromHost = value;
}

uint16_t calculateSizingOffset(uint16_t labelSteps)
{
    uint16_t steps = labelSteps;
    bool hasLargeGap = getLargeGapFlag();

    const size_t tableSize = sizeof(labelOffsetTable) / sizeof(labelOffsetTable[0]);

    // Default to first entry
    size_t index = 0;

    // Find the largest entry where stepLimit < steps
    for (size_t i = 0; i < tableSize; i++)
    {
        if (steps >= labelOffsetTable[i].stepLimit) 
        {
            index = i;
        }        
        else
        {
            break;
        }
    }

    // Clamp to last entry if steps exceeds all limits
    if (steps > labelOffsetTable[tableSize - 1].stepLimit)
    {
        index = tableSize - 1;
    }
    
    uint16_t stepOffset = 0;
        
    stepOffset = hasLargeGap ?
        labelOffsetTable[index].HTOffset : labelOffsetTable[index].GTOffset;
    
    PRINTF("\r\nSTEPS BEFORE OFFSETTING: %d", steps);  
    PRINTF("\r\nOFFSET INDEX: %d", index); 
    PRINTF("\r\nSIZING OFFSET APPLIED: %d\r\n\r\n", stepOffset);
        
    return stepOffset;
}




