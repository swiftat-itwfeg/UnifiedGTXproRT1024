#include "translator.h"
#include "averyProtocol.h"
#include "hobartProtocol.h"
#include "unifiedProtocol.h"
#include "averyWeigherCfg.h"
#include "queueManager.h"
#include "dotWearTask.h"
#include "printEngine.h"
#include "sensors.h"
#include "semphr.h"
#include "fsl_debug_console.h"
#include <stdio.h>

#ifdef LEGACY_PROTOCOL
/*  header + body */
AT_NONCACHEABLE_SECTION_ALIGN_INIT( unsigned char prRcvUsbBfr[ 74 ], 4U ) = { 0 };
AT_NONCACHEABLE_SECTION_ALIGN_INIT( unsigned char wrRcvUsbBfr[ 74 ], 4U ) = { 0 };
#else
/* header and body combined */ 
AT_NONCACHEABLE_SECTION_ALIGN_INIT( unsigned char prRcvUsbBfr[ 64 ], 4U ) = { 0 };
AT_NONCACHEABLE_SECTION_ALIGN_INIT( unsigned char wrRcvUsbBfr[ 64 ], 4U ) = { 0 };
#endif

extern uint16_t LABEL_TAKEN_THRESHOLD_LABEL;
extern uint16_t LABEL_TAKEN_THRESHOLD_NO_LABEL;

extern bool readLabelTakenSensor( void );
extern PrStatusInfo     currentStatus;
extern void postRequestStatus( void );
extern bool isLabelTakenTimerStarted( void );
extern void setLabelTakenTimer( void );

static bool             suspend_                = false;
static TaskHandle_t     tHandle_                = NULL;
static QueueHandle_t    pQUSBSendPrHandle_      = NULL;         /* USB tx messages printer */
static QueueHandle_t    pQUSBSendWrHandle_      = NULL;         /* USB tx messages weigher */
static QueueHandle_t    pQUSBRcvPrHandle_       = NULL;         /* USB rx messages printer */
static QueueHandle_t    pQUSBRcvWrHandle_       = NULL;         /* USB rx messages weigher */

static QueueSetHandle_t tQueueSet_              = NULL;         /* in and out set */
static QueueHandle_t    pWQHandle_              = NULL;         /* messages to the weigher task */ 
static QueueHandle_t    pPQHandle_              = NULL;         /* messages to the printer task */

static SemaphoreHandle_t tMutex_;
static bool  debugMsg_                          = false;

static TransMSG msgTracker;                                     /* for messages > 64 bytes */
static PrStatTracker    tckr[ _PRMAXFIELDS ];                   /* spamming protection for status message */
static DEVICESUBCLASS_t sClass_                 = UNKNOWN_SUBCLASS;
/* avery weigher encryption key */
const unsigned char avKey[] = { 0xcc, 0xb6, 0xa7, 0xf7, 0xcb, 0xc8, 0x69, 0xcd, 0xce, 0xcf, 0x00 };
   
extern bool getGapCalStatus( void );

/******************************************************************************/
/*!   \fn BaseType_t createTranslator( QueueHandle_t wgMsgQueue, 
                                       QueueHandle_t prMsgQueue, 
                                       QueueHandle_t UsbInPrMsgQueue,
                                       QueueHandle_t UsbInWrMsgQueue,
                                       QueueHandle_t UsbOutPrMsgQueue, 
                                       QueueHandle_t UsbOutWrMsgQueue )

      \brief
        This function intializes the USB message translator.
   
      \author
          Aaron Swift
*******************************************************************************/
BaseType_t createTranslator( DEVICESUBCLASS_t sClass, QueueHandle_t wgMsgQueue, QueueHandle_t prMsgQueue, 
                             QueueHandle_t UsbInPrMsgQueue,
                             QueueHandle_t UsbInWrMsgQueue,
                             QueueHandle_t UsbOutPrMsgQueue, 
                             QueueHandle_t UsbOutWrMsgQueue)
{
    BaseType_t result;

    sClass_ = sClass;
    if( ( wgMsgQueue != NULL ) && ( prMsgQueue != NULL ) && 
        ( UsbInPrMsgQueue != NULL ) && ( UsbInWrMsgQueue != NULL ) &&
        ( UsbOutPrMsgQueue != NULL ) && ( UsbOutWrMsgQueue != NULL ) ) {
        pWQHandle_              = wgMsgQueue;
        pPQHandle_              = prMsgQueue;
        pQUSBSendPrHandle_      = UsbInPrMsgQueue;
        pQUSBSendWrHandle_      = UsbInWrMsgQueue;
        pQUSBRcvPrHandle_       = UsbOutPrMsgQueue;
        pQUSBRcvWrHandle_       = UsbOutWrMsgQueue;
        
        PRINTF("createTranslator(): Starting...\r\n" );
        /* determine length of transmit and receive queues combined */
        unsigned long wQSetLength = ( getUsbInQueueLength() * 2 );
        
        /* create local queue set */
        tQueueSet_ = (QueueHandle_t)xQueueCreateSet( (UBaseType_t)wQSetLength );
        /* add queues to local set */
        xQueueAddToSet( pQUSBRcvPrHandle_, tQueueSet_ );
        xQueueAddToSet( pQUSBRcvWrHandle_, tQueueSet_ );
                
        /* create mutex for translateFrameHdr()  */
        tMutex_ = xSemaphoreCreateMutex(); 
        if( !tMutex_ ) {     
            PRINTF("createTranslator(): Failed to create mutex!\r\n" );
        }
        
        if( sClass_ == HB_GT ) {        
            
            /* clear status tracker */
            memset( &tckr[0], 0, _PRMAXFIELDS );
            
            tckr[ _PRERROR ].field = _PRERROR;
            tckr[ _PRSENSOR ].field = _PRSENSOR;
            tckr[ _PRSENSOR2 ].field = _PRSENSOR2;
            tckr[ _PRUSR ].field = _PRUSR;
            
            msgTracker.handlingMsg = false;
            msgTracker.msgSize = 0;
        } 
                
        PRINTF("translatorTask(): Starting...\r\n" );
        /*create our thread */
        result = xTaskCreate( translatorTask, "TranslatorTask", 
                              configMINIMAL_STACK_SIZE , NULL, translator_task_PRIORITY, 
                             &tHandle_ );
    } else {
        PRINTF("translateMessage(): Queues are null!\r\n" );
    }
    return result;        
}

/******************************************************************************/
/*!   \fn TaskHandle_t getTranslatorHandle( void )

      \brief
        This function returns the translator task handle. 
   
      \author
          Aaron Swift
*******************************************************************************/
TaskHandle_t getTranslatorHandle( void )
{
    return tHandle_;
}

/******************************************************************************/
/*!   \fn static void translatorTask( void *pvParameters )

      \brief
        This function converts usb frames into hobart messages for manager 
        processing.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translatorTask( void *pvParameters )
{
    ( void ) pvParameters;
    
    unsigned int msgCnt                 = 0;
    QueueSetMemberHandle_t setHandle    = NULL;
    
    PRINTF("translatorTask(): Thread running...\r\n" ); 
    
    while( !suspend_ ) {           
        /* wait for object in one of the queues within the set*/  
        setHandle = xQueueSelectFromSet( tQueueSet_, portMAX_DELAY );
        /* determine which queue */
        if( setHandle ==  pQUSBRcvPrHandle_ ) {
            /* how many transactions are queued? */
            msgCnt = uxQueueMessagesWaiting( setHandle );
            while( msgCnt ) {
              /* process printer message from host */
              if( xQueueReceive( setHandle, &prRcvUsbBfr[0], 0 ) ) {     
                  if( prRcvUsbBfr[0] == HOBART_PROTOCOL ) {
                      translateHBPrinterMessage( &prRcvUsbBfr[0] );   
                  } else if( prRcvUsbBfr[0] == AVERY_PROTOCOL ) {
                      translateAVPrinterMessage( &prRcvUsbBfr[0] );   
                  } else if( prRcvUsbBfr[0] == UNIFIED_PROIOCOL ) {
                      translateUNPrinterMessage( &prRcvUsbBfr[0] );   
                  } else {
                      PRINTF("translatorTask(): Unknown Printer protocol OUT message!\r\n" );              
                  }
                  msgCnt--;
              } else {
                  PRINTF("translatorTask(): Failed Printer OUT message from USB queue!\r\n" );              
              }
            }
        } else if( setHandle ==  pQUSBRcvWrHandle_ ) {
            /* how many transactions are queued? */
            msgCnt = uxQueueMessagesWaiting( setHandle );
            while( msgCnt ) {
                /* process weigher message from host */
                if( xQueueReceive( setHandle, &wrRcvUsbBfr[0], 0 ) ) {
                    if( wrRcvUsbBfr[0] == HOBART_PROTOCOL ) {
                        translateHBWeigherMessage( &wrRcvUsbBfr[0] ); 
                    } else if( wrRcvUsbBfr[0] == AVERY_PROTOCOL ) {
                        translateAVWeigherMessage( &wrRcvUsbBfr[0] ); 
                    } else if( wrRcvUsbBfr[0] == UNIFIED_PROIOCOL ) {
                        translateUNWeigherMessage( &wrRcvUsbBfr[0] );
                    } else {
                        PRINTF("translatorTask(): Unknown Weigher protocol OUT message!\r\n" );        
                    }
                    msgCnt--;
                } else {
                    PRINTF("weigherTask(): Failed to OUT message from USB queue!\r\n" );              
                }          
            }   
        } else if( setHandle ==  pQUSBSendPrHandle_ ) {
            PRINTF("translatorTask(): Unknown queue set handle!\r\n" );
        } else if( setHandle ==  pQUSBSendWrHandle_ ) {
            PRINTF("translatorTask(): Unknown queue set handle!\r\n" );
        }
        taskYIELD();
    }
    vTaskSuspend(NULL);    
}


/******************************************************************************/
/*!   \fn static bool isQueueAlmostFull( QueueHandle_t pQueue, 
                                         unsigned short queueLength, unsigned short margin, 
                                         bool interrupt )

      \brief
        Checks to see if the queue is almost full. The purpose of this function
        is to ensure we don't post a header without having room for the body also 
        because this leads to "out of sync" error in usbTask().
   
      \author
          Tom Fink
*******************************************************************************/
static bool isQueueAlmostFullISR( QueueHandle_t pQueue, 
                                  unsigned short queueLength, unsigned short margin, 
                                  bool interrupt )
 {      
       unsigned short itemsOnQueue = queueLength;
       if( interrupt ) {
          itemsOnQueue = uxQueueMessagesWaitingFromISR(pQUSBSendPrHandle_);
       } else {
          itemsOnQueue = uxQueueMessagesWaiting(pQUSBSendPrHandle_);
       }

       if( itemsOnQueue > queueLength-margin ) {
          return(true);          
       }
       else
          return(false);  
 }

/******************************************************************************/
/*!   \fn static bool isQueueAlmostFull( QueueHandle_t pQueue, 
                                         unsigned short queueLength, unsigned short margin )

      \brief
        Checks to see if the queue is almost full. The purpose of this function
        is to ensure we don't post a header without having room for the body also 
        because this leads to "out of sync" error in usbTask().
   
      \author
          Tom Fink
*******************************************************************************/ 
static bool isQueueAlmostFull( QueueHandle_t pQueue, unsigned short queueLength, unsigned short margin )
 {     
       unsigned short itemsOnQueue = queueLength;

       itemsOnQueue = uxQueueMessagesWaiting( pQUSBSendPrHandle_ );

       if( itemsOnQueue > queueLength-margin ) {
          return(true);          
       }
       else
          return(false);  
 }

/******************************************************************************/
/*!   \fn static void translateHBPrinterMessage( unsigned char *pBfr  )

      \brief
        private function translates hobart printer usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateHBPrinterMessage( unsigned char *pBfr )
{
    #ifdef LEGACY_PROTOCOL
    static unsigned int msgSize = 0;
    static USBHeader prHdr;
         
    if( pBfr != NULL ) {      
        if( msgSize == 0 ) {
            /* parse our header information first */              
            if( xSemaphoreTake( tMutex_, ( TickType_t )100 ) == pdTRUE ) {
                msgSize = translateFrameHdr( pBfr, &prHdr );               
                xSemaphoreGive( tMutex_ );
            } else {
                PRINTF("translateFrameHdr(): Failed to take semaphore!\r\n" );
            }
            if( msgSize == 0 ) {
                //PRINTF("translateFrameHdr(): msgSize: %d\r\n", msgSize );
                debugMsg_ = true;
            } else {
                debugMsg_ = false;
            }
        } else { 
            if( ( prHdr.DestLogicalAddr == LGCY_LOGICAL_ADDR_PRNTR ) ||
              ( prHdr.DestLogicalAddr == LGCY_LOGICAL_ADDR_GPRNTR ) ) { 
                  t_handlePrinterMessage( pBfr, msgSize );
                  if( msgSize <= 64 ) {
                      msgSize = 0;
                  } else {
                      msgSize -= 64;
                  }
            } else {
              PRINTF("translateMessage(): Unhandled device type %d\r\n", prHdr.DestLogicalAddr );
            }
        }
    } else {
        PRINTF("translateMessage(): pBfr is null!\r\n" );
    }
    
    #else
    HUSBMessage msg;

    if( pBfr != NULL ) {
        /* parse our header information first */
        translateFrameHdr( pBfr, &msg.hdr );
        if( isPrinterMessage( msg.hdr.device ) ) {            
            msg.pMsg = ( pBfr + sizeof( USBHeader ) );
            t_handlePrinterMessage( &msg );
        } else if( isWeigherMessage( msg.hdr.device ) ) {
            msg.pMsg = ( pBfr + sizeof( USBHeader ) );
            t_handleWeigherMessage( &msg );
        } else {
            PRINTF("translateMessage(): Unhandled device type %d\r\n", msg.hdr.device );
        }
    } else {
        PRINTF("translateMessage(): pBfr is null!\r\n" );
    }
    #endif    
}

/******************************************************************************/
/*!   \fn static void translateAVPrinterMessage( unsigned char *pBfr  )

      \brief
        private function translates avery printer usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateAVPrinterMessage( unsigned char *pBfr )
{          
    AVMSGPAYLOAD_t *pMsg = (AVMSGPAYLOAD_t *)pBfr;
    AVIMSG_t imsg;
      
    /* TO DO: validate message */
    if( isValidAVPacket( pBfr ) ) {
        /* parse the message command and length */
        imsg.cmd = (AVMSG_t)pMsg->header.cmd;
        imsg.msgLength = pMsg->header.len - sizeof( AVMSGHEADER_t );
        if( imsg.msgLength > 0 ) {
            memcpy( &imsg.msgData[0], &pMsg->msgData[0], imsg.msgLength );   
        } else {
            memset(  &imsg.msgData[0], 0, PACKET_DATA_SIZE );
        }
        
        if( imsg.cmd == 39 ) {
            PRINTF("translateAVPrinterMessage(): blck: %d\r\n", pMsg->header.blkNum );
        }
        
        if( pPQHandle_ ) {      
            /* add message to the printer queue */
            BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&imsg, 0 );
            if( result != pdTRUE ) {
                PRINTF("translateAVPrinterMessage(): PR Message Queue full! \r\n");  
            }             
        } else {
            PRINTF("translateAVPrinterMessage(): PR Message Queue handle null! \r\n");  
        }      
    } else {
        PRINTF("translateAVPrinterMessage(): Message paylaod is inValid! \r\n");  
    }
}

/******************************************************************************/
/*!   \fn static bool isValidAVPacket( unsigned char *pBuffer )

      \brief
        private function checks if incomming packet is valid.
   
      \author
          Aaron Swift
*******************************************************************************/
static bool isValidAVPacket( unsigned char *pBuffer )
{
      bool result = false;
      AVMSGHEADER_t *pHeader = (AVMSGHEADER_t *)pBuffer;
      
      if( pHeader->stx == AVERY_PROTOCOL ) {
          if( ( pHeader->cmd >= AV_FIRSTVALIDCMD ) && 
              ( pHeader->cmd < AV_INVALID_COMMAND ) ) {
              pHeader->blkNum = ntohs( pHeader->blkNum );
              pHeader->numBlks = ntohs( pHeader->numBlks );     
              if( pHeader->numBlks == 1 ) {
#if 0             /* debug */
                  PRINTF("isValidAVPacket(): Message is valid: \r\n" ); 
                  PRINTF("isValidAVPacket(): Message command: %d\r\n", pHeader->cmd ); 
                  PRINTF("isValidAVPacket(): Message length: %d\r\n", pHeader->len );
#endif
                  result = true;
              } else {
                  PRINTF("isValidAVPacket(): Message greater than 512 bytes are not supported yet!!!\r\n" );
              }
          } else {
              PRINTF("isValidAVPacket(): Message has invalid command! \r\n" );
          }
      } else {
              PRINTF("isValidAVPacket(): Message has invalid protocol! \r\n" );      
      }
      return result;
}

/******************************************************************************/
/*!   \fn static void translateHBWeigherMessage( unsigned char *pBfr  )

      \brief
        private function translates hobart weigher usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateHBWeigherMessage( unsigned char *pBfr )
{
    #ifdef LEGACY_PROTOCOL
    static unsigned int msgSize = 0;
    static USBHeader wrHdr;
     
    if( pBfr != NULL ) {      
        if( msgSize == 0 ) {
            if( xSemaphoreTake( tMutex_, ( TickType_t )100 ) == pdTRUE ) {
                /* parse our header information first */
                msgSize = translateFrameHdr( pBfr, &wrHdr );
                msgTracker.handlingMsg = false;
                msgTracker.msgSize = msgSize;
                xSemaphoreGive( tMutex_ );
            } else {
                PRINTF("translateWeigherMessage(): failed to take semaphore!\r\n");
            }
        } else {        
            if( ( wrHdr.DestLogicalAddr == LGCY_LOGICAL_ADDR_WGR ) ||
              ( wrHdr.DestLogicalAddr == LGCY_LOGICAL_ADDR_GWGR ) ) {                        
                  t_handleWeigherMessage( pBfr, msgSize );
                  if( msgSize <= 64 ) {
                      msgTracker.handlingMsg = false;
                      msgSize = 0;
                      msgTracker.msgSize = 0;
                  } else {
                      msgTracker.handlingMsg = true;
                      msgSize -= 64;
                      msgTracker.msgSize -= 64;
                  }
                  
            } else {
              PRINTF("translateMessage(): Unhandled device type %d\r\n", wrHdr.DestLogicalAddr );
            }
        }
    } else {
        PRINTF("translateMessage(): pBfr is null!\r\n" );
    }
    
    #else
    HUSBMessage msg;

    if( pBfr != NULL ) {
        /* parse our header information first */
        translateFrameHdr( pBfr, &msg.hdr );
        if( isPrinterMessage( msg.hdr.device ) ) {            
            msg.pMsg = ( pBfr + sizeof( USBHeader ) );
            t_handlePrinterMessage( &msg );
        } else if( isWeigherMessage( msg.hdr.device ) ) {
            msg.pMsg = ( pBfr + sizeof( USBHeader ) );
            t_handleWeigherMessage( &msg );
        } else {
            PRINTF("translateMessage(): Unhandled device type %d\r\n", msg.hdr.device );
        }
    } else {
        PRINTF("translateMessage(): pBfr is null!\r\n" );
    }
    #endif    
}

/******************************************************************************/
/*!   \fn static void translateAVWeigherMessage( unsigned char *pBfr  )

      \brief
        private function translates avery weigher usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateAVWeigherMessage( unsigned char *pBfr )
{
    /* BinaryHeader_t *pHdr; */
    AVIWGMSG_t  msg;
    /* unsigned char *pData = pBfr; */
    unsigned short mSize = 0;
    if( pBfr != NULL ) {
        /* skip passed the message protocol bytes */
        pBfr ++;
        msg.header.messageSize = fromAsciiHex16( (unsigned char *)pBfr, msg.header.messageSize );
        pBfr += 4;
        msg.header.cvn = fromAsciiHex16( (unsigned char *)pBfr, msg.header.cvn );
        pBfr += 4;
        msg.header.slaveID = fromAsciiHex8( (unsigned char *)pBfr, msg.header.slaveID );
        pBfr += 2; /* should be 13 and 14 */
        msg.header.cmd = fromAsciiHex16( (unsigned char *)pBfr, msg.header.cmd );
        pBfr += 4;
        
        msg.header.action = *pBfr++;          
        msg.header.type = *pBfr++;
        
        /* change to ascii */
        msg.header.action += 0x20;
        msg.header.type += 0x20;
        
        msg.header.status = fromAsciiHex16( (unsigned char *)pBfr, msg.header.status );
        pBfr += 4;
        msg.header.sequenceNum = fromAsciiHex16( (unsigned char *)pBfr, msg.header.sequenceNum );
        pBfr += 4;
        /* determine message body size which is the header size minus 8 bytes to exclude message size and crc */ 
        mSize = msg.header.messageSize - ( sizeof(AsciiHeader)  );
 
        if( mSize <= 64 ) {
            for( int i = 0; i < mSize; i++ ) {
                msg.msgData[i] = *pBfr++;             
            }        
        }
        
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("translateAVWeigherMessage(): WG Message Queue full! \r\n");  
        }                            
    } else {
        PRINTF("translateAVWeigherMessage(): Null message pointer!\r\n");
    }    
}

/******************************************************************************/
/*!   \fn static void translateUNPrinterMessage( unsigned char *pBfr  )

      \brief
        private function translates unified printer usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateUNPrinterMessage( unsigned char *pBfr )
{
  
}

/******************************************************************************/
/*!   \fn static void translateUNWeigherMessage( unsigned char *pBfr  )

      \brief
        private function translates unified weigher usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateUNWeigherMessage( unsigned char *pBfr )
{
  
}


/******************************************************************************/
/*!   \fn static unsigned int translateFrameHdr( unsigned char *pBfr, USBHeader *pHdr  )

      \brief
        private function parses hobart header information from the usb frame
        and returns the length of the message body.
   
      \author
          Aaron Swift
*******************************************************************************/
static unsigned int translateFrameHdr( unsigned char *pBfr, USBHeader *pHdr )
{
    unsigned int length = 0;
    
    #ifdef LEGACY_PROTOCOL 
    pHdr->sourcePhysAddr        = charToShort( pBfr );
    pBfr += 2;
    pHdr->sourceLogicalAddr     = charToShort( pBfr );
    pBfr += 2;
    pHdr->DestPhysAddr          = charToShort( pBfr );  /* unused */
    pBfr += 2;
    pHdr->DestLogicalAddr       = charToShort( pBfr );
    pBfr += 2;
    pHdr->msgSize               = charToShort( pBfr );        
    /* is the header valid? */
    if( ( pHdr->DestLogicalAddr == LGCY_LOGICAL_ADDR_PRNTR ) ||
        ( pHdr->DestLogicalAddr == LGCY_LOGICAL_ADDR_WGR ) ) {
      length = pHdr->msgSize;
      if( length == 0 ) {
          PRINTF("translateFrameHdr(): Error msgSize: 0\r\n");             
      }
    }
    #else
    pHdr->device         = (HDeviceType)*pBfr++;
    pHdr->stationId      = *pBfr++;
    pHdr->msgType        = *pBfr++;
    pHdr->msgSize        = charToLong( pBfr );
    pBfr += 4;
    pHdr->frameNumber    = *pBfr++;
    pHdr->numberOfFrames = *pBfr++;
    pHdr->lastFrameSize  = charToShort( pBfr ); 
    #endif       
    
    return length;
}

#pragma diag_suppress=Pe177
/******************************************************************************/
/*!   \fn static bool isWeigherMessage( HDeviceType type )

      \brief
        private function determine if message is of weigher type.
   
      \author
          Aaron Swift
*******************************************************************************/
static bool isWeigherMessage( HDeviceType type )
{
    bool result = false;
    if( type == G_WEIGHER_ ) {
        result = true;
    }
    return result;
}

/******************************************************************************/
/*!   \fn static bool isPrinterMessage( HDeviceType type )

      \brief
        private function determine if message is of printer type.
   
      \author
          Aaron Swift
*******************************************************************************/
static bool isPrinterMessage( HDeviceType type )
{   
    bool result = false;
    if( type == G_PRINTER_ ) {
        result = true;        
    }
    return result;
}
#pragma diag_default=Pe177

#ifdef LEGACY_PROTOCOL
/******************************************************************************/
/*!   \fn static void t_handlePrinterMessage( unsigned char *pMsg_ )

      \brief
        handles incomming usb hobart printer messages.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrinterMessage( unsigned char *pMsg_, unsigned int msgSize  )
{   
    if( pMsg_ != NULL ) {

        /* first byte is always the message type */
        switch( *pMsg_ )
        {
            case PR_REQ_WAKEUP:
            case PR_WAKEUP: {                
                t_handlePrReqWakeup();                
                break;  
            }
            case PR_CONFIG:
            case PR_GLOBAL_CONFIG: {
                t_handlePrWriteConfig( pMsg_);
                break;
            }
            case PR_REQ_CONFIG: {
                t_handlePrReqConfig();
                break;  
            }
            case PR_RESET: {
                t_handlePrReset();
                break;  
            }
            case PR_REQ_STATUS: {
                t_handlePrReqStatus();
                break;  
            }
            case PR_ENABLE: {
                t_handlePrEnable();
                break;  
            }
            case PR_DISABLE: {
                t_handlePrDisable();
                break;  
            }
            case PR_MODE: {
                t_handlePrMode( *( pMsg_ + 1 ) );
                break;  
            }
            case PR_REQ_SENSORS: {
                t_handlePrReqSensors();
                break;  
            }
            case PR_RAM: {
                t_handlePrRam( pMsg_ );
                break;
            }
            case PR_COMMAND: {                
                t_handlePrCommand( pMsg_ );
                break;  
            }
            case PR_REQ_CALIBRATE: {
                t_handlePrReqCalibration();
                break;  
            }
            case PR_SIZE: {
                t_handlePrNomSize(  pMsg_ );
                break;  
            }
            case PR_ENABLE_TAKEUP: {
                t_handlePrEnableTakeup();
                break;  
            }
            case PR_REQ_VERSION: {
                t_handlePrReqVersion();
                break;  
            }
            case PR_REQ_HEAD_TYPE: {
                t_handlePrReqHeadType();
                break;  
            }
            case PR_REQ_HEAD_POWER: {
                  break;
            }
            case PR_CUTTER_CUT: {
                t_handlePrCutterCut();
                break;  
            }
            case PR_REQ_CUTTER_STATUS: {
                t_handlePrReqCutterStatus();
                break;  
            }
            case PR_ERASE_CUTTER_BIT: {
                t_handlePrCutterDefault();
                break;
            }
            case PR_TEST: {
                t_handlePrTest( pMsg_ );
                break;
            }
            case PR_TEACH: {
                t_handlePrTeach( pMsg_ );
                break;
            }
            case PR_REQ_TRANSFER: {
                t_handlePrReqLabelTransfer( pMsg_ );
                break;
            }
            case PR_REQ_DOT_WEAR: {
                t_handlePrReqDotWear();
                break;  
            }
            case PR_REQ_DOT_STATUS: {
                t_handlePrReqDotStatus();
                break;  
            }
            case PR_FACTORY_DFLTS: {
                t_handlePrFactoryDefaults();
                break;  
            }
            case PR_START_GAP_CALIBRATION: {
                t_handlePrStartGapCal();
                break;  
            }
            case PR_CAL_GAP_SENSOR_NEXT: {                
                t_handlePrGapCalNext();
                break;  
            }
            case PR_START_TU_CALIBRATION: {
                t_handlePrStartTUCal();
                break;  
            }
            case PR_CAL_TU_SENSOR_NEXT: {                
                t_handlePrTUCalNext();
                break;  
            }
            case PR_USE_CONTINUOUS_STOCK: {
                t_handlePrUseContinuousStock();
                break;  
            }
            case PR_MASK: {
                t_handlePrMask( pMsg_ );
                break;
            }
            case PR_PCBA_REVISION: {
                t_handlePrPCBA_ID( pMsg_ );
                break;
            }
            case PR_STATION_ID: {
                t_handleStationId( *( pMsg_ + 1 ) );
                break;
            }
            case PR_LABEL_SIZE: {
                t_handlePrLabelSize( pMsg_ );
                break;
            }
            case PR_POWER: {
                t_handlePrHeadPower( pMsg_ );
                break;
            }
            case PR_REQ_SYS_INFO: {
                t_handleReqSysInfo();
                break;
            }
            case PR_SET_TIMER: {
                t_handlePrSetTime( pMsg_ );
                break;
            }
            case PR_STOP_TIME:{
                t_handlePrStopTime( pMsg_ );
                break;
            }
            case PR_SET_HT_GAP_SIZE: {
                t_handlePrSetHTGapSize( pMsg_ );
                break;  
            }
            case PR_SET_GT_GAP_SIZE: {
                t_handlePrSetGTGapSize( pMsg_ );
                break;  
            }
            case PR_SET_LOW_LABEL_MIN_MAX: {
                t_handlePrSetLowLabelMinMax( pMsg_ );
                break;  
            }
            case PR_CAL_PRINTHEAD_RESISTANCE: {
                t_handlePrCalPrintheadResistance( pMsg_ );
                break;  
            }
            default: {
                /* global printer does not support PR_FLUSH (16)! */
                if( *pMsg_ != 16 ) {
                    //PRINTF("t_handlePrinterMessage(): Unknown message type: %d!\r\n", *pMsg_ );
                }
            }
        }
    } else {
        PRINTF("t_handlePrinterMessage(): pMsg_ is NULL!\r\n" );    
    }
}

#else 
/******************************************************************************/
/*!   \fn static void t_handlePrinterMessage( HUSBMessage *pMsg_ )

      \brief
        handles incomming usb hobart printer messages.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrinterMessage( HUSBMessage *pMsg_ )
{   
    if( pMsg_ != NULL ) {
        switch( pMsg_->hdr.msgType )
        {
            case PR_WAKEUP: {                
                t_handlePrReqWakeup();                
                break;  
            }
            case PR_CONFIG: {
                t_handlePrWriteConfig( pMsg_->pMsg );
                break;
            }
            case PR_REQ_CONFIG: {
                t_handlePrReqConfig();
                break;  
            }
            case PR_RESET: {
                t_handlePrReset();
                break;  
            }
            case PR_REQ_STATUS: {
                t_handlePrReqStatus();
                break;  
            }
            case PR_ENABLE: {
                t_handlePrEnable();
                break;  
            }
            case PR_DISABLE: {
                t_handlePrDisable();
                break;  
            }
            case PR_MODE: {
                t_handlePrMode( (unsigned char)*pMsg_->pMsg );
                break;  
            }
            case PR_REQ_SENSORS: {
                t_handlePrReqSensors();
                break;  
            }
            case PR_RAM: {
                t_handlePrRam( pMsg_->pMsg );
                break;
            }
            case PR_COMMAND: {
                t_handlePrCommand( pMsg_->pMsg );
                break;  
            }
            case PR_REQ_CALIBRATE: {
                t_handlePrReqCalibration();
                break;  
            }
            case PR_SIZE: {
                t_handlePrNomSize(  pMsg_->pMsg );
                break;  
            }
            case PR_ENABLE_TAKEUP: {
                t_handlePrEnableTakeup();
                break;  
            }
            case PR_REQ_VERSION: {
                t_handlePrReqVersion();
                break;  
            }
            case PR_REQ_HEAD_TYPE: {
                t_handlePrReqHeadType();
                break;  
            }
            case PR_CUTTER_CUT: {
                t_handlePrCutterCut();
                break;  
            }
            case PR_REQ_CUTTER_STATUS: {
                t_handlePrReqCutterStatus();
                break;  
            }
            case PR_ERASE_CUTTER_BIT: {
                t_handlePrCutterDefault();
                break;
            }
            case PR_TEACH: {
                t_handlePrTeach( pMsg_->pMsg );
                break;
            }
            case PR_REQ_TRANSFER: {
                t_handlePrReqLabelTransfer( pMsg_->pMsg );
                break;
            }
            case PR_REQ_DOT_WEAR: {
                t_handlePrReqDotWear();
                break;  
            }
            case PR_FACTORY_DFLTS: {
                t_handlePrFactoryDefaults();
                break;  
            }
            case PR_START_GAP_CALIBRATION: {
                t_handlePrStartGapCal();
                break;  
            }
            case PR_CAL_GAP_SENSOR_NEXT: {
                t_handlePrGapCalNext();
                break;  
            }
            case PR_USE_CONTINUOUS_STOCK: {
                t_handlePrUseContinuousStock();
                break;  
            }
            case PR_MASK: {
                t_handlePrMask( pMsg_->pMsg );
                break;
            }
            case PR_PCBA_REVISION: {
                t_handlePrPCBA_ID( pMsg_ );
                break;
            }
            case PR_STATION_ID: {
                t_handleStationId( (unsigned char)*pMsg_->pMsg );
                break;
            }
            case PR_LABEL_SIZE: {
                t_handlePrLabelSize( pMsg_->pMsg );
                break;
            }
            case PR_POWER: {
                t_handlePrHeadPower( pMsg_->pMsg );
                break;
            }
            case PR_REQ_SYS_INFO: {
                t_handleReqSysInfo();
                break;
            }
            case PR_SET_TIMER: {
                t_handlePrSetTime( pMsg_ );
                break;
            }            
            default: {
                //PRINTF("t_handlePrinterMessage(): Unknown message type: %d!\r\n", pMsg_->hdr.msgType );
            }
        }
    } else {
        PRINTF("t_handlePrinterMessage(): pMsg_ is NULL!\r\n" );    
    }
}
#endif

#ifdef LEGACY_PROTOCOL
/******************************************************************************/
/*!   \fn static void t_handleWeigherMessage( unsigned char *pMsg_ )

      \brief
        private handles the incomming usb hobart weigher message
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleWeigherMessage( unsigned char *pMsg_, unsigned int msgSize )
{
    static unsigned char msgType = 0;
    if( pMsg_ != NULL ) {
        if(!msgTracker.handlingMsg) {
            msgType = *pMsg_;
        }
        
        switch( msgType )
        {
            case WG_WAKEUP: {
                t_handleReqWakeup();
                break;  
            }
            case WG_CONFIG: {
                t_handleWriteConfig( pMsg_ );
                break;  
            }
            case WG_REQ_CONFIG: {
                t_handleReqConfig();
                break;  
            }
            case WG_RESET: {
                t_handleReset();
                break;  
            }
            case WG_STATUS: {
                t_handleReqStatus();
                break;  
            }
            case WG_ENABLE: {
                t_handleEnable();
                break;  
            }
            case WG_DISABLE: {
                t_handleDisable();
                break;  
            }
            case WG_MODE: {
                t_handleModeMsg( *( pMsg_ + 1 ) );
                break;  
            }
            case WG_REZERO: {
                t_handleRezero();
                break;  
            }
            case WG_CONTROL: {
                t_handleControl( *( pMsg_ + 1 ) );
                break;  
            }
            case WG_CAT3_REQ_STATISTICS: {
                t_handleReqCat3Statistics();
                break;  
            }
            case WG_CAT3_DATE_TIME: {
                t_handleCat3DateTime( pMsg_ );
                break;  
            }
            case WG_CAT3_REQ_PAGE: {
                t_handleReqCat3Page( *( pMsg_ + 1 ), *( pMsg_ + 2 ) );
                break;  
            }
            case WG_CAT3_REQ_NEXT_RECORDS: {
                t_handleReqCat3NextRecord();
                break;  
            }
            case WG_CAT3_WRITE_RECORD: {
                t_handleCat3RecordWrite( pMsg_ );
                break;  
            }
            case WG_CAT3_REQ_ERASE_LOG: {
                t_handleReqCat3EraseLog();
                break;
            }
            case WG_REQ_WEIGHT: {
                t_handleReqWeight();
                break;
            }
            case WG_REQ_VERSION: {
                t_handleReqVersion();
                break;
            }
            case WG_VM_REQUEST: {
                t_handleValueMaxRequest( pMsg_ ); 
                break;
            }
            case WG_VM_WRITE_ASSEMBLY: {
                t_handleValueMaxWriteAssembly( pMsg_ );
                break;
            }
            case WG_VM_WRITE_VERSION: {
                t_handleValueMaxWriteVersion( pMsg_ );
                break;
            }
            case WG_VM_WRITE_SERIAL: {
                t_handleValueMaxWriteSerial( pMsg_ );
                break;
            }
            case WG_VM_WRITE_DATE: {
                t_handleValueMaxWriteDate( pMsg_ );
                break;
            }
            case WG_REQ_SYSTEM_INFO: {
                t_handleReqInfo();
                break;
            }
            case WG_FACTORY_DEFAULTS: {
                t_handleResetDefaultCfg();
                break;
            }
            case WG_VM_WRITE_FIT_VALUES: {
                t_handleWriteVMFit( pMsg_ );
                break;
            }
            case WG_REQ_SYS_INFO: {
                t_handleWgReqSysInfo();
                break;
            }
            default: {
                PRINTF("t_handleWeigherMessage(): Unknown message type: %d\r\n", msgType );
            }
        }
    } else {
        PRINTF("t_handleWeigherMessage(): pMsg_ is NULL!\r\n" );    
    }
}

#else
/******************************************************************************/
/*!   \fn static void t_handleWeigherMessage( HUSBMessage *pMsg_ )

      \brief
        private handles the incomming usb hobart weigher message
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleWeigherMessage( HUSBMessage *pMsg_ )
{
    if( pMsg_ != NULL ) {
        switch( pMsg_->hdr.msgType )
        {
            case WG_WAKEUP: {
                t_handleReqWakeup();
                break;  
            }
            case WG_CONFIG: {
                t_handleWriteConfig( pMsg_->pMsg );
                break;  
            }
            case WG_REQ_CONFIG: {
                t_handleReqConfig();
                break;  
            }
            case WG_RESET: {
                t_handleReset();
                break;  
            }
            case WG_STATUS: {
                t_handleReqStatus();
                break;  
            }
            case WG_ENABLE: {
                t_handleEnable();
                break;  
            }
            case WG_DISABLE: {
                t_handleDisable();
                break;  
            }
            case WG_MODE: {
                t_handleModeMsg( *pMsg_->pMsg );
                break;  
            }
            case WG_REZERO: {
                t_handleRezero();
                break;  
            }
            case WG_CONTROL: {
                t_handleControl( *pMsg_->pMsg );
                break;  
            }
            case WG_CAT3_REQ_STATISTICS: {
                t_handleReqCat3Statistics();
                break;  
            }
            case WG_CAT3_DATE_TIME: {
                t_handleCat3DateTime( pMsg_->pMsg );
                break;  
            }
            case WG_CAT3_REQ_PAGE: {
                t_handleReqCat3Page( pMsg_->pMsg[0], pMsg_->pMsg[1] );
                break;  
            }
            case WG_CAT3_REQ_NEXT_RECORDS: {
                t_handleReqCat3NextRecord();
                break;  
            }
            case WG_CAT3_WRITE_RECORD: {
                t_handleCat3RecordWrite( pMsg_->pMsg );
                break;  
            }
            case WG_CAT3_REQ_ERASE_LOG: {
                t_handleReqCat3EraseLog();
                break;
            }
            case WG_REQ_WEIGHT: {
                t_handleReqWeight();
                break;
            }
            case WG_REQ_VERSION: {
                t_handleReqVersion();
                break;
            }
            case WG_VM_REQUEST: {
                t_handleValueMaxRequest( pMsg_->pMsg ); 
                break;
            }
            case WG_VM_WRITE_ASSEMBLY: {
                t_handleValueMaxWriteAssembly( pMsg_->pMsg );
                break;
            }
            case WG_VM_WRITE_VERSION: {
                t_handleValueMaxWriteVersion( pMsg_->pMsg );
                break;
            }
            case WG_VM_WRITE_SERIAL: {
                t_handleValueMaxWriteSerial( pMsg_->pMsg );
                break;
            }
            case WG_VM_WRITE_DATE: {
                t_handleValueMaxWriteDate( pMsg_->pMsg );
                break;
            }
            case WG_REQ_SYSTEM_INFO: {
                t_handleReqInfo();
                break;
            }
            case WG_FACTORY_DEFAULTS: {
                t_handleResetDefaultCfg();
                break;
            }
            case WG_VM_WRITE_FIT_VALUES: {
                t_handleWriteVMFit( pMsg_->pMsg );
                break;
            }
            case WG_REQ_SYS_INFO: {
                t_handleWgReqSysInfo();
                break;
            }
            default: {
                PRINTF("t_handleWeigherMessage(): Unknown message type!\r\n" );
            }
        }
    } else {
        PRINTF("t_handleWeigherMessage(): pMsg_ is NULL!\r\n" );    
    }
}
#endif

/**************************PRINTER*********************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#pragma diag_suppress=Pa039  /* side affect of using #pragma pack */    

/******************************************************************************/
/*!   \fn void sendPrWakeup( PrWakeup *pWakeMsg)

      \brief
        handles converting printer wakeup message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendPrWakeup( PrWakeup *pWakeMsg )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof( PrWakeup ) );
    
    /* post message to usbtasks send queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWakeup(): pQUSBSendPrHandle_ is null!\r\n");
    }
    

    unsigned char body[ sizeof( PrWakeup ) ]; 
    unsigned int index = 0;
    /* initialize body of the usb message */
    body[index++] = pWakeMsg->msgType;
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->id, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->id, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->buffer_size, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->buffer_size, 0);    
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->printhead_size, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->printhead_size, 0);    
      
	
    if(getPrintHeadType() == ROHM_72MM_800_OHM) {        
        body[index++] = pWakeMsg->label_width = UFW_LABEL_STOCK; 
    } else {
        body[index++] = pWakeMsg->label_width = UFW_LABEL_STOCK;  
    }
		
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->transfer_size, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->transfer_size, 0 );
    body[index++] = pWakeMsg->configValid;   
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWakeup(): pQUSBSendPrHandle_ is null!\r\n");
    }
   
    #else  
    
    unsigned char frame[ ( sizeof(PrWakeup) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrWakeup) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_WAKEUP;
    usbMsg.hdr.msgSize          = (unsigned long)sizeof(PrWakeup);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrWakeup);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->id, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->id, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->buffer_size, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->buffer_size, 0);    
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->printhead_size, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->printhead_size, 0);    
    frame[index++] = pWakeMsg->label_width;    
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->transfer_size, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->transfer_size, 0 );
    frame[index++] = pWakeMsg->cutterInstalled;   
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWakeup(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    #endif    
}

/******************************************************************************/
/*!   \fn void sendWgSysInfo( WgSysInfo *pSysInfo )

      \brief
        handles converting weigher sysinfo message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Joseph DiCarlantonio
*******************************************************************************/
void sendWgSysInfo( WgSysInfo *pSysInfo )
{
    #ifdef LEGACY_PROTOCOL
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgSysInfo) );
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWrSysInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWrSysInfo(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgSysInfo ) ]; 
    unsigned int index = 0;
    /* initialize body of the usb message */
    body[index++] = pSysInfo->msgType; 
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 0 );
    body[index++] = pSysInfo->valueMaxEnabled; 
 
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWrSysInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWrSysInfo(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    
    #else
    
    unsigned char frame[ ( sizeof(WgSysInfo) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgSysInfo) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_SYS_INFO;
    usbMsg.hdr.msgSize          = (unsigned long)sizeof(WgSysInfo);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgSysInfo);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 0 );
    frame[index++] = pSysInfo->valueMaxEnabled;
 
 
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWrSysInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWrSysInfo(): pQUSBSendWrHandle_ is null!\r\n");
    } 
    
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrSysInfo( PrSysInfo *pSysInfo )

      \brief
        handles converting printer sysinfo message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendPrSysInfo( PrSysInfo *pSysInfo )
{
    #ifdef LEGACY_PROTOCOL
      
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrSysInfo) );
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrSysInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrSysInfo(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrSysInfo ) ]; 
    unsigned int index = 0;
    /* initialize body of the usb message */
    body[index++] = pSysInfo->msgType; 
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 0 );
    body[index++] = pSysInfo->id; 
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->buffer_size, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->buffer_size, 0);    
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->printhead_size, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->printhead_size, 0);        
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->transfer_size, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pSysInfo->transfer_size, 0 );
    body[index++] = pSysInfo->label_width;
    body[index++] = pSysInfo->headType;
    body[index++] = pSysInfo->cutterInstalled;   
    body[index++] = pSysInfo->cutterEnabled;   
    body[index++] = pSysInfo->configValid;   
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        //PRINTF("sendPrSysInfo()\r\n");
        if( result != pdPASS ) {
            PRINTF("sendPrSysInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrSysInfo(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    #else
    
    unsigned char frame[ ( sizeof(PrSysInfo) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrSysInfo) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_SYS_INFO;
    usbMsg.hdr.msgSize          = (unsigned long)sizeof(PrSysInfo);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrSysInfo);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->pid, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->id, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->id, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->buffer_size, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->buffer_size, 0);    
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->printhead_size, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->printhead_size, 0);        
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->transfer_size, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pSysInfo->transfer_size, 0 );

    if( getPrintHeadType() == ROHM_72MM_800_OHM ) {
        frame[index++] = pSysInfo->label_width = UFW_LABEL_STOCK;
    } else {
        frame[index++] = pSysInfo->label_width = UFW_LABEL_STOCK;
    }

    frame[index++] = pSysInfo->headType;
    frame[index++] = pSysInfo->cutterInstalled;   
    frame[index++] = pSysInfo->cutterEnabled;   
    frame[index++] = pSysInfo->configValid;   
 
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrSysInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrSysInfo(): pQUSBSendPrHandle_ is null!\r\n");
    } 
    
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrConfig( Pr_Config *pWakeMsg)

      \brief
        handles converting printer config message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendPrConfig( Pr_Config *pConfig )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], ( sizeof(Pr_Config) + 1 ) );
          
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrConfig(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrConfig(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( Pr_Config ) ]; 
    unsigned int index = 0;

    /* initialize body of the usb message */
    body[index++] = PR_CONFIG;
    body[index++] = pConfig->instance;
    

    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        //body[index++] = pConfig->label_width = GLOBAL_LABEL_STOCK;
        body[index++] = pConfig->label_width = UFW_LABEL_STOCK;
    }
    else
    {
        body[index++] = pConfig->label_width = UFW_LABEL_STOCK;
    }

    body[index++] = pConfig->shootThroughCal;
    body[index++] = pConfig->out_of_media_count;    
    body[index++] = pConfig->contrast_adjustment; 
    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->expel_position, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->expel_position, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->peel_position, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->peel_position, 0);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->retract_position, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->retract_position, 0);  
    
    pConfig->media_sensor_type = _SHOOTTHROUGH;
    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->media_sensor_type, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->media_sensor_type, 0);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->printheadResistance, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->printheadResistance, 0);   
	body[index++] = getCharFromShort( (unsigned short *)&pConfig->verticalPosition, 1);    
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->verticalPosition, 0);    

    body[index++] = pConfig->backingPaper;
    body[index++] = pConfig->backingAndlabel;
    body[index++] = pConfig->labelCalCnts;
    body[index++] = pConfig->noLabelCalCnts;
    body[index++] = pConfig->cutterEnabled;
    body[index++] = pConfig->takeupDriveCurrent;
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->takeupMaxTension, 1);
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->takeupMaxTension, 0);
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->takeupMinTension, 1);
    body[index++] = getCharFromShort( (unsigned short *)&pConfig->takeupMinTension, 0);
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrConfig(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrConfig(): pQUSBSendPrHandle_ is null!\r\n");
    }    
    
    #else 
    
    unsigned char frame[ ( sizeof(Pr_Config) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(Pr_Config) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_CONFIG;
    usbMsg.hdr.msgSize          = sizeof(Pr_Config);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(Pr_Config);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = pConfig->instance;
    	
    if( getPrintHeadType() == ROHM_72MM_800_OHM ) {
        frame[index++] = pConfig->label_width = UFW_LABEL_STOCK;
    } else {
        frame[index++] = pConfig->label_width = UFW_LABEL_STOCK;
    }
	
    frame[index++] = pConfig->media_sensor_adjustment;
    frame[index++] = pConfig->out_of_media_count;    
    frame[index++] = getCharFromShort( &pConfig->contrast_adjustment, 1 ); 
    frame[index++] = getCharFromShort( &pConfig->contrast_adjustment, 0 );
    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->expel_position, 1 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->expel_position, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->peel_position, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->peel_position, 0);    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->retract_position, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->retract_position, 0);    

    pConfig->media_sensor_type = _SHOOTTHROUGH;
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->media_sensor_type, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->media_sensor_type, 0);    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->verticalPosition, 1);    
    frame[index++] = getCharFromShort( (unsigned short *)&pConfig->verticalPosition, 0);    

    frame[index++] = pConfig->backingPaper;
    frame[index++] = pConfig->backingAndlabel;
    frame[index++] = pConfig->labelCalCnts;
    frame[index++] = pConfig->noLabelCalCnts;
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrConfig(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrConfig(): pQUSBSendPrHandle_ is null!\r\n");
    }
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrStatus( PrStatusInfo *pStatus, bool interrupt )

      \brief
        handles converting printer status message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendPrStatus( PrStatusInfo *pStatus, bool interrupt )
{

    #ifdef LEGACY_PROTOCOL   
    unsigned char frame[ sizeof( USBHeader ) ]; 
    unsigned char frameIsr[ sizeof( USBHeader ) ]; 

    //TFinkQueueSetFix
   if(isQueueAlmostFullISR(pQUSBSendPrHandle_, MAX_USB_IN_LENGTH, 4, interrupt))
   {
      PRINTF("\r\nskipped sendPrStatus\r\n"); 
      return;
   }
   
    BaseType_t result;
    if( interrupt ) {
        initPrHeader( &frameIsr[0], ( sizeof(PrStatusInfo) + 1 ) );
     } else {
        initPrHeader( &frame[0], ( sizeof(PrStatusInfo) + 1 ) );
     }
           
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL) {
        if( interrupt ) {
            result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frameIsr[0], 0 );
        } else {
            result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        }
        if( result != pdPASS ) {
            PRINTF("sendPrStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrStatus(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrStatusInfo ) ];
    unsigned char bodyIsr[ sizeof( PrStatusInfo ) ];
    unsigned int index = 0;
    unsigned int indexIsr = 0;
    
    if( interrupt ) {
        bodyIsr[indexIsr++] = PR_STATUS;                  
        bodyIsr[indexIsr++] = pStatus->state;
        bodyIsr[indexIsr++] = pStatus->command;
        bodyIsr[indexIsr++] = pStatus->error;
        bodyIsr[indexIsr++] = pStatus->history;
        bodyIsr[indexIsr++] = pStatus->sensor;    
        bodyIsr[indexIsr++] = pStatus->user;    
        bodyIsr[indexIsr++] = getCharFromShort( (unsigned short *)&pStatus->counter, 1 );    
        bodyIsr[indexIsr++] = getCharFromShort( (unsigned short *)&pStatus->counter, 0 );    
        bodyIsr[indexIsr++] = pStatus->mask.sensor;    
        bodyIsr[indexIsr++] = pStatus->mask.user;    
        bodyIsr[indexIsr++] = pStatus->mask.sensor2;  
        bodyIsr[indexIsr++] = pStatus->sla;
        bodyIsr[indexIsr++] = pStatus->sensor2;
        bodyIsr[indexIsr++] = pStatus->labelLowPercentage;
    } else {
        body[index++] = PR_STATUS;                  
        body[index++] = pStatus->state;
        body[index++] = pStatus->command;
        body[index++] = pStatus->error;
        body[index++] = pStatus->history;
        body[index++] = pStatus->sensor;    
        body[index++] = pStatus->user;    
        body[index++] = getCharFromShort( (unsigned short *)&pStatus->counter, 1 );    
        body[index++] = getCharFromShort( (unsigned short *)&pStatus->counter, 0 );    
        body[index++] = pStatus->mask.sensor;    
        body[index++] = pStatus->mask.user;    
        body[index++] = pStatus->mask.sensor2;  
        body[index++] = pStatus->sla;
        body[index++] = pStatus->sensor2;  
        body[index++] = pStatus->labelLowPercentage;  
    }
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL) {
        if( !interrupt ) { 
            BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
            if( result != pdPASS ) {
                PRINTF("sendPrStatus(): USB Tx Message queue is full!\r\n");
            }
        } else {
            BaseType_t xHigherPriorityTaskWoken = false;
            BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&bodyIsr[0], &xHigherPriorityTaskWoken );
            if( result != pdPASS ) {
                PRINTF("sendPrStatus(): USB Tx Message queue is full!\r\n");
            }    
        }
    } else {
        PRINTF("sendPrStatus(): USB Tx Message queue is null!\r\n");    
    }
    #else
    unsigned char frame[ ( sizeof(PrStatusInfo) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrStatusInfo) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_REQ_STATUS;
    usbMsg.hdr.msgSize          = sizeof(PrStatusInfo);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrStatusInfo);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
    
    frame[index++] = pStatus->state;
    frame[index++] = pStatus->command;
    frame[index++] = pStatus->error;
    frame[index++] = pStatus->history;
    frame[index++] = pStatus->sensor;    
    frame[index++] = pStatus->user;    
    frame[index++] = getCharFromShort( (unsigned short *)&pStatus->counter, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pStatus->counter, 0 );    
    frame[index++] = pStatus->mask.sensor;    
    frame[index++] = pStatus->mask.user;    
    frame[index++] = pStatus->mask.sensor2;  
    frame[index++] = pStatus->sla;
    frame[index++] = pStatus->sensor2;
    frame[index++] = pStatus->labelLowPercentage;
  
    /* for now .... segfault here when added */
    pQUSBSendPrHandle_ = NULL;    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        if( !interrupt ) { 
            BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
            if( result != pdPASS ) {
                PRINTF("sendPrStatus(): USB Tx Message queue is full!\r\n");
            }
        } else {
            BaseType_t xHigherPriorityTaskWoken = false;
            BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], &xHigherPriorityTaskWoken );
            if( result != pdPASS ) {
                PRINTF("sendPrStatus(): USB Tx Message queue is full!\r\n");
            }    
        }
    } else {
        PRINTF("sendPrStatus(): USB Tx Message queue is null!\r\n");    
    }
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrSensors(  PrSensors *pSensors )

      \brief
        handles converting printer sensors message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendPrSensors( PrSensors *pSensors )
{    
    #ifdef LEGACY_PROTOCOL 

    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], ( sizeof( GPrSensors ) ) );
      
    if(isQueueAlmostFull(pQUSBSendPrHandle_, MAX_USB_IN_LENGTH, 4))
    {
        PRINTF("\r\nskipped sendPrSensors\r\n");
        return;    
    }
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrSensors(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrSensors(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ ( sizeof( GPrSensors ) ) ]; 
    unsigned int index = 0;

    /* get current state of the printhead */
    if( ( currentStatus.error & HEAD_UP ) == HEAD_UP ) {
        pSensors->headup_reading = 1;
    } else {
        pSensors->headup_reading = 0;
    }            
    
        
    GPrSensors sensors;
    sensors.msgType = PR_GLOBAL_SENSORS;
    sensors.headUp = pSensors->headup_reading;
    
    if( getPrintHeadType() == ROHM_72MM_800_OHM ) {        
        sensors.labelWidth = UFW_LABEL_STOCK;
    } else {
        sensors.labelWidth = UFW_LABEL_STOCK;
    }
	
    sensors.lowStock = pSensors->lowStock;
    
    if( getLabelTaken() >= LABEL_TAKEN_THRESHOLD_LABEL ) {
        sensors.labelTaken = 0;
    } else {
        sensors.labelTaken = 1;
    }
      
    sensors.labelTaken = readLabelTakenSensor();
    sensors.headTemperature 	        = pSensors->head_temperature_reading;
    sensors.media_reading 		= pSensors->media_reading;
    sensors.headVoltage 		= pSensors->head_voltage_reading;
    sensors.takeup_reading		= pSensors->takeup_reading;
      
    /* initialize body of the usb message */
    body[index++] = sensors.msgType;
    body[index++] = sensors.headUp;    
    body[index++] = sensors.labelWidth;    
    body[index++] = sensors.lowStock;    
    body[index++] = sensors.labelTaken;        
    body[index++] = sensors.headTemperature;    
        
    body[index++] = getCharFromShort( (unsigned short *)&sensors.media_reading, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&sensors.media_reading, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&sensors.headVoltage, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&sensors.headVoltage, 0 );    
	
    body[index++] = getCharFromShort( (unsigned short *)&sensors.takeup_reading, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&sensors.takeup_reading, 0 );  
    
    body[index++] = getCharFromShort( (unsigned short *)&sensors.labelTakenReading, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&sensors.labelTakenReading, 0 );  
		
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrSensors(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrSensors(): USB Tx Message queue is null!\r\n");    
    }   
    
    #else
    
    unsigned char frame[ ( sizeof(PrSensors) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrSensors) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_SENSORS;
    usbMsg.hdr.msgSize          = sizeof(PrSensors);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrSensors);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->headup_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->headup_reading, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_width_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_width_reading, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->head_temperature_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->head_temperature_reading, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->head_voltage_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->head_voltage_reading, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->head_current_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->head_current_reading, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_high_average, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_high_average, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_low_average, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_low_average, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_threshold, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_threshold, 0 );    
    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->label_reading, 0 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_high_average, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_high_average, 0 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_low_average, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_low_average, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_threshold, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_threshold, 0 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_reading, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pSensors->media_reading, 0 );    

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrSensors(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrSensors(): USB Tx Message queue is null!\r\n");    
    } 
    #endif
}


/******************************************************************************/
/*!   \fn void sendPrVersion( PrVersion version )

      \brief
        handles converting printer config message into usb frames and 
        adding to usb transmit queue for sending. 
   
      \author
          Aaron Swift
*******************************************************************************/
void sendPrVersion( PrVersion *pVersion  )
{
    #ifdef LEGACY_PROTOCOL 
  
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrVersion) );
       
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrVersion(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrVersion ) ]; 
    unsigned int index = 0;

    /* initialize body of the usb message */
    body[index++] = pVersion->msgType;
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->pid, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->pid, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&pVersion->major, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->major, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&pVersion->minor, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->minor, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&pVersion->build, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->build, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMajor, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMajor, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMinor, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMinor, 0 );    

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrVersion(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrVersion(): USB Tx Message queue is null!\r\n");    
    }     
    
    #else
    unsigned char frame[ ( sizeof(PrVersion) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrVersion) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_REQ_VERSION;
    usbMsg.hdr.msgSize          = sizeof(PrVersion);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrVersion);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->pid, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->pid, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->major, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->major, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->minor, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->minor, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->build, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->build, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMajor, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMajor, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMinor, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMinor, 0 );    

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrVersion(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrVersion(): USB Tx Message queue is null!\r\n");    
    }    
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrHeadType( PrHead *pType )

      \brief
        handles converting printer head type message into can frames and 
        adding to flexcan transmit queue for sending. 
   
      \author
          Aaron Swift
*******************************************************************************/
void sendPrHeadType( PrHead *pType )
{
    #ifdef LEGACY_PROTOCOL 
    
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrHead) );
           
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrHeadType(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrHeadType(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrHead ) ]; 
    unsigned int index = 0;

    body[index++] = pType->msgType;
    body[index++] = pType->headType; 
    body[index++] = getCharFromShort( &pType->headSize, 1 );
    body[index++] = getCharFromShort( &pType->headSize, 0 );
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrHeadType(): USB tx message queue is full!\r\n");
        } 
    } else {
        PRINTF("sendPrHeadType(): USB Tx Message queue is null!\r\n");    
    }  
    
    #else
    
    unsigned char frame[ ( sizeof(PrHead) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrHead) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_HEAD_TYPE;
    usbMsg.hdr.msgSize          = sizeof(PrHead);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrHead);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pType->headType; 
    frame[index++] = getCharFromShort( &pType->headSize, 1 );
    frame[index++] = getCharFromShort( &pType->headSize, 0 );
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrHeadType(): USB tx message queue is full!\r\n");
        } 
    } else {
        PRINTF("sendPrHeadType(): USB Tx Message queue is null!\r\n");    
    } 
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrCutterStatus( PrCutterStatus *pStatus )

      \brief
        handles converting printer board test complete message into can frames and 
        adding to flexcan transmit queue for sending. 
   
      \author
          Aaron Swift
*******************************************************************************/                          
void sendPrCutterStatus( PrCutterStatus *pStatus )
{
    #ifdef LEGACY_PROTOCOL 
    
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    if( isQueueAlmostFull( pQUSBSendPrHandle_, MAX_USB_IN_LENGTH, 4 ) ) {
        PRINTF("\r\nskipped sendPrCutterStatus\r\n");
        return;
    }
    
    initPrHeader( &frame[0], sizeof(PrCutterStatus) );
       
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrCutterStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrCutterStatus(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrCutterStatus ) ]; 
    unsigned int index = 0;

    body[index++] = pStatus->msgType;
    body[index++] = pStatus->home;
    body[index++] = pStatus->saddleInterlock;
    body[index++] = pStatus->doorInterlock;
    body[index++] = pStatus->jammed;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {    
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrCutterStatus(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrCutterStatus(): USB Tx Message queue is null!\r\n");    
    } 
    
    #else
    unsigned char frame[ ( sizeof(PrCutterStatus) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrCutterStatus) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_CUTTER_STATUS;
    usbMsg.hdr.msgSize          = sizeof(PrCutterStatus);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrCutterStatus);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pStatus->home;
    frame[index++] = pStatus->saddleInterlock;
    frame[index++] = pStatus->doorInterlock;
    frame[index++] = pStatus->jammed;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {    
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrCutterStatus(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrCutterStatus(): USB Tx Message queue is null!\r\n");    
    } 
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrCutterStatusFromISR( PrCutterStatus *pStatus )

      \brief
        handles converting printer board test complete message into can frames and 
        adding to flexcan transmit queue for sending. 
   
      \author
          Aaron Swift
*******************************************************************************/                          
BaseType_t sendPrCutterStatusFromISR( PrCutterStatus *pStatus )
{
    BaseType_t result = pdFALSE;
    
    #ifdef LEGACY_PROTOCOL 
    
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrCutterStatus) );
           
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrCutterStatusFromISR(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrCutterStatusFromISR(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrCutterStatus ) ]; 
    unsigned int index = 0;

    body[index++] = pStatus->msgType;
    body[index++] = pStatus->home;
    body[index++] = pStatus->saddleInterlock;
    body[index++] = pStatus->doorInterlock;
    body[index++] = pStatus->jammed;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {    
        result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrCutterStatusFromISR(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrCutterStatusFromISR(): USB Tx Message queue is null!\r\n");    
    } 
    
    #else
    unsigned char frame[ ( sizeof(PrCutterStatus) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(PrCutterStatus) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_CUTTER_STATUS;
    usbMsg.hdr.msgSize          = sizeof(PrCutterStatus);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrCutterStatus);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pStatus->home;
    frame[index++] = pStatus->installed;
    frame[index++] = pStatus->interlock;
    frame[index++] = pStatus->jammed;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {    
        result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrCutterStatusFromISR(): USB tx message queue is full!\r\n");
        } 
    } else {
        PRINTF("sendPrCutterStatusFromISR(): USB Tx Message queue is null!\r\n");    
    }
    #endif    
    return result;
    
}

/******************************************************************************/
/*!   \fn void sendPrTransferReady( bool ready )

      \brief
        handles converting transfer ready message into can frames and 
        adding to flexcan transmit queue for sending. 
   
      \author
          Aaron Swift
*******************************************************************************/                          
void sendPrTransferReady( bool ready )
{
    #ifdef LEGACY_PROTOCOL 
    
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(bool) );
       
    if( isQueueAlmostFull( pQUSBSendPrHandle_, MAX_USB_IN_LENGTH, 4 ) ) {
        PRINTF("\r\nskipped sendPrTransferReady\r\n");
        return;
    }
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrTransferReady(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrTransferReady(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( bool ) ]; 
    body[0] = PR_TRANSFER_READY;
    body[1] = ready;
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {    
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrTransferReady(): USB tx message queue is full!\r\n");
        } 
    } else {
        PRINTF("sendPrTransferReady(): USB Tx Message queue is null!\r\n");    
    }              
    
    #else
    unsigned char frame[ ( sizeof(unsigned char) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_TRANSFER_READY;
    usbMsg.hdr.msgSize          = 1;                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = 1;
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index] = ready;
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {    
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrTransferReady(): USB tx message queue is full!\r\n");
        } 
    } else {
        PRINTF("sendPrTransferReady(): USB Tx Message queue is null!\r\n");    
    }              
    #endif  
}


/******************************************************************************/
/*!   \fn void sendPrFactoryDlftsComplete( bool status )

      \brief
        handles sending restoring factory defaults complete message. 

      \author
          Aaron Swift
*******************************************************************************/
void sendPrFactoryDlftsComplete( bool status )
{
    #ifdef LEGACY_PROTOCOL 
 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], ( 1 + sizeof(bool) ) );
           
    // post message to queue     
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrFactoryDlftsComplete(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrFactoryDlftsComplete(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( bool ) ]; 
    unsigned int index = 0;

    body[index++] = PR_FACTORY_DFLTS_CMPLT;
    body[index] = status;
    
    // post message to queue 
    if( pQUSBSendPrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrFactoryDlftsComplete(): USB tx message queue is full!\r\n");
        }    
    } else {
        PRINTF("sendPrFactoryDlftsComplete(): USB Tx Message queue is null!\r\n");    
    }  

    #else

    unsigned char frame[ ( sizeof(unsigned char) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_TRANSFER_READY;
    usbMsg.hdr.msgSize          = 1;                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = 1;
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
    frame[index] = status;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrFactoryDlftsComplete(): USB tx message queue is full!\r\n");
        }    
    } else {
        PRINTF("sendPrFactoryDlftsComplete(): USB Tx Message queue is null!\r\n");    
    }              
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrGapCalStatus( PrGapCalStatus *pPrGapCalStatus )

      \brief
        sends printer gap calibration status message to the host.
   
      \author
          Aaron Swift
*******************************************************************************/
void sendPrGapCalStatus( PrGapCalStatus *pPrGapCalStatus )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrGapCalStatus) );

    pPrGapCalStatus->msgType = PR_GAP_CAL_STATUS;      
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrGapCalStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrGapCalStatus(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrGapCalStatus ) ]; 
    unsigned int index = 0;

    body[index++] = pPrGapCalStatus->msgType;
    body[index++] = pPrGapCalStatus->state;    
        
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->deflectionVoltage, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->deflectionVoltage, 0 );   
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->driveCurrent, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->driveCurrent, 0 );    
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->backingVoltage, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->backingVoltage, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->labelBackVoltage, 1 );
    body[index] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->labelBackVoltage, 0 );
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrGapCalStatus(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrGapCalStatus(): USB Tx Message queue is null!\r\n");    
    }              

    #else

    unsigned char frame[ ( sizeof(unsigned char) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_TRANSFER_READY;
    usbMsg.hdr.msgSize          = 1;                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = 1;
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
        
    frame[index++] = pPrGapCalStatus->state;    
    frame[index++] = pPrGapCalStatus->deflectionVoltage;
    frame[index++] = pPrGapCalStatus->driveCurrent;
    frame[index++] = pPrGapCalStatus->backingVoltage;
    frame[index]   = pPrGapCalStatus->labelBackVoltage;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrGapCalStatus(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrGapCalStatus(): USB Tx Message queue is null!\r\n");    
    }              
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrPeelLog(PrPeelLog *pPrPeelLog);

      \brief
        sends the printer peel statistics. The application will save the 
        label peel statistics to allow peel issues to be quantified
   
      \author
          Tom Fink
*******************************************************************************/
void sendPrPeelLog(PrPeelLog *pPrPeelLog)
{ 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrPeelLog) );

    pPrPeelLog->msgType = PR_PEEL_LOG;      
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrPeelLog(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrPeelLog(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrPeelLog ) ]; 
    unsigned int index = 0;

    body[index++] = pPrPeelLog->msgType;  
        
    body[index++] = getCharFromShort( (unsigned short *)&pPrPeelLog->numMtrStalls, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrPeelLog->numMtrStalls, 0 );   
    body[index++] = getCharFromShort( (unsigned short *)&pPrPeelLog->numRecoverFromMtrStall, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrPeelLog->numRecoverFromMtrStall, 0 );    
    body[index++] = getCharFromShort( (unsigned short *)&pPrPeelLog->numFailToPeel, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrPeelLog->numFailToPeel, 0 );
    body[index++] = getCharFromLong ( (unsigned long  *)&pPrPeelLog->totalLabelsPrinted, 3);
    body[index++] = getCharFromLong ( (unsigned long  *)&pPrPeelLog->totalLabelsPrinted, 2 );
    body[index++] = getCharFromLong ( (unsigned long  *)&pPrPeelLog->totalLabelsPrinted, 1 );
    body[index]   = getCharFromLong ( (unsigned long  *)&pPrPeelLog->totalLabelsPrinted, 0 );
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrPeelLog(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrPeelLog(): USB Tx Message queue is null!\r\n");    
    }              
}


/******************************************************************************/
/*!   \fn void sendPrTUCalStatus( PrGapCalStatus *pPrGapCalStatus )

      \brief
        sends printer TU calibration status message to the host.
   
      \author
          Aaron Swift
*******************************************************************************/
void sendPrTUCalStatus( PrGapCalStatus *pPrGapCalStatus )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrGapCalStatus) );

    pPrGapCalStatus->msgType = PR_GAP_CAL_STATUS;      
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrGapCalStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrGapCalStatus(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrGapCalStatus ) ]; 
    unsigned int index = 0;

    body[index++] = pPrGapCalStatus->msgType;
    body[index++] = pPrGapCalStatus->state;    
        
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->deflectionVoltage, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->deflectionVoltage, 0 );   
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->driveCurrent, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->driveCurrent, 0 );    
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->backingVoltage, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->backingVoltage, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->labelBackVoltage, 1 );
    body[index] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->labelBackVoltage, 0 );
    
	
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrGapCalStatus(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrGapCalStatus(): USB Tx Message queue is null!\r\n");    
    }              

    #else

    unsigned char frame[ ( sizeof(unsigned char) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_TRANSFER_READY;
    usbMsg.hdr.msgSize          = 1;                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = 1;
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
        
    frame[index++] = pPrGapCalStatus->state;    
    frame[index++] = pPrGapCalStatus->deflectionVoltage;
    frame[index++] = pPrGapCalStatus->driveCurrent;
    frame[index++] = pPrGapCalStatus->backingVoltage;
    frame[index]   = pPrGapCalStatus->labelBackVoltage;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrGapCalStatus(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrGapCalStatus(): USB Tx Message queue is null!\r\n");    
    }              
    #endif
}

/******************************************************************************/
/*!   \fn void sendPrTUCalStatusFromISR( PrGapCalStatus *pPrGapCalStatus )

      \brief
        sends printer TU calibration status message to the host.
   
      \author
          Aaron Swift
*******************************************************************************/
void sendPrTUCalStatusFromISR( PrGapCalStatus *pPrGapCalStatus )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrGapCalStatus) );

    pPrGapCalStatus->msgType = PR_GAP_CAL_STATUS;      
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrTUCalStatusFromISR(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrTUCalStatusFromISR(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrGapCalStatus ) ]; 
    unsigned int index = 0;

    body[index++] = pPrGapCalStatus->msgType;
    body[index++] = pPrGapCalStatus->state;    
        
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->deflectionVoltage, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->deflectionVoltage, 0 );   
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->driveCurrent, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->driveCurrent, 0 );    
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->backingVoltage, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->backingVoltage, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->labelBackVoltage, 1 );
    body[index] = getCharFromShort( (unsigned short *)&pPrGapCalStatus->labelBackVoltage, 0 );
    
	/* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrTUCalStatusFromISR(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrTUCalStatusFromISR(): USB Tx Message queue is null!\r\n");    
    }              

    #else

    unsigned char frame[ ( sizeof(unsigned char) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_TRANSFER_READY;
    usbMsg.hdr.msgSize          = 1;                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = 1;
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
        
    frame[index++] = pPrGapCalStatus->state;    
    frame[index++] = pPrGapCalStatus->deflectionVoltage;
    frame[index++] = pPrGapCalStatus->driveCurrent;
    frame[index++] = pPrGapCalStatus->backingVoltage;
    frame[index]   = pPrGapCalStatus->labelBackVoltage;

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrTUCalStatusFromISR(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrTUCalStatusFromISR(): USB Tx Message queue is null!\r\n");    
    }              
    #endif
}

/******************************************************************************/
/*!   \fn void sendCutterVersion( PrCutterVersion *pVer )

      \brief
        handles converting cutter version message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/                          
void sendCutterVersion( PrCutterVersion *pVer )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ];     
    initPrHeader( &frame[0], sizeof(PrCutterVersion) );
    
    pVer->msgType = PR_CUTTER_VERSION;      
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCutterVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCutterVersion(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrCutterVersion ) ]; 
    unsigned int index = 0;

    body[index++] = pVer->msgType;
    for( int i = 0; i < MAX_PROD_NAME_SIZE + 1; i++) {
        body[index++] = pVer->productName[ i ];
    }
    
    for( int i = 0; i < MAX_FIRM_NUM_SIZE + 1; i++) {
        body[index++] = pVer->firmware[ i ];    
    }
    for( int i = 0; i < MAX_FIRM_ISSUE_SIZE + 1; i++) {
        body[index++] = pVer->issueDate[ i ];    
    }
    for( int i = 0; i < MAX_DATE_SZ + 1; i++) {
        body[index++] = pVer->date[ i ];    
    }
    for( int i = 0; i < MAX_TIME_SZ + 1; i++) {
        body[index++] = pVer->time[ i ];    
    }   
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCutterVersion(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendCutterVersion(): USB Tx Message queue is null!\r\n");    
    }              
    #else
  
    
    #endif    
}

/******************************************************************************/
/*!   \fn void sendPrGlobalCutterStatus( PrGblCutterStatus *pStatus )

      \brief
        handles converting cutter status message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/                          
void sendPrGlobalCutterStatus( PrGblCutterStatus *pStatus )
{
    #ifdef LEGACY_PROTOCOL
  
    unsigned char frame[ sizeof( USBHeader ) ];     
    initPrHeader( &frame[0], sizeof(PrCutterVersion) );
    
    pStatus->msgType = PR_GL_CUTTER_STATUS;      
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCutterVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCutterVersion(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrGblCutterStatus ) ]; 
    unsigned int index = 0;

    body[index++] = pStatus->msgType;
    body[index++] = getCharFromShort( (unsigned short *)&pStatus->statusBits, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pStatus->statusBits, 0 );   

    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCutterVersion(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendCutterVersion(): USB Tx Message queue is null!\r\n");    
    }              
    #else
  
    
    #endif      
}

/******************************************************************************/
/*!   \fn void sendPrHeadDotStatus( int size )

      \brief
   
      \author
          Chris King
*******************************************************************************/ 
void sendPrHeadDotStatus(int size)
{
    PrDotStatusMessage dotMessage;
    uint16_t currentDotIndex = 0;

    while (currentDotIndex < DOT_CHECKER_TOTAL_DOTS)
    {
        // Determine how many dots to send this round
        uint8_t dotsThisPacket = (DOT_CHECKER_TOTAL_DOTS - currentDotIndex >= DOT_CHECKER_DOTS_PER_PACKET)
                                 ? DOT_CHECKER_DOTS_PER_PACKET
                                 : (DOT_CHECKER_TOTAL_DOTS - currentDotIndex);  /* last packet = 24 dots */

        /* build message */
        dotMessage.msgType = PR_DOT_STATUS;
        dotMessage.startingDotIndex = currentDotIndex;
        dotMessage.reserved[0] = 0;
        dotMessage.reserved[1] = 0;
        
        /* build header */ 
        unsigned char frame[ sizeof( USBHeader ) ]; 
        
        initPrHeader( &frame[0], sizeof(PrDotStatusMessage) );
        
        /* build body */
        unsigned char body[ sizeof( PrDotStatusMessage ) ]; 
        unsigned int index = 0;
        
        body[index++] = dotMessage.msgType;
        body[index++] = getCharFromShort( (unsigned short *)&dotMessage.startingDotIndex, 1 );
        body[index++] = getCharFromShort( (unsigned short *)&dotMessage.startingDotIndex, 0 );    
        body[index++] = dotMessage.reserved[0];
        body[index++] = dotMessage.reserved[1];

        /* populate the dot data */
        for (uint8_t i = 0; i < dotsThisPacket; i++)
        {
            dotMessage.dots[i] = (unsigned char)getHeadWearDotStatus( currentDotIndex + i );
            
            if( dotMessage.dots[i] == DOT_BAD ) {
                PRINTF("\r\nDOT BAD #%d", currentDotIndex + i);
            } else if( dotMessage.dots[i] == DOT_MARGINAL ) {
                PRINTF("\r\nDOT MARGINAL #%d", currentDotIndex + i);
            }            
            body[index++] = dotMessage.dots[i];
        }

        /* zero-fill unused bytes (only affects last packet) */
        for ( uint8_t i = dotsThisPacket; i < DOT_CHECKER_DOTS_PER_PACKET; i++ ) 
        {
            dotMessage.dots[i] = 0;
            body[index++] = dotMessage.dots[i];
        }
        
        /* post header to queue */    
        if( pQUSBSendPrHandle_ != NULL ) {
            BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
            if( result != pdPASS ) {
                PRINTF("sendPrHeadDotStatus(): USB tx message queue is full!\r\n");
            }
        } else {
            PRINTF("sendPrHeadDotStatus(): pQUSBSendPrHandle_ is null!\r\n");
        }
                       
        if( pQUSBSendPrHandle_ != NULL ) {
            BaseType_t result = xQueueSendFromISR(pQUSBSendPrHandle_, (void *)&dotMessage, 0);                        
            if( result != pdPASS ) {
                PRINTF("sendPrHeadDotStatus(): USB tx message queue is full!\r\n");
            }
        } else {
            PRINTF("sendPrHeadDotStatus(): USB Tx Message queue is null!\r\n");
            break;
        }

        /* Move to next group */
        currentDotIndex += dotsThisPacket;
    }
}

/******************************************************************************/
/*!   \fn void send( int size )

      \brief
   
      \author
          Chris King
*******************************************************************************/ 
void sendPrHeadCalResponse( PrHeadCalResponse* response )
{ 
    PRINTF("sendPrHeadCalResponse()\r\n");

    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initPrHeader( &frame[0], sizeof(PrHeadCalResponse) );
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrHeadCalResponse(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendPrHeadCalResponse(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrHeadCalResponse ) ]; 
    unsigned int index = 0;

    body[index++] = response->msgType;  
    body[index++] = response->calValue;
    
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendPrHeadCalResponse(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendPrHeadCalResponse(): USB Tx Message queue is null!\r\n");    
    }              
}

/******************************************************************************/
/*!   \fn void sendLowLabelMinMax( short minPeeling, short maxPeeling, 
                                   short minStreaming, short maxStreaming )

      \brief
   
      \author
          Chris King
*******************************************************************************/ 
void sendLowLabelMinMax(short minPeeling, short maxPeeling, short minStreaming, short maxStreaming)
{
  
    /* build header */
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    /* init header */
    initPrHeader( &frame[0], sizeof(LowLabelMinMaxMessage) );
    
    /* post header to queue */   
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendLowLabelMinMax(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendLowLabelMinMax(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    /* build body */
    unsigned char body[ sizeof( LowLabelMinMaxMessage ) ]; 
    unsigned int index = 0;
    
    body[index++] = PR_SET_LOW_LABEL_MIN_MAX;    
    body[index++] = getCharFromShort( (unsigned short *)&minPeeling, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&minPeeling, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&maxPeeling, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&maxPeeling, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&minStreaming, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&minStreaming, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&maxStreaming, 1 );
    body[index] = getCharFromShort( (unsigned short *)&maxStreaming, 0 );
    
    /* post body to queue */
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t result = xQueueSendFromISR(pQUSBSendPrHandle_, (void *)&body[0], 0);
        if( result != pdPASS ) {
            PRINTF("sendLowLabelMinMax(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendLowLabelMinMax(): USB Tx Message queue is null!\r\n");
    }
}

/******************************************************************************/
/*!   \fn void sendTakeLabelError( int size )

      \brief
        Sends label taken error message to the host. If label timer expires 
        without the label taken, then we inform the host to present the take label 
        error to the operator.
      \arg
        bool error: true for host to present error to operator. False for when 
        label is taken and the host to clear error.
      \author
          Aaron Swift
*******************************************************************************/                          
void sendTakeLabelError( bool error )
{
    PrTakeLabelErr msg;
    
    msg.msgType = PR_LABEL_TAKEN_ERROR;      
    msg.error = error;
  
    #ifdef LEGACY_PROTOCOL 
    
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    if( isQueueAlmostFull( pQUSBSendPrHandle_, MAX_USB_IN_LENGTH, 4 ) ) {
        PRINTF("\r\nskipped sendTakeLabelError\r\n");
        return;
    }
    
    initPrHeader( &frame[0], sizeof(PrTakeLabelErr) );
    
    /* post message to queue */    
    if( pQUSBSendPrHandle_ != NULL ) {
        BaseType_t xHigherPriorityTaskWoken = false;
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&frame[0], &xHigherPriorityTaskWoken );
        if( result != pdPASS ) {
            PRINTF("sendTakeLabelError(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendTakeLabelError(): pQUSBSendPrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( PrTakeLabelErr ) ]; 
    unsigned int index = 0;

    body[index++] = msg.msgType;
    body[index++] = msg.error;   
    
    PRINTF("sendTakeLabelError(): msg.error: %d\r\n", msg.error); 
    if( ( !isLabelTakenTimerStarted() ) && msg.error ) {
        PRINTF("sendTakeLabelError(): Opps!\r\n");
        setLabelTakenTimer();
    }
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {   
        BaseType_t xHigherPriorityTaskWoken = false;
        BaseType_t result = xQueueSendFromISR( pQUSBSendPrHandle_, (void *)&body[0], &xHigherPriorityTaskWoken );
        if( result != pdPASS ) {
            PRINTF("sendTakeLabelError(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendTakeLabelError(): USB Tx Message queue is null!\r\n");    
    } 
    
    #else
        
    unsigned char frame[ MAX_USB_FRAME_SIZE ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_PRINTER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = PR_LABEL_TAKEN_ERROR;
    usbMsg.hdr.msgSize          = sizeof(PrTakeLabelErr);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(PrTakeLabelErr); 
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);
       
    /* fill up the usb frame to the host */
    for( int i = 0; i < ( ( MAX_USB_FRAME_SIZE / 2 ) - sizeof( USBHeader ) ); i++ ) {
        frame[index++] = msg.msgType;        
        frame[index++] = msg.error;     
    }
        
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendTakeLabelError(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendTakeLabelError(): USB Tx Message queue is null!\r\n");    
    }                  
    
    #endif
}



/******************************************************************************/
/*!   \fn void sendAVRespByte( bool result )

      \brief
        handles converting avery response message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/                          
void sendAVRespByte( bool result )
{
    unsigned char frame[ MAX_USB_FRAME_SIZE ];
    frame[0] = result;
    /* post message to queue */
    if( pQUSBSendPrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendAVRespByte(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendAVRespByte(): USB Tx Message queue is null!\r\n");    
    }                          
}

/******************************************************************************/
/*!   \fn void sendAVData( unsigned char *pBfr, unsigned short length )

      \brief
        handles converting avery message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/                          
void sendAVData( unsigned char *pBfr, unsigned short length )
{
    unsigned char frame[ MAX_USB_FRAME_SIZE ];
    if( length <= MAX_AV_USB_FRAME_SIZE ) {
        memcpy( &frame[0], pBfr, length );
        /* post message to queue */
        if( pQUSBSendPrHandle_ != NULL ) {            
            BaseType_t result = xQueueSend( pQUSBSendPrHandle_, (void *)&frame[0], 0 );
            if( result != pdPASS ) {
                PRINTF("sendAVData(): USB tx message queue is full!\r\n");
            }     
        } else {
            PRINTF("sendAVData(): USB Tx Message queue is null!\r\n");    
        }                          
    }
}

/******************************************************************************/
/*!   \fn static void t_handlePrinterMessage( flexcan_frame_t *pFrame_ )

      \brief
        handles incomming printer messages.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqWakeup( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_WAKEUP;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqWakeup(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqWakeup(): PR Message Queue handle null! \r\n");  
    }
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqConfig( void )

      \brief
        handles request for printer configuration.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqConfig( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_CONFIG;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqConfig(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqConfig(): PR Message Queue handle null! \r\n");  
    }  
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqStatus( void )

      \brief
        handles request for printer status.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqStatus( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_STATUS;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqStatus(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqStatus(): PR Message Queue handle null! \r\n");  
    }  
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqSensors( void )

      \brief
        handles request for printer sensors.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqSensors( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_SENSORS;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqSensors(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqSensors(): PR Message Queue handle null! \r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqCalibration( void )

      \brief
        handles request for printer calibration.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqCalibration( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_CALIBRATE;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqCalibration(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqCalibration(): PR Message Queue handle null! \r\n");  
    }    
  
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqVersion( void )

      \brief
        handles request for label transfer.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqVersion( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_VERSION;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqVersion(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqVersion(): PR Message Queue handle null! \r\n");  
    }    
  
}

/******************************************************************************/
/*!   \fn static void t_handlePrHeadPower( unsigned char *pFrame )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrHeadPower( unsigned char *pFrame )
{
    PrPower prMsg; 
    prMsg.msgType = PR_POWER;
    prMsg.state = (bool)*( pFrame + 1 );
    
    if( pPQHandle_ ) {      
        /* post message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrHeadPower(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrHeadPower(): PR Message Queue handle null! \r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqHeadType( void )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqHeadType( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_HEAD_TYPE;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqHeadType(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqHeadType(): PR Message Queue handle null! \r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handlePrMode( unsigned char mode )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrMode( unsigned char mode )
{
    PrMode modeMsg;
    modeMsg.msgType = PR_MODE;
    modeMsg.mode = (PrintMode)mode;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&modeMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrMode(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrMode(): PR Message Queue handle null! \r\n");  
    }    
    
}

/******************************************************************************/
/*!   \fn static void t_handlePrDisable( void )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrDisable( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_DISABLE;

    if( pPQHandle_ ) {
        
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrDisable(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrDisable(): PR Message Queue handle null! \r\n");  
    }      
}

/******************************************************************************/
/*!   \fn static void t_handlePrEnable( void )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrEnable( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_ENABLE;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrEnable(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrEnable(): PR Message Queue handle null! \r\n");  
    }      
}

/******************************************************************************/
/*!   \fn static void t_handlePrReset( void )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReset( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_RESET;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReset(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReset(): PR Message Queue handle null! \r\n");  
    }       
}

/******************************************************************************/
/*!   \fn static void t_handlePrWriteConfig( unsigned char *pFrame )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrWriteConfig( unsigned char *pFrame )
{
    PrConfig prConfig;       

    prConfig.msgType                            = PR_CONFIG;
    
    /* skip the message type byte */
    pFrame++;

    prConfig.disposition                        = nCharToShort( pFrame );
    pFrame += sizeof(unsigned long);
    
    prConfig.config.instance                    = (StationID)*pFrame;
    pFrame += sizeof(unsigned long);
    
    prConfig.config.label_width                 = (LabelWidth)*pFrame;
    pFrame += sizeof(unsigned long);
     
    //prConfig.config.media_sensor_adjustment     = *pFrame++;
    //prConfig.config.out_of_media_count          = *pFrame++;
    
    //skip shoot cal values
    pFrame++;
    pFrame++;
    
    prConfig.config.contrast_adjustment         = *pFrame++;
        
    prConfig.config.expel_position              = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    prConfig.config.peel_position               = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    prConfig.config.retract_position            = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    prConfig.config.media_sensor_type           = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    prConfig.config.printheadResistance         = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    prConfig.config.verticalPosition            = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    //prConfig.config.backingPaper                = *pFrame++; 
    //prConfig.config.backingAndlabel             = *pFrame++;     
    //prConfig.config.labelCalCnts                = *pFrame++;
    //prConfig.config.noLabelCalCnts              = *pFrame++;
    
    
    //skip shoot cal values
    pFrame++;
    pFrame++;
    pFrame++;
    pFrame++;
    
    prConfig.config.cutterEnabled               = *pFrame;
    
    /* add message to the printer queue */
    BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prConfig, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handlePrWriteConfig(): PR Message Queue full! \r\n");  
    }                        
}

/******************************************************************************/
/*!   \fn static void t_handlePrEnableTakeup( void )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrEnableTakeup( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_ENABLE_TAKEUP;

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrEnableTakeup(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrEnableTakeup(): PR Message Queue handle null! \r\n");  
   }         
}


/******************************************************************************/
/*!   \fn static void t_handlePrPCBA_ID( unsigned char *pFrame )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrPCBA_ID( unsigned char *pFrame )
{  
    PrPCBA_Ver  prMsg;
    prMsg.msgType = PR_PCBA_REVISION;
    
    /* skip the message type byte */
    pFrame++;

    prMsg.pcbaRev = *pFrame++;
   
    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrPCBA_ID: PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrPCBA_ID: PR Message Queue handle null! \r\n");  
    }       
}


/******************************************************************************/
/*!   \fn static void t_handlePrMask( unsigned char *pFrame )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrMask( unsigned char *pFrame )
{  
    PrMask prMsg;
    prMsg.msgType = PR_MASK;
    
    /* skip the message type byte */
    pFrame++;

    prMsg.mask.sensor = *pFrame++;
    prMsg.mask.sensor2 = *pFrame++;
    prMsg.mask.user = *pFrame++;
    
    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrMask(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrMask(): PR Message Queue handle null! \r\n");  
    }       
}

/******************************************************************************/
/*!   \fn static void t_handlePrNomSize( unsigned char *pFrame  )

      \brief
        handles nominla size of the sized label.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrNomSize( unsigned char *pFrame )
{
    PrSize prMsg;
    prMsg.msgType = PR_SIZE;
    
    /* skip the message type byte */
    pFrame++;

    prMsg.actual_size = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);    
    prMsg.measured_size = nCharToShort( pFrame );
    
    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrNomSize(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrNomSize(): PR Message Queue handle null! \r\n");  
    }       
      
}

/******************************************************************************/
/*!   \fn static void t_handlePrTest( unsigned char *pFrame )

      \brief
        handles test message from the host.  
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrTest( unsigned char *pFrame )
{
    PrTest  prMsg;
    prMsg.msgType = PR_TEST;
    
    /* skip the message type byte */
    pFrame++;
    
    prMsg.data = *pFrame++;
    prMsg.data_channel = *pFrame;
    
    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrTest(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrTest(): PR Message Queue handle null! \r\n");  
    }          
}

/******************************************************************************/
/*!   \fn static void t_handlePrRam( unsigned char *pFrame  )

      \brief
        handles 
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrRam( unsigned char *pFrame  )
{ 
    PrRam ram;
    ram.msgType = PR_RAM;
    
    /* skip the message type byte */
    pFrame++;
    
    ram.ram[0].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[0].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[1].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[1].value  = nCharToShort( pFrame );        
    pFrame += sizeof(unsigned short);
    
    ram.ram[2].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[2].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[3].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[3].value = nCharToShort( pFrame );             
    pFrame += sizeof(unsigned short);
    
    ram.ram[4].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[4].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[5].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[5].value = nCharToShort( pFrame );                                 
    pFrame += sizeof(unsigned short);
    
    ram.ram[6].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[6].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[7].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[7].value = nCharToShort( pFrame );                                             
    pFrame += sizeof(unsigned short);
    
    ram.ram[8].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[8].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[9].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[9].value = nCharToShort( pFrame );                                             
    pFrame += sizeof(unsigned short);
    
    ram.ram[10].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[10].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[11].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[11].value = nCharToShort( pFrame );                                            
    pFrame += sizeof(unsigned short);
    
    ram.ram[12].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[12].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[13].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[13].value = nCharToShort( pFrame );                                            
    pFrame += sizeof(unsigned short);
    
    ram.ram[14].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[14].value  = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    ram.ram[15].location = nCharToShort( pFrame ); 
    pFrame += sizeof(unsigned short);
    
    ram.ram[15].value = nCharToShort( pFrame );                                             
    
    /* post message to the printer queue */
    BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&ram, 100 );
    if( result != pdTRUE ) {
        PRINTF("t_handlePrRam(): PR Message Queue full! \r\n");  
    }                    
}


/******************************************************************************/
/*!   \fn static void t_handlePrCommand( unsigned char *pFrame )

      \brief
        handles printer command message
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrCommand( unsigned char *pFrame )
{
    PrCommand prMsg;
    prMsg.msgType       = PR_COMMAND;
    
    /* skip the message type byte */
    pFrame++;
    /* this field is sent as a long */    
    prMsg.identifier    = *pFrame;
    pFrame +=4;
    /* this field is sent as a long */    
    prMsg.options       = *pFrame;
    pFrame +=4;
    /* this field is sent as a long */    
    prMsg.data_item     = nCharToShort( pFrame );   
    pFrame +=4;
    
    prMsg.value         = nCharToShort( pFrame );

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrCommand(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrCommand(): PR Message Queue handle null! \r\n");  
    }       
      
    
}


/******************************************************************************/
/*!   \fn static void t_handlePrCutterCut( void )

      \brief
        handles request to fire cutter mechanism.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrCutterCut( void )
{
    PrGeneric msg;
    msg.msgType = PR_CUTTER_CUT;
    
    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrCutterCut(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrCutterCut(): PR Message Queue handle null! \r\n");  
   }           
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqCutterStatus( void )

      \brief
        handles request for cutter status.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqCutterStatus( void )
{
    PrGeneric msg;
    msg.msgType = PR_REQ_CUTTER_STATUS;
    if( getGapCalStatus() == false )
    {
        if( pPQHandle_ ) 
        {
            /* add message to the printer queue */
            BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
            if( result != pdTRUE ) 
            {
                PRINTF("t_handlePrReqCutterStatus(): PR Message Queue full! \r\n");  
            }             
        } 
        else 
        {
            PRINTF("t_handlePrReqCutterStatus(): PR Message Queue handle null! \r\n");  
        }    
    }           
}


/******************************************************************************/
/*!   \fn static void t_handlePrFactoryDefaults( void )

      \brief
        handles request for printer to reset configuration to factory defaults.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrFactoryDefaults( void )
{
    PrGeneric msg;
    msg.msgType = PR_FACTORY_DFLTS;

    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrFactoryDefaults(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrFactoryDefaults(): PR Message Queue handle null! \r\n");  
    }             
}

/******************************************************************************/
/*!   \fn static void t_handlePrTeach( unsigned char *pFrame )

      \brief
        handles teach table message from the host.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrTeach( unsigned char *pFrame )
{
    PrTeach teach;               
    teach.msgType = PR_TEACH;
    
    /* skip the message type byte and pad*/
    pFrame +=2;

    teach.identifier = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    teach.entries = nCharToShort( pFrame );
    pFrame += sizeof(unsigned short);
    
    for( int i = 0; i < teach.entries; i++ ) {
        teach.operation[i].generic.directive = *pFrame++;
        teach.operation[i].generic.d0 = *pFrame++;          
        teach.operation[i].generic.d1 = *pFrame++;
        teach.operation[i].generic.d2 = *pFrame++;
        teach.operation[i].generic.d3 = *pFrame++;
        teach.operation[i].generic.d4 = *pFrame++;
        teach.operation[i].generic.d5 = *pFrame++;            
        teach.operation[i].generic.d6 = *pFrame++;            
    }
    
    /* add message to the printer queue */
    BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&teach, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handlePrTeach(): PR Message Queue full! \r\n");  
    }                    
}

/******************************************************************************/
/*!   \fn static void t_handleStationId( unsigned char order )

      \brief
        handles changing the printer order id from the host.
        Ex: primary printer --> secondary printer 
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handleStationId( unsigned char order )
{
    PrStationID msg;  
    msg.msgType = PR_STATION_ID;
    msg.station = (StationID)order;
    
    if( pPQHandle_ ) {
        /* post message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrOrderId(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrOrderId(): PR Message Queue handle null! \r\n");  
    }
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqDotWear( void )

      \brief
        handles request for print head dot wear.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqDotWear( void )
{  
    PrGeneric msg;
    msg.msgType = PR_REQ_DOT_WEAR;
    
    if( pPQHandle_ ) {
        /* post message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqDotWear(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqDotWear(): PR Message Queue handle null! \r\n");  
    }
}


/******************************************************************************/
/*!   \fn static void t_handlePrStartGapCal( void )

      \brief
        handles message for starting calibration of shoot through sensor.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrStartGapCal( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_START_GAP_CALIBRATION;
    
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrStartGapCal(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrStartGapCal(): PR Message Queue handle null! \r\n");  
    }                            
}

/******************************************************************************/
/*!   \fn static void t_handlePrGapCalNext( void )

      \brief
        handles message for processing next step in the calibration of
        shoot through sensor.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrGapCalNext( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_CAL_GAP_SENSOR_NEXT;
    
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrGapCalNext(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrGapCalNext(): PR Message Queue handle null! \r\n");  
    }                            
}

/******************************************************************************/
/*!   \fn static void t_handlePrStartTUCal( void )

      \brief
        handles message for starting calibration of takeup clutch sensor.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrStartTUCal( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_START_TU_CALIBRATION;
    
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrStartTUCal(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrStartTUCal(): PR Message Queue handle null! \r\n");  
    }                            
}

/******************************************************************************/
/*!   \fn static void t_handlePrTUCalNext( void )

      \brief
        handles message for processing next step in the calibration of
        takeup clutch sensor.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrTUCalNext( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_CAL_TU_SENSOR_NEXT;
    
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrTUCalNext(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrTUCalNext(): PR Message Queue handle null! \r\n");  
    }                            
}

/******************************************************************************/
/*!   \fn static void t_handlePrUseContinuousStock( void )

      \brief
        handles message from host notifying the printer that continuous stock 
        is loaded.       
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrUseContinuousStock( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_USE_CONTINUOUS_STOCK;
    
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrUseContinuousStock(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrUseContinuousStock(): PR Message Queue handle null! \r\n");  
    }                               
}

/******************************************************************************/
/*!   \fn static void t_handlePrCutterDefault( void )

      \brief
        handles message for clearing the cutter installed bit in serial flash.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrCutterDefault( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_ERASE_CUTTER_BIT;
    
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrCutterDefault(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrCutterDefault(): PR Message Queue handle null! \r\n");  
    }                        
}

/******************************************************************************/
/*!   \fn static void t_handlePrLabelSize( unsigned char *pFrame )

      \brief
        handles setting the label size to be printed.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrLabelSize( unsigned char *pFrame )
{
    PrLabelSize msg;
    msg.msgType = PR_LABEL_SIZE;
    
    msg.size = charToLong( pFrame );
    if( pPQHandle_ ) {
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrLabelSize(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrLabelSize(): PR Message Queue handle null! \r\n");  
    }                   
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqLabelTransfer( unsigned char *pFrame )

      \brief
        handles request for label transfer.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqLabelTransfer( unsigned char *pFrame  )
{
    PrReqTransfer  prMsg;
    prMsg.msgType = PR_REQ_TRANSFER;
    
    /* skip the message type byte */
    pFrame++;
    prMsg.transferSize = nCharToShort( pFrame );
    pFrame += 2;
    prMsg.imageSize = nCharToLong( pFrame ); 

    if( pPQHandle_ ) {
      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqVersion(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqVersion(): PR Message Queue handle null! \r\n");  
    }     
}

/******************************************************************************/
/*!   \fn static void t_handleWgReqSysInfo( void )

      \brief
        handles request for initial weigher system info (before craeting weigher
        manager in the backend).
         
      \author
          Joseph DiCarlantonio
*******************************************************************************/  
static void t_handleWgReqSysInfo( void )
{
    WgGeneric  wgMsg;
    wgMsg.msgType = WG_REQ_SYS_INFO;
        
    if( pWQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleWgReqSysInfo(): WQ Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handleWgReqSysInfo(): WQ Message Queue handle null! \r\n");  
    } 
}

/******************************************************************************/
/*!   \fn static void t_handleReqSysInfo( void )

      \brief
        handles request for system info.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handleReqSysInfo( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_SYS_INFO;
        
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqVersion(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqVersion(): PR Message Queue handle null! \r\n");  
    }         
}

/******************************************************************************/
/*!   \fn static void t_handlePrSetTime( unsigned char *pFrame )

      \brief
        handles set time message.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrSetTime( unsigned char *pFrame )
{
    PrSetTime prMsg; 
    prMsg.msgType = PR_SET_TIMER;

    /* skip the message type byte*/
    pFrame++;

    prMsg.timer = (SysTimerType)*pFrame;
    /* typedef are sent as long */
    pFrame += 4;
    prMsg.milliseconds = nCharToLong( pFrame );
    
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrSetTime(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrSetTime(): PR Message Queue handle null! \r\n");  
    }           
}

/******************************************************************************/
/*!   \fn static void t_handleReqSysInfo( void )

      \brief
        handles stop time message.
         
      \author
          Aaron Swift
*******************************************************************************/
static void t_handlePrStopTime( unsigned char *pFrame )
{
    PrStopTime  prMsg; 
    prMsg.msgType = PR_STOP_TIME;

    /* skip the message type byte*/
    pFrame++; 
    prMsg.timer = (SysTimerType)*pFrame;
    
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrSetTime(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrSetTime(): PR Message Queue handle null! \r\n");  
    }           
}

static void t_handlePrSetHTGapSize( unsigned char *pFrame )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_SET_HT_GAP_SIZE;
    
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrSetTime(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrSetTime(): PR Message Queue handle null! \r\n");  
    }           
}

/******************************************************************************/
/*!   \fn static void t_handlePrGblReqCutterStatus( void )

      \brief
        handles request for cutter status.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrGblReqCutterStatus( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_GBL_CUTTER_STATUS;
        
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrGblReqCutterStatus(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrGblReqCutterStatus(): PR Message Queue handle null! \r\n");  
    }           
}

/******************************************************************************/
/*!   \fn static void t_handlePrReqCutterVersion( void )

      \brief
        handles request for cutter version.
         
      \author
          Aaron Swift
*******************************************************************************/          
static void t_handlePrReqCutterVersion( void )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_REQ_CUTTER_VERSION;
        
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrReqCutterVersion(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrReqCutterVersion(): PR Message Queue handle null! \r\n");  
    }           
}

static void t_handlePrSetGTGapSize( unsigned char *pFrame )
{
    PrGeneric  prMsg;
    prMsg.msgType = PR_SET_GT_GAP_SIZE;
    
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrSetTime(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrSetTime(): PR Message Queue handle null! \r\n");  
    }           
}

static void t_handlePrSetLowLabelMinMax( unsigned char *pFrame )
{
    LowLabelMinMaxMessage prMsg;
    prMsg.msgType = PR_SET_LOW_LABEL_MIN_MAX;
    
    //PRINTF("t_handlePrSetLowLabelMinMax\r\n");

    pFrame++;
  
    prMsg.minValuePeeling = nCharToShort( pFrame );
    //PRINTF("minValuePeeling %d\r\n", prMsg.minValuePeeling);
    
    pFrame++;
    pFrame++;

    prMsg.maxValuePeeling = nCharToShort( pFrame );
    //PRINTF("maxValuePeeling %d\r\n", prMsg.maxValuePeeling);
    
    pFrame++;
    pFrame++;

    prMsg.minValueStreaming = nCharToShort( pFrame );
    //PRINTF("minValueStreaming %d\r\n", prMsg.minValueStreaming);
    
    pFrame++;
    pFrame++;

    prMsg.maxValueStreaming = nCharToShort( pFrame );
    //PRINTF("maxValueStreaming %d\r\n", prMsg.maxValueStreaming);
    
    if( pPQHandle_ ) {      
        /* add message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&prMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrSetTime(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrSetTime(): PR Message Queue handle null! \r\n");  
    }           
}

static void t_handlePrCalPrintheadResistance( unsigned char *pFrame )
{  
    PRINTF("t_handlePrCalPrintheadResistance\r\n");
  
    PrGeneric msg;
    msg.msgType = PR_CAL_PRINTHEAD_RESISTANCE;
    
    if( pPQHandle_ ) {
        /* post message to the printer queue */
        BaseType_t result = xQueueSendToBack( pPQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handlePrOrderId(): PR Message Queue full! \r\n");  
        }             
    } else {
        PRINTF("t_handlePrOrderId(): PR Message Queue handle null! \r\n");  
    }
}


/**************************WEIGHER*********************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*!   \fn void sendWgWakeup( WgWakeup *pWakeMsg )

      \brief
        handles converting weigher wakeup message into sub frames and adding
        to sub transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgWakeup( WgWakeup *pWakeMsg )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], ( sizeof(WgWakeup) + 1 ) );
       
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgWakeup(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgWakeup ) ]; 
    unsigned int index = 0;

    /* initialize body of the usb message */
    body[index++] = WG_WAKEUP;
    body[index++] = pWakeMsg->msgType;
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 0 );    

    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->weigherType, 1 );    
    body[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->weigherType, 0 );    

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgWakeup(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendWgWakeup(): USB Tx Message queue is null!\r\n");    
    }              
    
    #else

    unsigned char frame[ ( sizeof(WgWakeup) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgWakeup) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_WAKEUP;
    usbMsg.hdr.msgSize          = sizeof(WgWakeup);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgWakeup);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->pid, 0 );    

    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->weigherType, 1 );    
    frame[index++] = getCharFromShort( (unsigned short *)&pWakeMsg->weigherType, 0 );    

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgWakeup(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgWakeup(): USB Tx Message queue is null!\r\n");    
    }    
    #endif
}



/******************************************************************************/
/*!   \fn void sendWgStatus( WgStatus *pStatusMsg )

      \brief
        handles converting weigher status message into usb frames and adding
        to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgStatus( WgStatus *pStatusMsg )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    if(isQueueAlmostFull(pQUSBSendWrHandle_, MAX_USB_IN_LENGTH, 4))
    {
        PRINTF("\r\nskipped sendWgStatus\r\n");
        return;
    }
    
    initWrHeader( &frame[0], sizeof(WgStatus) );       
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgStatus(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgStatus ) ]; 
    unsigned int index = 0;

    /* initialize body of the usb message */
    body[index++] = pStatusMsg->msgType = WG_STATUS;
    body[index++] = getCharFromShort( &pStatusMsg->status, 1 );
    body[index++] = getCharFromShort( &pStatusMsg->status, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 0 );  
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->rawCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->rawCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->rawCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->rawCounts, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltX, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltX, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltY, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltY, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgStatus(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgStatus(): USB Tx Message queue is null!\r\n");    
    }    

    #else
    
    unsigned char frame[ ( sizeof(WgStatus) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgStatus) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_STATUS;
    usbMsg.hdr.msgSize          = sizeof(WgStatus);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgStatus);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( &pStatusMsg->status, 1 );
    frame[index++] = getCharFromShort( &pStatusMsg->status, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->avgFilterCounts, 0 );    
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->nonZeroCalibratedCounts, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->zeroedCalibratedCounts, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgXCounts, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgYCounts, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pStatusMsg->acclAvgZCounts, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltX, 1 );
    frame[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltX, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltY, 1 );
    frame[index++] = getCharFromShort( (unsigned short *)&pStatusMsg->tiltY, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgStatus(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgStatus(): USB Tx Message queue is null!\r\n");    
    }    
    #endif
}

/******************************************************************************/
/*!   \fn void sendWgConfig( WgCfg *pConfigMsg )

      \brief
        handles converting weigher config message into usb frames and adding
        to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgConfig( WgCfg *pConfigMsg )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgCfg) );              
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgConfig(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgConfig(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgCfg ) ]; 
    unsigned int index = 0;

    body[index++] = pConfigMsg->msgType; 
    body[index++] = getCharFromShort( &pConfigMsg->disposition, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 0 );    
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.prepack_motion_count, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.prepack_motion_count, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.initialize_zero_time, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.initialize_zero_time, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.small_motion_limit, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.small_motion_limit, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.large_motion_limit, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.large_motion_limit, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.large_motion_count, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.large_motion_count, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.small_motion_count, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.small_motion_count, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.no_motion_count, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.no_motion_count, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.number_of_calibrations, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.number_of_calibrations, 0 );
    body[index++] = getCharFromShort( &pConfigMsg->config.number_of_configurations, 1 );
    body[index++] = getCharFromShort( &pConfigMsg->config.number_of_configurations, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 0 );
    body[index++] = pConfigMsg->config.filter_speed;
    body[index++] = pConfigMsg->config.weigher_type;
    body[index++] = pConfigMsg->config.flags;       
    body[index++] = pConfigMsg->config.weigher_model;
    body[index++] = pConfigMsg->config.value_max_on_off;

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgConfig(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgConfig(): USB Tx Message queue is null!\r\n");    
    }        

    #else
    
    unsigned char frame[ ( sizeof(WgCfg) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgCfg) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_CONFIG;
    usbMsg.hdr.msgSize          = sizeof(WgCfg);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgCfg);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );    
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( &pConfigMsg->disposition, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->disposition, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.center_of_maintenance_zone, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.prepack_motion_count, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.prepack_motion_count, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.scale_factor, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.max_weight, 0 );    
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.gain_factor, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.initialize_zero_time, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.initialize_zero_time, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.small_motion_limit, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.small_motion_limit, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.large_motion_limit, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.large_motion_limit, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.large_motion_count, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.large_motion_count, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.small_motion_count, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.small_motion_count, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.no_motion_count, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.no_motion_count, 0 );
    frame[index++] = pConfigMsg->config.filter_speed;
    frame[index++] = pConfigMsg->config.weigher_type;
    frame[index++] = pConfigMsg->config.flags;
    frame[index++] = pConfigMsg->config.weigher_model;
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.min_weight_to_print, 0 );
    frame[index++] = pConfigMsg->config.value_max_on_off;
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_calibration_date, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.number_of_calibrations, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.number_of_calibrations, 0 );
    /azsm no longer used in message. frame[index++] = pConfigMsg->config.azsm_motion_limit;
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&pConfigMsg->config.last_configuration_date, 0 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.number_of_configurations, 1 );
    frame[index++] = getCharFromShort( &pConfigMsg->config.number_of_configurations, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgConfig(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgConfig(): USB Tx Message queue is null!\r\n");    
    }        
    #endif
}



/******************************************************************************/
/*!   \fn void sendWgVersion( WgVersion *pVersion )

      \brief
        handles converting weigher version information into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgVersion( WgVersion *pVersion )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgVersion) );              
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVersion(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgVersion ) ]; 
    unsigned int index = 0;

    /* initialize body of the usb message 
    pVersion->msgType = */
    body[index++] = getCharFromShort( &pVersion->pid, 0 ); 
    body[index++] = getCharFromShort( &pVersion->pid, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->swMajor, 0 ); 
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->swMinor, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMajor, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMinor, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&pVersion->swBuild, 0 );
    body[index++] = 0; /* weigher type first weigher */
    body[index++] = getCharFromLong( &pVersion->firmware, 0 ); 
    body[index++] = getCharFromLong( &pVersion->firmware, 1 );
    body[index++] = getCharFromLong( &pVersion->firmware, 2 ); 
    body[index++] = getCharFromLong( &pVersion->firmware, 3 );
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWgVersion(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVersion(): USB Tx Message queue is null!\r\n");    
    }        
    
    #else
    
    unsigned char frame[ ( sizeof(WgVersion) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgVersion) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_REQ_VERSION;
    usbMsg.hdr.msgSize          = sizeof(WgVersion);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgVersion);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = getCharFromShort( &pVersion->pid, 0 ); 
    frame[index++] = getCharFromShort( &pVersion->pid, 1 );
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->swMajor, 0 ); 
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->swMinor, 1 );
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMajor, 0 );
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->hwMinor, 1 );
    frame[index++] = getCharFromShort( (unsigned short *)&pVersion->swBuild, 0 );
    frame[index++] = 0; /* weigher type first weigher */
    frame[index++] = getCharFromLong( &pVersion->firmware, 0 ); 
    frame[index++] = getCharFromLong( &pVersion->firmware, 1 );
    frame[index++] = getCharFromLong( &pVersion->firmware, 2 ); 
    frame[index++] = getCharFromLong( &pVersion->firmware, 3 );
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVersion(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVersion(): USB Tx Message queue is null!\r\n");    
    }        
    #endif    
    
}

/******************************************************************************/
/*!   \fn void sendWgSystemInfo( WgInfo *pInfo )

      \brief
        handles sending current weigher software and hardware versions and 
        states of the serial flash and eeprom. 

      \author
          Aaron Swift
*******************************************************************************/
void sendWgSystemInfo( WgInfo *pInfo )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgInfo) );              
       
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgSystemInfo(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgSystemInfo(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgInfo ) ]; 
    unsigned int index = 0;

    /* initialize body of the usb message */
    body[index++] = pInfo->msgType;
    body[index++] = pInfo->serialFlashValid;
    body[index++] = pInfo->eepromValid;
    body[index++] = pInfo->softwareVersion[0];
    body[index++] = pInfo->softwareVersion[1];
    body[index++] = pInfo->softwareVersion[2];
    body[index++] = pInfo->hardwareVersion[0];
    body[index++] = pInfo->hardwareVersion[1];
    body[index++] = 0;  /* weigher position ( first, second ) */
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgSystemInfo(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgSystemInfo(): USB Tx Message queue is null!\r\n");    
    }        
    
    #else
    
    unsigned char frame[ ( sizeof(WgInfo) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgInfo) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_REQ_VERSION;
    usbMsg.hdr.msgSize          = sizeof(WgInfo);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgInfo);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    /* initialize body of the usb message */
    frame[index++] = pInfo->serialFlashValid;
    frame[index++] = pInfo->eepromValid;
    frame[index++] = pInfo->softwareVersion[0];
    frame[index++] = pInfo->softwareVersion[1];
    frame[index++] = pInfo->softwareVersion[2];
    frame[index++] = pInfo->hardwareVersion[0];
    frame[index++] = pInfo->hardwareVersion[1];
    frame[index++] = 0;  /* weigher position ( first, second ) */
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgSystemInfo(): USB Tx Message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgSystemInfo(): USB Tx Message queue is null!\r\n");    
    }        
    #endif
}

/******************************************************************************/
/*!   \fn void sendCat3Statistics( WgCat3Statistics *pStatsMsg )

      \brief
        handles converting weigher cat3 statistics message into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendCat3Statistics( WgCat3Statistics *pStatsMsg )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgCat3Statistics) );              
       
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3Statistics(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3Statistics(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof( WgCat3Statistics ) ]; 
    unsigned int index = 0;

    body[index++] = WG_CAT3_AUDIT_STATISTICS;
    body[index++] = pStatsMsg->totalPages;
    body[index++] = pStatsMsg->totalRecordsPerPage;
    body[index++] = pStatsMsg->currentPage;
    body[index++] = getCharFromShort( &pStatsMsg->totalFreeRecords, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->totalFreeRecords, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->totalRecordedEvents, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->totalRecordedEvents, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->totalCalibrationRecords, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->totalCalibrationRecords, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->totalConfigurationRecords, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->totalConfigurationRecords, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->gainCoefficientCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->gainCoefficientCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->offsetCoefficientCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->offsetCoefficientCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->gainFactorCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->gainFactorCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->centerOfMainCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->centerOfMainCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->zeroReferenceCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->zeroReferenceCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->weigherModelCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->weigherModelCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->minWeightPrintCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->minWeightPrintCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->maxWeightCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->maxWeightCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->filterSpeedCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->filterSpeedCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->divisionSizeCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->divisionSizeCounter, 0 );
    body[index++] = getCharFromShort( &pStatsMsg->weigherModeCounter, 1 );
    body[index++] = getCharFromShort( &pStatsMsg->weigherModeCounter, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3Statistics(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3Statistics(): USB Tx Message queue is null!\r\n");    
    }                
    
    #else
    
    unsigned char frame[ ( sizeof(WgCat3Statistics) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgCat3Statistics) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_CAT3_AUDIT_STATISTICS;
    usbMsg.hdr.msgSize          = sizeof(WgCat3Statistics);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgCat3Statistics);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pStatsMsg->totalPages;
    frame[index++] = pStatsMsg->totalRecordsPerPage;
    frame[index++] = pStatsMsg->currentPage;
    frame[index++] = getCharFromShort( &pStatsMsg->totalFreeRecords, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalFreeRecords, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalRecordedEvents, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalRecordedEvents, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalCalibrationRecords, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalCalibrationRecords, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalConfigurationRecords, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->totalConfigurationRecords, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->gainCoefficientCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->gainCoefficientCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->offsetCoefficientCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->offsetCoefficientCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->gainFactorCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->gainFactorCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->centerOfMainCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->centerOfMainCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->zeroReferenceCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->zeroReferenceCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->weigherModelCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->weigherModelCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->minWeightPrintCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->minWeightPrintCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->maxWeightCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->maxWeightCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->filterSpeedCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->filterSpeedCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->divisionSizeCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->divisionSizeCounter, 0 );
    frame[index++] = getCharFromShort( &pStatsMsg->weigherModeCounter, 1 );
    frame[index++] = getCharFromShort( &pStatsMsg->weigherModeCounter, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3Statistics(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3Statistics(): USB Tx Message queue is null!\r\n");    
    }                
    #endif
}

/******************************************************************************/
/*!   \fn void sendCat3Records( WgCat3Records *pRecordsMsg )

      \brief
        handles converting weigher cat3 record message into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendCat3Records( WgCat3Records *pRecordsMsg )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgCat3Records) );              
       
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3Records(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3Records(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ MAX_USB_INT_FRAME_SIZE ];
    unsigned int index = 0;
    body[0] = WG_CAT3_READ_RECORDS;
    body[1] = pRecordsMsg->pageNumber;
    body[2] = pRecordsMsg->index;
    
    /* send 320 bytes of record data ( 5 transfers of 64 bytes + 3 extra bytes ) */
    // start iterator at 4th position in buffer
    int i = 3;
    for( int y = 0; y < 5; y++ ) { 
        while( i < MAX_USB_INT_FRAME_SIZE ) {
            body[i++] = pRecordsMsg->records[ index++ ];
        }
        // reset to start of buffer
        i = 0;
        
        /* post message to queue */
        if( pQUSBSendWrHandle_ != NULL ) {
            BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
            if( result != pdPASS ) {
                PRINTF("sendCat3Records(): USB tx message queue is full!\r\n");
            }  
        } else {
            PRINTF("sendCat3Records(): USB Tx Message queue is null!\r\n");    
        }                   
    }
    
    // send extra 3 bytes
    if( index < 320 ) {
        // we should only have 3 bytes left over
        if( 320 - index == 3) {
            unsigned char extraBody[3];
            for(int i = 0; index < 320; ++i) {
                extraBody[i] = pRecordsMsg->records[ index++ ];
            }
            
            if( pQUSBSendWrHandle_ != NULL ) {
                BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&extraBody[0], 0 );
                if( result != pdPASS ) {
                    PRINTF("sendCat3Records(): USB tx message queue is full!\r\n");
                }  
            } else {
                PRINTF("sendCat3Records(): USB Tx Message queue is null!\r\n");    
            } 
        } else {
            PRINTF("sendCat3Records(): Error sending cat3 records buffer\r\n");
        }
    }

#else
    
    unsigned char frame[ ( sizeof(WgCat3Records) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgCat3Records) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_CAT3_READ_RECORDS;
    usbMsg.hdr.msgSize          = sizeof(WgCat3Records);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgCat3Records);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pRecordsMsg->pageNumber;
    frame[index++] = pRecordsMsg->index;    

    for( int i = 0; i < 320; i++ ) {       
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i++ ];
        frame[index++] = pRecordsMsg->records[ i ];
    }        
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3Records(): USB tx message queue is full!\r\n");
        }  
    } else {
        PRINTF("sendCat3Records(): USB Tx Message queue is null!\r\n");    
    }                   
    #endif
}

/******************************************************************************/
/*!   \fn void sendCat3RecordStatus( WgCat3RecordStatus *pStatusMsg )

      \brief
        handles converting weigher cat3 record status message into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendCat3RecordStatus( WgCat3RecordStatus *pStatusMsg )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgCat3RecordStatus) );              
        
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3RecordStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3RecordStatus(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ sizeof(WgCat3RecordStatus) ];     
    body[1] = WG_CAT3_RECORD_STATUS;
    body[1] = pStatusMsg->saved;
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3RecordStatus(): USB tx message queue is full!\r\n");
        }  
    } else {
        PRINTF("sendCat3RecordStatus(): USB Tx Message queue is null!\r\n");    
    }                   
    
    #else
    
    unsigned char frame[ ( sizeof(WgCat3RecordStatus) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgCat3RecordStatus) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_CAT3_RECORD_STATUS;
    usbMsg.hdr.msgSize          = sizeof(WgCat3RecordStatus);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgCat3RecordStatus);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index] = pStatusMsg->saved;
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3RecordStatus(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3RecordStatus(): USB Tx Message queue is null!\r\n");    
    }                          
    #endif
}

/******************************************************************************/
/*!   \fn void sendCat3LogEraseComplete( void )

      \brief
        handles converting weigher cat3 erase complete message into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendCat3LogEraseComplete( void )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(unsigned char) );              
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3LogEraseComplete(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendCat3LogEraseComplete(): pQUSBSendWrHandle_ is null!\r\n");
    }
    
    unsigned char body[ MAX_USB_INT_FRAME_SIZE ]; 
    body[0] = WG_CAT3_LOG_ERASE_COMPLETE;
    body[1] = 1;
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3LogEraseComplete(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendCat3LogEraseComplete(): USB Tx Message queue is null!\r\n");    
    }                   

    #else
    
    unsigned char frame[ ( sizeof(unsigned char) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(unsigned char) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_CAT3_LOG_ERASE_COMPLETE;
    usbMsg.hdr.msgSize          = sizeof(unsigned char);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(unsigned char);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index] = 1;
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendCat3LogEraseComplete(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendCat3LogEraseComplete(): USB Tx Message queue is null!\r\n");    
    }                   
    #endif
}

/******************************************************************************/
/*!   \fn void sendWgVmAssembly( WgVMWrAssembly *pVmAssembly )

      \brief
        handles sending weigher value max assembly number into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgVmAssembly( WgVMWrAssembly *pVmAssembly )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgVMWrAssembly) );              
       
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmAssembly(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVmAssembly(): pQUSBSendWrHandle_ is null!\r\n");
    }

    unsigned char body[ sizeof(WgVMWrAssembly) ]; 
    unsigned int index = 0;
    body[index++] = WG_VM_ASSEMBLY;
    body[index++] = pVmAssembly->assemblyNumber[0];
    body[index++] = pVmAssembly->assemblyNumber[1];
    body[index++] = pVmAssembly->assemblyNumber[2];
    body[index++] = pVmAssembly->assemblyNumber[3];
    body[index++] = pVmAssembly->assemblyNumber[4];
    body[index++] = pVmAssembly->assemblyNumber[5];
    body[index++] = pVmAssembly->assemblyNumber[6];
    body[index++] = pVmAssembly->assemblyNumber[7];
    body[index++] = pVmAssembly->assemblyNumber[8];
    body[index++] = pVmAssembly->assemblyNumber[9];

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {    
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmAssembly(): USB tx message queue is full!\r\n");
        }   
    } else {
        PRINTF("sendCat3LogEraseComplete(): USB Tx Message queue is null!\r\n");    
    }                          
    
    #else
    
    unsigned char frame[ ( sizeof(WgVMWrAssembly) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgVMWrAssembly) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_VM_ASSEMBLY;
    usbMsg.hdr.msgSize          = sizeof(WgVMWrAssembly);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgVMWrAssembly);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pVmAssembly->assemblyNumber[0];
    frame[index++] = pVmAssembly->assemblyNumber[1];
    frame[index++] = pVmAssembly->assemblyNumber[2];
    frame[index++] = pVmAssembly->assemblyNumber[3];
    frame[index++] = pVmAssembly->assemblyNumber[4];
    frame[index++] = pVmAssembly->assemblyNumber[5];
    frame[index++] = pVmAssembly->assemblyNumber[6];
    frame[index++] = pVmAssembly->assemblyNumber[7];
    frame[index++] = pVmAssembly->assemblyNumber[8];
    frame[index++] = pVmAssembly->assemblyNumber[9];

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {    
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmAssembly(): USB tx message queue is full!\r\n");
        }   
    } else {
        PRINTF("sendCat3LogEraseComplete(): USB Tx Message queue is null!\r\n");    
    }                          
    #endif
}



/******************************************************************************/
/*!   \fn void sendWgVmAssembly( WgVMWrVersion *pVmVersion )

      \brief
        handles sending weigher value max hardware version into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgVmVersion( WgVMWrVersion *pVmVersion )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgVMWrVersion) );  
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVmVersion(): pQUSBSendWrHandle_ is null!\r\n");
    }

    unsigned char body[ sizeof(WgVMWrAssembly) ]; 
    unsigned int index = 0;
    body[index++] = WG_VM_VERSION;
    body[index++] = pVmVersion->version[0];
    body[index++] = pVmVersion->version[1];

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmVersion(): USB tx message queue is full!\r\n");
        }       
    } else {
        PRINTF("sendWgVmVersion(): USB Tx Message queue is null!\r\n");    
    }                          

    #else
    
    unsigned char frame[ ( sizeof(WgVMWrVersion) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgVMWrVersion) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_VM_VERSION;
    usbMsg.hdr.msgSize          = sizeof(WgVMWrVersion);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgVMWrVersion);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pVmVersion->version[0];
    frame[index++] = pVmVersion->version[1];

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmVersion(): USB tx message queue is full!\r\n");
        }       
    } else {
        PRINTF("sendWgVmVersion(): USB Tx Message queue is null!\r\n");    
    }                          
    #endif
}

/******************************************************************************/
/*!   \fn void sendWgVmSerial( WgVMWrSerial *pVmSerial )

      \brief
        handles sending weigher value max serial number into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgVmSerial( WgVMWrSerial *pVmSerial )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgVMWrSerial) );  
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmSerial(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVmSerial(): pQUSBSendWrHandle_ is null!\r\n");
    }

    unsigned char body[ sizeof(WgVMWrAssembly) ]; 
    unsigned int index = 0;
    body[index++] = WG_VM_SERIAL;
    body[index++] = pVmSerial->serial[0];
    body[index++] = pVmSerial->serial[1];
    body[index++] = pVmSerial->serial[2];
    body[index++] = pVmSerial->serial[3];
    body[index++] = pVmSerial->serial[4];
    body[index++] = pVmSerial->serial[5];
    body[index++] = pVmSerial->serial[6];
    body[index++] = pVmSerial->serial[7];
    body[index++] = pVmSerial->serial[8];
    body[index++] = pVmSerial->serial[9];
    body[index++] = pVmSerial->serial[10];
    body[index++] = pVmSerial->serial[11];
    body[index++] = pVmSerial->serial[12];
    body[index++] = pVmSerial->serial[13];
    body[index++] = pVmSerial->serial[14];
    body[index++] = pVmSerial->serial[15];

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmSerial(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendWgVmVersion(): USB Tx Message queue is null!\r\n");    
    }                                  

    #else
    
    unsigned char frame[ ( sizeof(WgVMWrSerial) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgVMWrSerial) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_VM_SERIAL;
    usbMsg.hdr.msgSize          = sizeof(WgVMWrSerial);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgVMWrSerial);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = pVmSerial->serial[0];
    frame[index++] = pVmSerial->serial[1];
    frame[index++] = pVmSerial->serial[2];
    frame[index++] = pVmSerial->serial[3];
    frame[index++] = pVmSerial->serial[4];
    frame[index++] = pVmSerial->serial[5];
    frame[index++] = pVmSerial->serial[6];
    frame[index++] = pVmSerial->serial[7];
    frame[index++] = pVmSerial->serial[8];
    frame[index++] = pVmSerial->serial[9];
    frame[index++] = pVmSerial->serial[10];
    frame[index++] = pVmSerial->serial[11];
    frame[index++] = pVmSerial->serial[12];
    frame[index++] = pVmSerial->serial[13];
    frame[index++] = pVmSerial->serial[14];
    frame[index++] = pVmSerial->serial[15];

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmSerial(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendWgVmVersion(): USB Tx Message queue is null!\r\n");    
    }                                  
    #endif
}

/******************************************************************************/
/*!   \fn void sendWgVmDate( WgVMWrDate *pVmDate )

      \brief
        handles sending weigher value max manufacturing date into can frames and 
        adding to flexcan transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgVmDate( WgVMWrDate *pVmDate )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(WgVMWrDate) );  
        
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmDate(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVmDate(): pQUSBSendWrHandle_ is null!\r\n");
    }

    unsigned char body[ sizeof(WgVMWrDate) ]; 
    unsigned int index = 0;
    body[index++] = WG_VM_DATE;
    body[index++] = getCharFromLong( &pVmDate->epoch, 3 );
    body[index++] = getCharFromLong( &pVmDate->epoch, 2 );
    body[index++] = getCharFromLong( &pVmDate->epoch, 1 );
    body[index++] = getCharFromLong( &pVmDate->epoch, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmDate(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendWgVmDate(): USB Tx Message queue is null!\r\n");    
    }

    #else
    
    unsigned char frame[ ( sizeof(WgVMWrDate) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(WgVMWrDate) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_VM_DATE;
    usbMsg.hdr.msgSize          = sizeof(WgVMWrDate);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(WgVMWrDate);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = getCharFromLong( &pVmDate->epoch, 3 );
    frame[index++] = getCharFromLong( &pVmDate->epoch, 2 );
    frame[index++] = getCharFromLong( &pVmDate->epoch, 1 );
    frame[index++] = getCharFromLong( &pVmDate->epoch, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {        
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVmDate(): USB tx message queue is full!\r\n");
        }     
    } else {
        PRINTF("sendWgVmDate(): USB Tx Message queue is null!\r\n");    
    }
    #endif
}

/******************************************************************************/
/*!   \fn void sendWgFactoryDlftsComplete( bool status )

      \brief
        handles sending restoring factory defaults complete message. 

      \author
          Aaron Swift
*******************************************************************************/
void sendWgFactoryDlftsComplete( bool status )
{  
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(bool) );  
    
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgFactoryDlftsComplete(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgFactoryDlftsComplete(): pQUSBSendWrHandle_ is null!\r\n");
    }

    unsigned char body[ 2 ]; 
    
    body[0] = WG_FACTORY_DEFAULTS_CMPLT;
    body[1] = status;
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgFactoryDlftsComplete(): USB tx message queue is full!\r\n");
        }          
    } else {
        PRINTF("sendWgFactoryDlftsComplete(): USB Tx Message queue is null!\r\n");    
    }       
    
    #else
    
    unsigned char frame[ ( sizeof(bool) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(bool) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_VM_SERIAL;
    usbMsg.hdr.msgSize          = sizeof(bool);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(bool);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = status;
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgFactoryDlftsComplete(): USB tx message queue is full!\r\n");
        }          
    } else {
        PRINTF("sendWgFactoryDlftsComplete(): USB Tx Message queue is null!\r\n");    
    }       
    #endif
}

/******************************************************************************/
/*!   \fn void sendWgVMFitValues( fit_flash_section *fitValues )

      \brief
        handles converting valueMax eep data into usb frames and adding
        to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendWgVMFitValues( fit_flash_section *fitValues )
{
    #ifdef LEGACY_PROTOCOL 
    unsigned char frame[ sizeof( USBHeader ) ]; 
    
    initWrHeader( &frame[0], sizeof(fit_flash_section) );  
       
    /* post message to queue */    
    if( pQUSBSendWrHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVMFitValues(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWgVMFitValues(): pQUSBSendWrHandle_ is null!\r\n");
    }

    unsigned char body[ MAX_USB_INT_FRAME_SIZE ]; 
    unsigned int index = 0;
    body[index++] = WG_VM_FIT_VALUES;
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 2 );
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 0 );
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 2 );
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 0 );      
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 2 );
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 0 );
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 0 );      
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVMFitValues(): USB tx message queue is full!\r\n");
        }          
    } else {
        PRINTF("sendWgVMFitValues(): USB Tx Message queue is null!\r\n");    
    }

    index = 0;
    
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 0 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 0 );      
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVMFitValues(): USB tx message queue is full!\r\n");
        }          
    } else {
        PRINTF("sendWgVMFitValues(): USB Tx Message queue is null!\r\n");    
    }
    
    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&body[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVMFitValues(): USB tx message queue is full!\r\n");
        }          
    } else {
        PRINTF("sendWgVMFitValues(): USB Tx Message queue is null!\r\n");    
    }       

    #else
    
    unsigned char frame[ ( sizeof(fit_flash_section) + sizeof( USBHeader ) ) ];
    memset( (void *)&frame[0], 0, ( sizeof(fit_flash_section) + sizeof( USBHeader ) ) );
    HUSBMessage usbMsg;
    /* initialize the message header */
    usbMsg.hdr.device           = G_WEIGHER_; 
    usbMsg.hdr.stationId        = 1; /* to do */
    usbMsg.hdr.msgType          = WG_VM_FIT_VALUES;
    usbMsg.hdr.msgSize          = sizeof(fit_flash_section);                
    usbMsg.hdr.frameNumber      = 1;      
    usbMsg.hdr.numberOfFrames   = 1;
    usbMsg.hdr.lastFrameSize    = sizeof(fit_flash_section);
        
    memcpy( &frame[0], &usbMsg.hdr, sizeof(USBHeader) );
    int index =  sizeof(USBHeader);

    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->m, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zerovalue, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->spanvalue, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->Z0, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->D, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x2, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x1, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->x0, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y2, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y1, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->y0, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[0], 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[1], 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xrot[2], 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[0], 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[1], 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->yrot[2], 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[0], 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[1], 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->zrot[2], 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->xgain, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->ygain, 0 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->g_coeff, 0 );      
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 3 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 2 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 1 );
    frame[index++] = getCharFromLong( (unsigned long *)&fitValues->checksum, 0 );

    /* post message to queue */
    if( pQUSBSendWrHandle_ != NULL ) {            
        BaseType_t result = xQueueSend( pQUSBSendWrHandle_, (void *)&frame[0], 0 );
        if( result != pdPASS ) {
            PRINTF("sendWgVMFitValues(): USB tx message queue is full!\r\n");
        }          
    } else {
        PRINTF("sendWgVMFitValues(): USB Tx Message queue is null!\r\n");    
    }
    #endif
}

/******************************************************************************/
/*!   \fn static void  t_handleReqStatus( void )

      \brief
        private handles weigher request status usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqStatus( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_STATUS;

    /* add message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqStatus(): WG Message Queue full! \r\n");  
    }
}

/******************************************************************************/
/*!   \fn static void t_handleReqWakeup( void )

      \brief
        private handles weigher request wakeup usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqWakeup( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_WAKEUP;

    /* add message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqWakeup(): WG Message Queue full! \r\n");  
    }  
}

/******************************************************************************/
/*!   \fn static void t_handleReqConfig( void )

      \brief
        private handles weigher request config usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqConfig( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_REQ_CONFIG;

    /* add message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqConfig(): WG Message Queue full! \r\n");  
    }           
}

/******************************************************************************/
/*!   \fn static void  t_handleModeMsg( unsigned char mode )

      \brief
        private handles weigher mode message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleModeMsg( unsigned char mode )
{
    
    //PRINTF("t_handleModeMsg(): Mode: %d \r\n", mode );
   
    WgMode wgMode;
    wgMode.msgType = WG_MODE;
    wgMode.mode = (WeighMode)mode;
    
    /* add message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMode, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleModeMsg(): WG Message Queue full! \r\n");  
    }
}

/******************************************************************************/
/*!   \fn static void t_handleWriteConfig( unsigned char *pFrame )

      \brief
        private handles weigher write config usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleWriteConfig( unsigned char *pFrame )
{  
    static WgCfg wgConfig;

    #if 0 // this config used to be > 64 bytes, hence the message tracker. after 
      // removing the azsm motion limit, this is now == 64 bytes
    if(msgTracker.msgSize == 0) {
        PRINTF("t_handleWriteConfig(): message size is zero\r\n");
    }    
    if(msgTracker.msgSize > 64) {
    #endif
               
        wgConfig.msgType                            = WG_CONFIG;
        pFrame++;
        
        wgConfig.disposition                        = nCharToShort( pFrame );    
        pFrame += sizeof(unsigned long);
     
        wgConfig.config.center_of_maintenance_zone  = nCharToLong( pFrame );
        pFrame += sizeof(unsigned long);
            
        wgConfig.config.scale_factor                = nCharToLong( pFrame );
        pFrame += sizeof(unsigned long);

        wgConfig.config.max_weight                  = nCharToLong( pFrame );                   
        pFrame += sizeof(unsigned long);

        wgConfig.config.gain_factor                 = nCharToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        wgConfig.config.prepack_motion_count        = nCharToLong( pFrame );
        pFrame += sizeof(unsigned short);

        wgConfig.config.initialize_zero_time        = nCharToShort( pFrame );
        pFrame += sizeof(unsigned short);
        
        wgConfig.config.small_motion_limit          = nCharToShort( pFrame );
        pFrame += sizeof(unsigned short);

        wgConfig.config.large_motion_limit          = nCharToShort( pFrame );
        pFrame += sizeof(unsigned short);

        wgConfig.config.large_motion_count          = nCharToShort( pFrame );
        pFrame += sizeof(unsigned short);

        wgConfig.config.small_motion_count          = nCharToShort( pFrame );
        pFrame += sizeof(unsigned short);

        wgConfig.config.no_motion_count             = nCharToShort( pFrame );
        pFrame += sizeof(unsigned short);

        wgConfig.config.filter_speed                = *pFrame;
        pFrame += sizeof(unsigned long);
        
        wgConfig.config.last_calibration_date       = nCharToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        wgConfig.config.number_of_calibrations      = nCharToShort( pFrame );      
        pFrame += sizeof(unsigned short);
        
        wgConfig.config.last_configuration_date     = nCharToLong( pFrame ); 
        pFrame += sizeof(unsigned long);
        
        wgConfig.config.number_of_configurations    = nCharToShort( pFrame ); 
        pFrame += sizeof(unsigned short);
        
        wgConfig.config.weigher_type                = nCharToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        wgConfig.config.flags                       = *pFrame++;

        wgConfig.config.min_weight_to_print         = nCharToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        wgConfig.config.weigher_model               = (WeigherModel)*pFrame; //TFink 5/15/24 model was a long. Made short because msg was 65 bytes 
		pFrame += sizeof(unsigned short);
		PRINTF("t_handleWriteConfig(): wgConfig.config.weigher_model = %d \r\n", wgConfig.config.weigher_model);  
        
        wgConfig.config.value_max_on_off            = (bool)*pFrame++;   
        
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgConfig, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleWriteConfig(): WG Message Queue full! \r\n");  
        }   
}

/******************************************************************************/
/*!   \fn static void t_handleReqWeight( void )

      \brief
        private handles weigher request weight usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqWeight( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_REQ_WEIGHT;

    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqWeight(): WG Message Queue full! \r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handleEnable( void )

      \brief
        private handles weigher enable usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleEnable( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_ENABLE;

    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleEnable(): WG Message Queue full! \r\n");  
    }      
}

/******************************************************************************/
/*!   \fn static void t_handleDisable( void )

      \brief
        private handles weigher disable usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleDisable( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_DISABLE;

    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleDisable(): WG Message Queue full! \r\n");  
    }        
}

/******************************************************************************/
/*!   \fn static void t_handleRezero( void )

      \brief
        private handles weigher rezero usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleRezero( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_REZERO;

    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleRezero(): WG Message Queue full! \r\n");  
    }        
}

/******************************************************************************/
/*!   \fn static void void t_handleReset( void )

      \brief
        private handles weigher reset usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReset( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_RESET;

    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReset(): WG Message Queue full! \r\n");  
    }          
}

/******************************************************************************/
/*!   \fn static void t_handleControl( unsigned char cntrl )

      \brief
        private handles weigher mode usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleControl( unsigned char cntrl )
{
    WgControl cntrlMsg;
    cntrlMsg.msgType = WG_CONTROL;
    cntrlMsg.control = cntrl;
    
    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&cntrlMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleControl(): WG Message Queue full! \r\n");  
    }          
  
}

/******************************************************************************/
/*!   \fn static void t_handleReqCat3Statistics( void )

      \brief
        private handles weigher request cat3 statistics usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqCat3Statistics( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_CAT3_REQ_STATISTICS;

    /* post message to the weigher queue */
    BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqCat3Statistics(): WG Message Queue full! \r\n");  
    }            
}

/******************************************************************************/
/*!   \fn static void t_handleCat3DateTime( unsigned char *pFrame )

      \brief
        private handles weigher cat3 date time usb frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleCat3DateTime( unsigned char *pFrame )
{
    WgCat3DateTime dateTime;
    
    dateTime.msgType = WG_CAT3_DATE_TIME;
    pFrame++;
    
    dateTime.epochTime = nCharToLong( pFrame );

    if( pWQHandle_ != NULL ) {     
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&dateTime, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleCat3DateTime(): WG Message Queue full! \r\n");  
        }
    } else {
        PRINTF("t_handleCat3DateTime(): Weigher message queue null!\r\n");  
    }
}

/******************************************************************************/
/*!   \fn static void t_handleReqCat3NextRecord( void )

      \brief
        private handles weigher request cat3 next record CAN frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqCat3NextRecord( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_CAT3_REQ_NEXT_RECORDS;

    if( pWQHandle_ != NULL ) {     
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleReqCat3NextRecord(): WG Message Queue full! \r\n");  
        }
    } else {
        PRINTF("t_handleReqCat3NextRecord(): Weigher message queue null!\r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handleReqCat3EraseLog( void )

      \brief
        private handles weigher request cat3 erase log CAN frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqCat3EraseLog( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_CAT3_REQ_ERASE_LOG;

    if( pWQHandle_ != NULL ) {     
    /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleReqCat3EraseLog(): WG Message Queue full! \r\n");  
        } 
    } else {
        PRINTF("t_handleReqCat3EraseLog(): Weigher message queue null!\r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handleReqCat3Page( unsigned char page, unsigned char index )

      \brief
        private handles weigher request cat3 page CAN frame.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqCat3Page( unsigned char page, unsigned char index )
{
    WgReqCat3Page  reqMsg;
    reqMsg.msgType = WG_CAT3_REQ_PAGE;
    reqMsg.pageNumber = page;
    reqMsg.index = index;
    
    if( pWQHandle_ != NULL ) {      
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&reqMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleReqCat3Page(): WG Message Queue full! \r\n");  
        }           
    } else {
        PRINTF("t_handleReqCat3Page(): Weigher message queue null!\r\n");  
    }
    
}

/******************************************************************************/
/*!   \fn static void t_handleCat3RecordWrite( unsigned char *pFrame  )

      \brief
        private handles weigher cat3 write record CAN frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleCat3RecordWrite( unsigned char *pFrame  )
{
    WgCat3WriteRecord record;
    
    record.msgType      = WG_CAT3_WRITE_RECORD;        
    record.event        = *pFrame++;
    record.parameterId  = *pFrame++;
    record.eventType    = *pFrame++;
        
    record.epochTime    = nCharToLong( pFrame );
    pFrame += sizeof(unsigned long); 
    
    record.oldValue     = nCharToLong( pFrame );
    pFrame += sizeof(unsigned long); 
    
    record.newValue     = nCharToLong( pFrame );
    pFrame += sizeof(unsigned long); 
    
    if( pWQHandle_ != NULL ) {      
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&record, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleCat3RecordWrite(): WG Message Queue full! \r\n");  
        }
    } else {
        PRINTF("t_handleCat3RecordWrite(): Weigher message queue null!\r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handleReqVersion( void )

      \brief
        private handles request for weigher version.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqVersion( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_REQ_VERSION;

    if( pWQHandle_ != NULL ) {    
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleReqVersion(): WG Message Queue full! \r\n");  
        }                 
    } else {
        PRINTF("t_handleReqVersion(): Weigher message queue null!\r\n");  
    }    
}



/******************************************************************************/
/*!   \fn static void t_handleValueMaxRequest( unsigned char *pFrame )

      \brief
        private handles weigher value max request message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleValueMaxRequest( unsigned char *pFrame )
{
    WgVMRequest req;
    req.msgType = WG_VM_REQUEST;
    req.req = (ValueMaxReq)*pFrame;

    if( pWQHandle_ != NULL ) {
        /* post message to the weigher message queue    */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&req, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleValueMaxRequest(): Weigher test Queue full! \r\n");  
        }                    
    } else {
        PRINTF("t_handleValueMaxRequest(): Weigher test message queue null!\r\n");  
    }
}

/******************************************************************************/
/*!   \fn static void t_handleValueMaxWriteAssembly( unsigned char *pFrame )

      \brief
        private handles weigher value max write assembly message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleValueMaxWriteAssembly( unsigned char *pFrame )
{
        WgVMWrAssembly msg;
        msg.msgType = WG_VM_WRITE_ASSEMBLY;
 
        for( int i = 0; i < VM_ASSEMBLY_MAX; i++ ) {
            msg.assemblyNumber[i] = *pFrame++;
        }
        if( pWQHandle_ != NULL ) {
             /* post message to the weigher message queue    */
            BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&msg, 0 );
            if( result != pdTRUE ) {
                PRINTF("t_handleValueMaxWriteAssembly(): WG Message Queue full! \r\n");  
            }                    
        } else {
            PRINTF("t_handleValueMaxWriteAssembly():  WG Message Queue null!\r\n");
        }
}

/******************************************************************************/
/*!   \fn static void t_handleValueMaxWriteVersion( unsigned char *pFrame )

      \brief
        private handles weigher value max write hw version message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleValueMaxWriteVersion( unsigned char *pFrame )
{
    WgVMWrVersion  msg;
    msg.msgType = WG_VM_WRITE_VERSION;
    msg.version[0] =  *pFrame++;
    msg.version[1] =  *pFrame;

    if( pWQHandle_ != NULL ) {
        /* post message to the weigher message queue    */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleValueMaxWriteVersion(): Weigher test Queue full! \r\n");  
        }                    
    } else {
        PRINTF("t_handleValueMaxWriteVersion(): Weigher test message queue null!\r\n");  
    }    
}

/******************************************************************************/
/*!   \fn static void t_handleValueMaxWriteSerial( unsigned char *pFrame )

      \brief
        private handles weigher value max write serial message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleValueMaxWriteSerial( unsigned char *pFrame )
{
    WgVMWrSerial msg;

    msg.msgType = WG_VM_WRITE_SERIAL;
    for( int i = 0; i < VM_SERIAL_MAX; i++ ) {
        msg.serial[i] = *pFrame++;
    }
    
    if( pWQHandle_ != NULL ) {    
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleValueMaxWriteSerial(): WG Message Queue full! \r\n");  
        }                    
    } else {
        PRINTF("t_handleValueMaxWriteSerial(): Weigher message queue null!\r\n");  
    }        

}

/******************************************************************************/
/*!   \fn static void t_handleValueMaxWriteDate( unsigned char *pFrame )

      \brief
        private handles weigher value max write date message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleValueMaxWriteDate( unsigned char *pFrame )
{
    WgVMWrDate msg;
    msg.msgType = WG_VM_WRITE_DATE;
    msg.epoch = charToLong( pFrame );

    if( pWQHandle_ != NULL ) {
        /* post message to the weigher message queue    */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&msg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleValueMaxWriteDate(): Weigher Queue full! \r\n");  
        }                    
    } else {
        PRINTF("t_handleValueMaxWriteDate(): Weigher message queue null!\r\n");  
    }        
}

/******************************************************************************/
/*!   \fn static void t_handleReqInfo( void )

      \brief
        private handles request for weigher info message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleReqInfo( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_REQ_SYSTEM_INFO;

    if( pWQHandle_ != NULL ) {
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleReqInfo(): WG Message Queue full! \r\n");  
        }     
    } else {
        PRINTF("t_handleReqInfo(): Weigher message queue null!\r\n");  
    }            
}

/******************************************************************************/
/*!   \fn static void t_handleResetDefaultCfg( void )

      \brief
        private handles request to reset weigher configuration to factory 
        defaults message.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleResetDefaultCfg( void )
{
    WgGeneric wgMsg;
    wgMsg.msgType = WG_FACTORY_DEFAULTS;

    if( pWQHandle_ != NULL ) {
        /* post message to the weigher queue */
        BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&wgMsg, 0 );
        if( result != pdTRUE ) {
            PRINTF("t_handleResetDefaultCfg(): WG Message Queue full! \r\n");  
        }                
    } else {
        PRINTF("t_handleReqInfo(): Weigher message queue null!\r\n");  
    }        
  
}

/******************************************************************************/
/*!   \fn static void t_handleWriteVMFit( unsigned char *pFrame )

      \brief
        private handles vm fit write usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void t_handleWriteVMFit( unsigned char *pFrame )
{   
        WgVMFitMsg fitMsg;   
        fitMsg.msgType = WG_VM_WRITE_FIT_VALUES;
        fitMsg.fit.g = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.m = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.zerovalue = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.spanvalue = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.Z0 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.D  = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.x2 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.x1 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.x0 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.y2 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.y1 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.y0 = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.xrot[0] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.xrot[1] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.xrot[2] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.yrot[0] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.yrot[1] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.yrot[2] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.zrot[0] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.zrot[1] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.zrot[2] = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.xgain = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.ygain = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.g_coeff = charToLong( pFrame );
        pFrame += sizeof(unsigned long);
        
        fitMsg.fit.checksum = charToLong( pFrame );  
        
        if( pWQHandle_ != NULL ) {
            /* post message to the weigher queue */
            BaseType_t result = xQueueSendToBack( pWQHandle_, (void *)&fitMsg, 0 );
            if( result != pdTRUE ) {
                PRINTF("t_handleWriteVMFit(): WG Message Queue full! \r\n");  
            }                    
        } else {
            PRINTF("t_handleWriteVMFit(): Weigher message queue null!\r\n");  
        }        
            
}

uint32_t formatSendWGMsg( BinaryHeader_t *pHeader, void *pData, uint16_t length )
{
    AsciiHeader ascHeader;
    uint8_t msgBfr[512] = {0};
    uint16_t checksum = 0;
    uint32_t msgSize = sizeof(AsciiHeader) + length;
    if( ( msgSize + BCC_LEN ) <= 512 ) {
      
        pHeader->messageSize = msgSize;


        /* convert our binary header to ascii */
        toAsciiHex( (unsigned char *)&ascHeader.messageSize, sizeof(pHeader->messageSize), pHeader->messageSize );
        toAsciiHex( (unsigned char *)&ascHeader.cvn, sizeof(pHeader->cvn), pHeader->cvn );
        toAsciiHex( (unsigned char *)&ascHeader.slaveID, sizeof(pHeader->slaveID), pHeader->slaveID );
        toAsciiHex( (unsigned char *)&ascHeader.cmd, sizeof(pHeader->cmd), pHeader->cmd );

        ascHeader.action = pHeader->action;
        ascHeader.type = pHeader->type;
        
        toAsciiHex( (unsigned char *)&ascHeader.status, sizeof(pHeader->status), pHeader->status );
        stoAsciiHex( (unsigned char *)&ascHeader.sequenceNum, sizeof(pHeader->sequenceNum ), pHeader->sequenceNum );
                
        /* add control character */
        msgBfr[0] = T_STX;
        
        /* copy our ascii header to our message buffer */
        memcpy( &msgBfr[1], (void *)&ascHeader, sizeof(AsciiHeader) );

        
        /* copy the message body to the message buffer */
        memcpy( &msgBfr[sizeof(AsciiHeader) + 1], pData, length );

        /* calculate and add crc to the message */
        checksum = crc16( &msgBfr[1], msgSize );
        
        msgSize += 1;
        /* convert checksum to ascii and append to message buffer */          
        msgSize += toAsciiHex( (unsigned char *)&msgBfr[msgSize], sizeof(uint16_t), checksum );
        
        /* append control character at the end of block */
        msgBfr[ msgSize ] = T_ETX;
        msgSize += 1;

#if 0   /* debug */      
        if( ( pHeader->cmd == 18 ) ||
            ( pHeader->cmd == 1 ) ) {  
#if 0              
            PRINTF("ascHeader.messageSize[0]: %2x\r\n", ascHeader.messageSize[0]);  
            PRINTF("ascHeader.messageSize[1]: %2x\r\n", ascHeader.messageSize[1]);  
            PRINTF("ascHeader.messageSize[2]: %2x\r\n", ascHeader.messageSize[2]);  
            PRINTF("ascHeader.messageSize[3]: %2x\r\n", ascHeader.messageSize[3]);  
              
            PRINTF("ascHeader.cvn[0]: %2x\r\n", ascHeader.cvn[0]);  
            PRINTF("ascHeader.cvn[1]: %2x\r\n", ascHeader.cvn[1]);  
            PRINTF("ascHeader.cvn[2]: %2x\r\n", ascHeader.cvn[2]);  
            PRINTF("ascHeader.cvn[3]: %2x\r\n", ascHeader.cvn[3]);  

            PRINTF("ascHeader.slaveID[0]: %2x\r\n", ascHeader.slaveID[0]);  
            PRINTF("ascHeader.slaveID[1]: %2x\r\n", ascHeader.slaveID[1]);  
            
            PRINTF("ascHeader.cmd[0]: %2x\r\n", ascHeader.cmd[0]);  
            PRINTF("ascHeader.cmd[1]: %2x\r\n", ascHeader.cmd[1]);  
            PRINTF("ascHeader.cmd[2]: %2x\r\n", ascHeader.cmd[2]);  
            PRINTF("ascHeader.cmd[3]: %2x\r\n", ascHeader.cmd[3]);  

            PRINTF("ascHeader.action: %2x\r\n", ascHeader.action);  
            PRINTF("ascHeader.type: %2x\r\n", ascHeader.type);  
          
            PRINTF("ascHeader.status[0]: %2x\r\n", ascHeader.status[0]);  
            PRINTF("ascHeader.status[1]: %2x\r\n", ascHeader.status[1]);              
            PRINTF("ascHeader.status[2]: %2x\r\n", ascHeader.status[2]);  
            PRINTF("ascHeader.status[3]: %2x\r\n", ascHeader.status[3]);  

            PRINTF("ascHeader.sequenceNum[0]: %2x\r\n", ascHeader.sequenceNum[0]);  
            PRINTF("ascHeader.sequenceNum[1]: %2x\r\n", ascHeader.sequenceNum[1]);  
            PRINTF("ascHeader.sequenceNum[2]: %2x\r\n", ascHeader.sequenceNum[2]);  
            PRINTF("ascHeader.sequenceNum[3]: %2x\r\n", ascHeader.sequenceNum[3]);  
#endif         
#if 0            
            PRINTF("crc: %d\r\n", checksum);
            PRINTF("message size: %d\r\n", msgSize);  
            PRINTF("before ascii and encryption:\r\n");
            for( int i = 0; i < msgSize; i++ ) {                
                PRINTF("%2x, ", msgBfr[i] );
            }
            PRINTF("\r\n");
#endif
        }
#endif
        
        /* encrypt message buffer before we send */
        for( int i = 0; i < msgSize;  i++ ) {
            encryptChar(  &msgBfr[ i ] ); 
        }
#if 0        
        if( pHeader->cmd == 18 ) {
             PRINTF("translator => to host:\r\n");
            for( int i = 0; i < msgSize; i++ ) {
                PRINTF("%2x, ", msgBfr[i] );
            }            
            PRINTF("\r\n");
            PRINTF("crc: %d\r\n", checksum);
        }       
#endif        
        sendAVWeigherMsg( &msgBfr[0], msgSize );
    }
    return msgSize;
}

void sendAVWeigherMsg(  unsigned char *pMsg, unsigned short length )
{
    static unsigned short remaining = 0;

    if( ( tMutex_ != NULL ) && ( pMsg != NULL ) ) {
        if( xSemaphoreTake( tMutex_, ( TickType_t )100 ) == pdTRUE ) {
            while( length != 0) {
                if( length > 64 )  {                  
                    remaining = USB_Weigher_SendData( pMsg, 64 );                               
                    length -= 64;
                    pMsg += 64;
                } else {                    
                    USB_Weigher_SendData( pMsg, 64 );                                         
                    length = 0;
                }
            }
        }
        xSemaphoreGive( tMutex_ );
    }
}

 unsigned char buildResponsePacket( Packet_t *pMsg, AVMSG_t cmd, 
                                    unsigned short blkNum, unsigned short numBlks,
                                    void *pData, unsigned char dataLen )
{

    pMsg->stx = AV_STX;
    pMsg->cmd = cmd;
    pMsg->blkNum = htons(blkNum);
    pMsg->numBlks = htons(numBlks);
    pMsg->pad = 0;

    if( blkNum > numBlks )
            return 0;
    
    pMsg->len = dataLen + sizeof(AVMSGHEADER_t);
    memcpy( pMsg->data, (unsigned char*)pData, dataLen );

    return pMsg->len;  
}

void sendAVPrinterMsg(  unsigned char *pMsg, unsigned short length )
{

    if( ( tMutex_ != NULL ) && ( pMsg != NULL ) ) {
        if( xSemaphoreTake( tMutex_, ( TickType_t )100 ) == pdTRUE ) {
            USB_Printer_SendData( pMsg, length );
        }
        xSemaphoreGive( tMutex_ );
    }
}

void encryptChar( unsigned char *pChar )
{
      uint8_t size = strlen( (const char *)&avKey[0] );
      for( int i = 0; i < size; i++ ) {
          *pChar ^= avKey[i] ^ ( avKey[0] * i );
      }
}


/******************************************************************************/
/*!   \fn unsigned short charToShort( unsigned char *pData  )

      \brief
        public creates unsigned word from char array
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned short charToShort( unsigned char *pData )
{
    unsigned short data;
    data =  *pData++;
    data <<= 8;
    data |= *pData;
    return data;
}

/******************************************************************************/
/*!   \fn unsigned short nCharToShort( unsigned char *pData )

      \brief
        public creates unsigned word from byte swapped char array
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned short nCharToShort( unsigned char *pData )
{
    unsigned short data;
    data =  *( pData + 1 );
    data <<= 8;
    data |= *pData;
    return data;    
}

/******************************************************************************/
/*!   \fn unsigned long charToLong( unsigned char *pData )

      \brief
        public converts char data into long.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long charToLong( unsigned char *pData )
{
    
    unsigned long data;
  
    data =  *pData++;
    data <<= 8;
    data |= *pData++;
    data <<= 8;
    data |= *pData++;
    data <<= 8;
    data |= *pData;    

    return data;
}

/******************************************************************************/
/*!   \fn unsigned long nCharToLong( unsigned char *pData )

      \brief
        public creates unsigned long from byte swapped char array
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long nCharToLong( unsigned char *pData )
{
    
    unsigned long data;
  
    data =  *( pData + 3 );
    data <<= 8;
    data |= *( pData + 2 );
    data <<= 8;
    data |= *( pData + 1 );
    data <<= 8;
    data |= *pData;    

    return data;
}

/******************************************************************************/
/*!   \fn unsigned char getCharFromShort( unsigned short *pWord, unsigned char index )

      \brief
        public get character from indexed unsigned word.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned char getCharFromShort( unsigned short *pWord, unsigned char index )
{
    unsigned char char_ = 0;
    if( index == 0 ) {
      char_ = (unsigned char)( *pWord & 0x00ff );
    }
    else if( index == 1 ) {
      char_ = (unsigned char)( ( *pWord & 0xff00 ) >> 8 );
    }
    else {
        PRINTF("getCharFromShort(): Conversion error!\r\n");        
    }
    return char_;
}

/******************************************************************************/
/*!   \fn unsigned char getCharFromLong( unsigned long *pLong, unsigned char index )

      \brief
        public get character from indexed unsigned long.

      \author
          Aaron Swift
*******************************************************************************/
unsigned char getCharFromLong( unsigned long *pLong, unsigned char index )
{
    unsigned char char_ = 0;
    if( index == 0 ) {
    	char_ = (unsigned char)( *pLong & 0x00ff );
    }
    else if( index == 1 ) {
    	char_ = (unsigned char)( ( *pLong & 0xff00 ) >> 8 );
    }
    else if(index == 2 ) {
    	char_ = (unsigned char)( ( *pLong & 0xff0000 ) >> 16 );
    }
    else if(index == 3 ) {
    	char_ = (unsigned char)( ( *pLong & 0xff000000 ) >> 24 );
    }
    else {
        PRINTF("getCharFromLong(): Conversion error!\r\n");
    }
    return char_;

}

/******************************************************************************/
/*!   \fn void initPrHeader( unsigned char *pHdr, unsigned short size )

      \brief
        public initialize the header for the printer message.

      \author
          Aaron Swift
*******************************************************************************/
void initPrHeader( unsigned char *pHdr, unsigned short size )
{
    unsigned char *pSize = (unsigned char *)&size;
    if( pHdr != NULL ) {
      *pHdr++ = 0x46;
      *pHdr++ = 0x46;
      *pHdr++ = 0x00;
      *pHdr++ = LGCY_LOGICAL_ADDR_PRNTR;
      *pHdr++ = 0x00;
      *pHdr++ = 0x00;
      *pHdr++ = 0x00;
      *pHdr++ = LGCY_LOGICAL_ADDR_PRNTR;
        if( size > 255 ) {
            *pHdr++ = *( pSize + 1 );         
            *pHdr = *pSize;
        } else {
            *pHdr++ = 0x00;
            *pHdr = (char)size;
        }            
    }      
}

/******************************************************************************/
/*!   \fn void initWrHeader( unsigned char *pHdr, unsigned short size )

      \brief
        public initialize the header for the weigher message.

      \author
          Aaron Swift
*******************************************************************************/
void initWrHeader( unsigned char *pHdr, unsigned short size )
{
    unsigned char *pSize = (unsigned char *)&size;
    if( pHdr != NULL ) {
      *pHdr++ = 0x00;
      *pHdr++ = 0x00;
      *pHdr++ = 0x00;
      *pHdr++ = 0x09;
      *pHdr++ = 0x46;
      *pHdr++ = 0x46;
      *pHdr++ = 0x00;
      *pHdr++ = LGCY_LOGICAL_ADDR_WGR;
        if( size > 255 ) {
            *pHdr++ = *( pSize + 1 );         
            *pHdr = *pSize;
        } else {
            *pHdr++ = 0x00;
            *pHdr = (char)size;
        }            
    }      
}

unsigned char fromAsciiHex8( unsigned char *pData, unsigned char val )
{
    uint8_t tempA = 0, tempB = 0, result = 0;
    char bfr[2] = { 0 };
    memcpy( &bfr[0],  pData,  sizeof(unsigned long) );
    if( bfr[1] > 0x39 ) {
        result = bfr[1] - 'P';      
    } else {
        result = bfr[1] - '0';
    }    
    tempA = result;

    if( bfr[0] > 0x39 ) {
        result = bfr[0] - 'P';      
    } else {
        result = bfr[0] - '0';
    }
    tempB = result;

    result = (uint16_t)( tempB << 4 );    
    result |=  (uint16_t)tempA;
   

    return (unsigned char)result;
}

unsigned short fromAsciiHex16( unsigned char *pData, unsigned short val )
{
    uint8_t tempA = 0, tempB = 0, tempC = 0, tempD = 0; 
    uint16_t conversion = 0;
    char result = 0;    
    char bfr[5] = { 0 };
    memcpy( &bfr[0],  pData,  sizeof(unsigned long) );
    
    
    if( bfr[0] > 0x39 ) {
        result = bfr[0] - 'P';      
    } else {
        if( bfr[0] >= 0x30 ) {
            result = bfr[0] - '0';
        } else {
            result = bfr[0] - 0x17;     /* 0x11 */
        }
    }
    tempC = result;
    
    if( bfr[1] > 0x39 ) {
        result = bfr[1] - 'P';      
    } else {
        if( bfr[1] >= 0x30 ) {
            result = bfr[1] - '0';
        } else {
            result = bfr[1] - 0x17;
        }
    }
    tempD = result;

    conversion = (uint16_t)( tempC << 4 );  
    conversion |=  (uint16_t)tempD;
    conversion <<= 8;
    
    if( bfr[2] > 0x39 ) {
        result = bfr[2] - 'P';      
    } else {
        if( bfr[2] >= 0x30 ) {
            result = bfr[2] - '0';
        } else {
            result = bfr[2] - 0x17;
        }
    }    
    tempB = result;

    if( bfr[3] > 0x39 ) {
        result = bfr[3] - 'P'; 
    } else {
        if( bfr[3] >= 0x30 ) {
            result = bfr[3] - '0';
        } else {
            result = bfr[3] - 0x17;
        }
    }
    tempA = result;

    conversion |= (uint16_t)( tempB << 4 );    
    conversion |=  (uint16_t)tempA;
     
    return (unsigned short)conversion;
  
}

unsigned long fromAsciiHex32( unsigned char *pData, unsigned long val )
{
    uint32_t temp = 0;
    char bfr[5] = { 0 };
    memcpy( &bfr[0],  pData,  sizeof(unsigned long) );    
    return (unsigned long)temp;  
}

void fromAsciiHexBlock( unsigned char *pData, unsigned char *pSrc, unsigned short length )
{
    uint16_t index = 0;
    while( length ) {
        uint8_t val = 0;  
        *pData++ = fromAsciiHex8( &pSrc[ index ], val );
        length -= 2;
        index += 2;
    }
}

unsigned short binBlockToAscii( unsigned char *pData, unsigned char *pSrc, unsigned short length )
{
    uint16_t bytesWritten = 0, srcIdx = 0;

    while( length ) {
        bytesWritten += toAsciiHex(  &pData[ bytesWritten ], 
                                     SIZE_IN_NIBBLES(sizeof(unsigned char)), 
                                     pSrc[ srcIdx++ ] );
        --length;
    }
    return bytesWritten;  
}


unsigned char toAsciiHex( unsigned char *pData, unsigned char len, int val  )
{
    uint8_t length = 0;
    if( len == sizeof(int64_t) ) {
        uint64_t val_ = (uint64_t)val;        
        sprintf( (char *)pData, "%0*llX", SIZE_IN_NIBBLES( sizeof( int64_t ) ), val_ ); 
        length = ( sizeof(uint64_t) * 2 ); 
    } else if( len == sizeof(int32_t) ) {
        uint32_t val_ = (uint32_t)val;
        sprintf( (char *)pData, "%0*X", SIZE_IN_NIBBLES( sizeof( int32_t ) ), val_ ); 
        length = ( sizeof(uint32_t) * 2 ); 
    } else if( len == sizeof(int16_t) ) {
        uint16_t val_ = (uint16_t)val;
        sprintf( (char *)pData, "%0*X", SIZE_IN_NIBBLES( sizeof( int16_t ) ), val_ );     
        length = ( sizeof(uint16_t) * 2 ); 
    } else {
        uint8_t val_ = (uint8_t)val;
        length = sprintf( (char *)pData, "%0*X", SIZE_IN_NIBBLES( sizeof( int8_t ) ), val_ );     
    }
    return length;
}

unsigned char stoAsciiHex( unsigned char *pData, unsigned char len, int val )
{
     uint8_t length = 0;
        uint16_t val_ = (uint16_t)val;
        sprintf( (char *)pData, "%0*X", SIZE_IN_NIBBLES( sizeof( uint16_t ) ), val_ );     
        length = ( sizeof(uint16_t) * 2 ); 
  return length;
}

void longToAsciiHex( unsigned char *pData, unsigned char len, long val )
{
    sprintf( (char *)pData, "%0*X", SIZE_IN_NIBBLES( sizeof( long ) ), val );   
}
               
void charToAsciiHex( unsigned char *pData, unsigned char len, char val )
{
    sprintf( (char *)pData, "%0*X", SIZE_IN_NIBBLES( sizeof( char ) ), val );  
}
               
void auditLogToAsciiHex( unsigned char *pData, unsigned char *pSrc )
{
  
  
}

#include <ctype.h>
void testAscii( void )
{
    char ascii[] = {'A','a','B','b','C','c','D','d','E','e','F','f','1','2','3','4','5','6','7','8','9'};
    uint8_t result = 0;
  
    for( int i = 0; i < 21; i++ ) {
        if( isdigit(ascii[i]) ) {
            result = ascii[i] - '0';
        } else if( isxdigit(ascii[i]) ) {
            result = toupper(ascii[i]) - 'A' + 33;
        }
        PRINTF("result: %1x\r\n", result);
    }
}