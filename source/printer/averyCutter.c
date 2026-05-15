#include "averyCutter.h"
#include "semphr.h"
#include "fsl_debug_console.h"
//#include "globalPrinterTask.h"
#include "takeupMotor.h"
#include "idleTask.h"
#include "vendor.h"
#include "translator.h"
#include "queueManager.h"


SemaphoreHandle_t       pCutDoneSemaphore       = NULL;

static TaskHandle_t     cHandle_                = NULL;
static QueueHandle_t    pMsgQHandle_            = NULL;
static QueueHandle_t    pIPMsgQueue             = NULL;
static bool             suspend_                = false;
static bool             cutterDetected          = false;
static bool             cutPending              = false;
static bool             statusPending           = false;
uint16_t                cutterStatusCounter     = 0;
uint16_t                cutterStatusTimeOutCounter = 0;

extern Pr_Config config_;

#define TFinkOyaneCutterToDo
#define TFinkOyaneCutter

AT_NONCACHEABLE_SECTION_ALIGN( lpuart_handle_t cutterHandle_, 4U ); 
AT_NONCACHEABLE_SECTION_ALIGN( unsigned char uartTxBfr[ MAX_MESSGAE_SIZE ], 4U );
AT_NONCACHEABLE_SECTION_ALIGN( unsigned char uartRxBfr[ MAX_MESSGAE_SIZE ], 4U );
AT_NONCACHEABLE_SECTION_ALIGN( unsigned char uartRxBfr2[ MAX_MESSGAE_SIZE ], 4U );
AT_NONCACHEABLE_SECTION_ALIGN( static CutterMgr cutterMgr_, 4U );

const unsigned short crc16[CRC_TABLE_SIZE] = 
                            {   0xABCD, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 
                                0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 
                                0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 
                                0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
                                0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 
                                0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 
                                0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 
                                0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
                                0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 
                                0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 
                                0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 
                                0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
                                0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 
                                0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 
                                0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 
                                0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
                                0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 
                                0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 
                                0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 
                                0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
                                0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 
                                0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 
                                0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 
                                0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
                                0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 
                                0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 
                                0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 
                                0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
                                0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 
                                0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 
                                0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 
                                0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
                                0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 
                                0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 
                                0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 
                                0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
                                0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 
                                0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 
                                0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 
                                0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 
                                0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 
                                0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 
                                0x4100, 0x81C1, 0x8081, 0x4040 };


const TickType_t                lightSleep = 500 / portTICK_PERIOD_MS;
const TickType_t                deepSleep = 2000 / portTICK_PERIOD_MS;


SemaphoreHandle_t               cMutex_;
static ACTSTATES                actState_;



extern uint32_t BOARD_DebugConsoleSrcFreq(void);

/******************************************************************************/
/*!   \fn BaseType_t createAveryCutterTask( QueueHandle_t msgQueue, 
                                            QueueHandle_t printerMsgQueue )

      \brief
        This function creates a cutter task to manage messaging between the 
        device and host and printer task.   
       
      \author
          Aaron Swift
*******************************************************************************/                
BaseType_t createAveryCutterTask( QueueHandle_t msgQueue, 
                                  QueueHandle_t printerMsgQueue )
{
    BaseType_t result;
    PRINTF("createAveryCutterTask(): Starting...\r\n" );
    /* set our task state */
    actState_ = AC_INIT_INTERFACE_;
    /* assign my internal message queue */
    pMsgQHandle_ = msgQueue;
    /* assign the printers internal message queue */
    pIPMsgQueue = printerMsgQueue;
    
    pCutDoneSemaphore = xSemaphoreCreateBinary();
    
    if( ( pMsgQHandle_ != NULL ) && ( pIPMsgQueue != NULL ) ) {
        /* create printer task thread */
        result = xTaskCreate( averyCutterTask,  "CutterTask", configMINIMAL_STACK_SIZE,
                                            NULL, cutter_task_PRIORITY, &cHandle_ );
    } else {
        PRINTF("createAveryCutterTask(): Failed to create Cutter task. Queue is null!\r\n" );
    }
    return result;
}

/******************************************************************************/
/*!   \fn bool initCutter( void ) 

      \brief
        This function initializes the cutter manager and opens the cutter 
        interface. Returns  
       
      \author
          Aaron Swift
*******************************************************************************/                
bool initCutter( void )
{
    bool status = false;
    /* reset the manager */
    memset( (void *)&cutterMgr_, 0, sizeof(cutterMgr_) );
    
    cutterMgr_.version.productName[19] = '\0';
    cutterMgr_.version.firmware[12] = '\0';
    cutterMgr_.version.issueDate[16] = '\0';
    cutterMgr_.version.date[11] = '\0'; 
    cutterMgr_.version.time[8] = '\0';
    
    /* clear the mgs buffers */
    memset( (void *)&uartTxBfr[0], 0, MAX_MESSGAE_SIZE );
    memset( (void *)&uartRxBfr[0], 0xff, MAX_MESSGAE_SIZE );
 
    /* create mutex for interface */
    vSemaphoreCreateBinary( cMutex_ )
    //cMutex_ = xSemaphoreCreateMutex(); 
    if( cMutex_ ) {             
        /* open the interface */
        if( openCutterInterface( CUTTER_UART_BAUD ) ) {                                    
            status = true;
            
            //PRINTF("initCutter() - true\r\n");
        }
    } else {
        PRINTF("initCutter(): Failed to create mutex!\r\n" );
    }
    return status;
}

/******************************************************************************/
/*!   \fn bool sendACCut( CutterMgr *pMgr, ACCutPaper *pMsg )

      \brief
        This function builds and sends the cut command message to the cutter 
        interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACCut( CutterMgr *pMgr, ACCutPaper *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    //PRINTF("sendACCut(): send cut msg\r\n");
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_CUT_, &msg, 0, pMsg->speed );
            stuffTxBuffer( (unsigned char *)&msg, (unsigned char)( HEADER_SIZE + sizeof(ACCutPaper) ) );
            
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], ( getMessageSize( msg.header.command ) - 1 ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = ( sizeof(ACHeader) + sizeof(ACCutPaper) + 5 );
            
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACCut(): Failed to send AC_CUT_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACCut(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACCut(): Mutex is null!\r\n" );
    }
    return sent;
}

/******************************************************************************/
/*!   \fn bool sendACReqStatus( CutterMgr *pMgr, ACCutPaper *pMsg )

      \brief
        This function builds and sends a request for status command message 
        to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReqStatus( CutterMgr *pMgr )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
        
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            /* build the cutter message */ 
            buildCutterMsg( AC_REQ_STATUS_, &msg, NULL, NULL );
            stuffTxBuffer( (unsigned char *)&msg, (unsigned char)HEADER_SIZE );
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );
            uartTxBfr[5] = '1';
            uartTxBfr[6] = '1';
            uartTxBfr[7] = '1';
            uartTxBfr[8] = '1';
            uartTxBfr[14] = 'E';
            uartTxBfr[15] = '1';
            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            

#if 0
            uint16_t totalSize = xfer.dataSize;
            
            PRINTF("\r\n========= RAW TX BUFFER =========\r\n");

            for (uint16_t i = 0; i < totalSize; i++)
            {
                PRINTF("%02X ", uartTxBfr[i]);
            }
            PRINTF("\r\n=================================\r\n");

            /* ---------------- DECODE ---------------- */

            uint8_t *buf = uartTxBfr;

            PRINTF("\r\n========= DECODED PACKET =========\r\n");

            /* STX */
            PRINTF("STX        : 0x%02X\r\n", buf[0]);

            /* Basic header fields */
            PRINTF("Slave ID   : %c\r\n", buf[1]);
            PRINTF("Command    : %c\r\n", buf[2]);
            PRINTF("Type       : %c\r\n", buf[3]);
            PRINTF("CR         : %c\r\n", buf[4]);

            /* Config (4 ASCII hex chars) */
            PRINTF("Config     : %c%c%c%c\r\n",
                   buf[5], buf[6], buf[7], buf[8]);

            /* Length (4 ASCII hex chars) */
            PRINTF("Length     : %c%c%c%c\r\n",
                   buf[9], buf[10], buf[11], buf[12]);

            /* Convert ASCII hex length manually (no helper function) */
            uint16_t lengthVal = 0;
            for (int i = 9; i < 13; i++)
            {
                lengthVal <<= 4;

                if (buf[i] >= '0' && buf[i] <= '9')
                    lengthVal |= (buf[i] - '0');
                else if (buf[i] >= 'A' && buf[i] <= 'F')
                    lengthVal |= (buf[i] - 'A' + 10);
            }

            PRINTF("Length (dec): %d\r\n", lengthVal);

            /* If Cut command, decode speed */
            if (buf[2] == 'C')
            {
                PRINTF("Speed      : %c\r\n", buf[13]);
            }

            /* CRC (last 5 bytes are CRC[4] + ETX[1]) */
            uint16_t crcIndex = totalSize - 5;

            PRINTF("CRC (ASCII): %c%c%c%c\r\n",
                   buf[crcIndex],
                   buf[crcIndex + 1],
                   buf[crcIndex + 2],
                   buf[crcIndex + 3]);

            /* ETX */
            PRINTF("ETX        : 0x%02X\r\n", buf[totalSize - 1]);

            PRINTF("==================================\r\n\r\n");
#endif
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACReqStatus(): Failed to send AC_REQ_STATUS_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACReqStatus(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACReqStatus(): Mutex is null!\r\n" );
    }
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACReqStatus( CutterMgr *pMgr, ACCutPaper *pMsg )

      \brief
        This function builds and sends a request for version command message 
        to the cutter interface. The version recieved is based on the version
        index.
        
        0: Product name
        1: Firmware number
        2: Firmware issue
        3: Firmware date
        4: Time
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReqVersion( CutterMgr *pMgr, ACRVersion *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_VERSION_, &msg, pMsg->index, NULL );
            stuffTxBuffer( (unsigned char *)&msg, (unsigned char)( sizeof(ACHeader) + sizeof(ACRVersion) ) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            
#if 0
            uint16_t totalSize = xfer.dataSize;
            
            PRINTF("\r\n========= RAW TX BUFFER =========\r\n");

            for (uint16_t i = 0; i < totalSize; i++)
            {
                PRINTF("%02X ", uartTxBfr[i]);
            }
            PRINTF("\r\n=================================\r\n");

            /* ---------------- DECODE ---------------- */

            uint8_t *buf = uartTxBfr;

            PRINTF("\r\n========= DECODED PACKET =========\r\n");

            /* STX */
            PRINTF("STX        : 0x%02X\r\n", buf[0]);

            /* Basic header fields */
            PRINTF("Slave ID   : %c\r\n", buf[1]);
            PRINTF("Command    : %c\r\n", buf[2]);
            PRINTF("Type       : %c\r\n", buf[3]);
            PRINTF("CR         : %c\r\n", buf[4]);

            /* Config (4 ASCII hex chars) */
            PRINTF("Config     : %c%c%c%c\r\n",
                   buf[5], buf[6], buf[7], buf[8]);

            /* Length (4 ASCII hex chars) */
            PRINTF("Length     : %c%c%c%c\r\n",
                   buf[9], buf[10], buf[11], buf[12]);

            /* Convert ASCII hex length manually (no helper function) */
            uint16_t lengthVal = 0;
            for (int i = 9; i < 13; i++)
            {
                lengthVal <<= 4;

                if (buf[i] >= '0' && buf[i] <= '9')
                    lengthVal |= (buf[i] - '0');
                else if (buf[i] >= 'A' && buf[i] <= 'F')
                    lengthVal |= (buf[i] - 'A' + 10);
            }

            PRINTF("Length (dec): %d\r\n", lengthVal);

            /* If Cut command, decode speed */
            if (buf[2] == 'C')
            {
                PRINTF("Speed      : %c\r\n", buf[13]);
            }

            /* CRC (last 5 bytes are CRC[4] + ETX[1]) */
            uint16_t crcIndex = totalSize - 5;

            PRINTF("CRC (ASCII): %c%c%c%c\r\n",
                   buf[crcIndex],
                   buf[crcIndex + 1],
                   buf[crcIndex + 2],
                   buf[crcIndex + 3]);

            /* ETX */
            PRINTF("ETX        : 0x%02X\r\n", buf[totalSize - 1]);

            PRINTF("==================================\r\n\r\n");
#endif
            
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACReqVersion(): Failed to send AC_VERSION_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACReqVersion(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACReqVersion(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACHome( CutterMgr *pMgr, ACHome *pMsg )

      \brief
        This function builds and sends a home command message 
        to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACHome( CutterMgr *pMgr, ACHome *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_HOME_, &msg, pMsg->speed, NULL );
            stuffTxBuffer( (unsigned char *)&msg, ( sizeof(ACHeader) + sizeof(ACHome) ) );
            
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );
            
            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACHome(): Failed to send AC_HOME_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACHome(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACHome(): Mutex is null!\r\n" );
    }
    return sent;        
}

/******************************************************************************/
/*!   \fn bool sendACCutSteps( CutterMgr *pMgr, ACCutSteps *pMsg )

      \brief
        This function builds and sends a cut command message 
        with the distance to travel and how fast to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACCutSteps( CutterMgr *pMgr, ACCutSteps *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_CUT_STEPS_, &msg, pMsg->distance, pMsg->speed );
            stuffTxBuffer( (unsigned char *)&msg, ( sizeof(ACHeader) + sizeof(ACCutSteps) ) );
            
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACCutSteps(): Failed to send AC_CUT_STEPS_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACCutSteps(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACCutSteps(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACReadProfile( CutterMgr *pMgr, ACReadProfile *pMsg )

      \brief
        This function builds and sends a profile command message 
         to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReadProfile( CutterMgr *pMgr, ACReadProfile *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_READ_SPEED_, &msg, pMsg->data, NULL );
            stuffTxBuffer( (unsigned char *)&msg, ( sizeof(ACHeader) + sizeof(ACReadProfile) ) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );
            
            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACReadProfile(): Failed to send AC_READ_SPEED_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACReadProfile(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACReadProfile(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACTestMode( CutterMgr *pMgr )

      \brief
        This function builds and sends a test mode command message 
         to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                               
void enterTestMode(void)
{
   sendACTestMode(&cutterMgr_);
}

bool sendACTestMode( CutterMgr *pMgr )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_TEST_MODE_, &msg, NULL, NULL );
            stuffTxBuffer( (unsigned char *)&msg, sizeof(ACHeader) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACTestMode(): Failed to send AC_TEST_MODE_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACTestMode(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACTestMode(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACReset( CutterMgr *pMgr )

      \brief
        This function builds and sends a reset command message 
         to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReset( CutterMgr *pMgr )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_RESET_, &msg, NULL, NULL );
            stuffTxBuffer( (unsigned char *)&msg, sizeof(ACHeader) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACTestMode(): Failed to send AC_RESET_!\r\n" );
            } else {
                sent = true;
                PRINTF("sendACTestMode(): send AC_RESET_!\r\n" );
            }
        } else {
            PRINTF("sendACTestMode(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACTestMode(): Mutex is null!\r\n" );
    } 
    return sent;    
    
}

/******************************************************************************/
/*!   \fn unsigned char getMessageSize( unsigned char type )

      \brief
        This function returns the fixed size of the message based on the message 
        type. 
               
      \author
          Aaron Swift
*******************************************************************************/                
unsigned char getMessageSize( unsigned char type )
{
    unsigned char size = 0;
   
    if( type == CUTTER_CUT_PAPER ) {
        size = PAPER_CUT_MSG_SIZE_;
    } else if( type == CUTTER_RETURN_HOME ) {
        size = RETURN_HOME_MSG_SIZE_;
    } else if( type == CUTTER_CUT_PAPER_STEPS ) {
        size = PAPER_CUT_STEPS_MSG_SIZE_;
    } else if( type == CUTTER_READ_SPEED ) {
        size = READ_PROFILE_MSG_SIZE_;
    } else if( type == CUTTER_RESET ) {
        size = DEFAULT_MSG_SIZE_;
    } else if( type == CUTTER_STATUS ) {
        size = DEFAULT_MSG_SIZE_;
    } else if( type == CUTTER_TEST_MODE ) {
        size = DEFAULT_MSG_SIZE_;
    } else if( type == CUTTER_VERSION ) {
        size = READ_VERSION_MSG_SIZE_;
    } else {
        PRINTF("getMessageSize(): Unknown message type: %d!\r\n", type );
    }
    return size;
}

/******************************************************************************/
/*!   \fn static void averyCutterTask( void *pvParameters )

      \brief
        This function handles initializing the cutter mechanism and processes  
        requests from the printer manager.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void averyCutterTask( void *pvParameters )
{
    static ACRVersion version;
    version.index = 0;
    
    PRINTF("averyCutterTask(): Thread running...\r\n" ); 
    while( !suspend_ ) 
    {
        #if 1
        static ACTSTATES previousState = AC_ERROR_;    
        
        if(actState_ != previousState) 
        {
           previousState = actState_;
           //PRINTF("Cutter State_ = : %d\r\n", actState_);
        }       
        #endif
        
        switch( actState_ )
        {
            case AC_INIT_INTERFACE_: 
            {
                //PRINTF("CUTTER TASK RUNNING init\r\n");
                /* open interface to cutter mechanism */
                if( initCutter() ) 
                {
                    /* set initialization sequence flag */
                    cutterMgr_.initSeq = true;
                    
                    /* request the cutter status */
                    if( sendACReqStatus( &cutterMgr_ ) ) 
                    { 
                        receiveCutterMsg( DEFAULT_MSG_RX_SIZE_ );
                        actState_ = AC_INIT_DEVICE_STATUS_;
                    } 
                    else 
                    {
                        PRINTF("averyCutterTask(): sendACReqStatus() failed!\r\n" ); 
                    }
                } 
                else 
                {
                    PRINTF("averyCutterTask(): initCutter() failed!\r\n" ); 
                }
                
                break;
            }
            case AC_INIT_DEVICE_STATUS_: 
            {
                //PRINTF("CUTTER TASK RUNNING device_status\r\n");
              
                cutterStatusTimeOutCounter++;
                
                #define CUTTER_STATUS_TIME_OUT 50000
                
                if(cutterStatusTimeOutCounter > CUTTER_STATUS_TIME_OUT)
                {
                    setCutterInstalled(false);
                  
                    if( cHandle_ != NULL )
                    {
                        PrSysInfo info;
                        info.msgType = PR_SYS_INFO;
                        info.pid = getProductId();
                        info.id = config_.instance;        

                        info.printhead_size = HEAD_DOTS_80MM;
                        info.buffer_size = PRINTER_BUFFER_SIZE_80MM;    

                        info.transfer_size = 512;
                        info.headType = getPrintHeadType();
                        info.cutterInstalled = getCutterDetected();
                        
                        config_.cutterEnabled = false;
                        
                        info.cutterEnabled = config_.cutterEnabled;
                        info.configValid = true;   
                        
                        
                        //sendPrSysInfo( &info );
                        
                        //createGlobalPrinterTask( RT_PRINTER_SERVICE_SCALE_80MM, (QueueHandle_t)getPrinterQueueHandle() );
                      
                        PRINTF("\r\nCutter not detected - cutter tasked deleted");
                        vTaskDelete( cHandle_ );
                    }
                }
              
                if( cutterMgr_.msgReady ) 
                {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                   
                    cutterMgr_.initSeq = false;
                    setCutterInstalled(true);
                    cutterStatusTimeOutCounter = 0;
                    actState_ = AC_INIT_DEVICE_TX_VERSION_;
                    
                    PrSysInfo info;
                    info.msgType = PR_SYS_INFO;
                    info.pid = getProductId();
                    info.id = config_.instance;        

                    info.printhead_size = HEAD_DOTS_80MM;
                    info.buffer_size = PRINTER_BUFFER_SIZE_80MM;    

                    info.transfer_size = 512;
                    info.headType = getPrintHeadType();
                    info.cutterInstalled = getCutterDetected();
                    
                    config_.cutterEnabled = true;
                    
                    info.cutterEnabled = config_.cutterEnabled;
                    info.configValid = true;   
                    
                    
                    //sendPrSysInfo( &info );
                    
                    //createGlobalPrinterTask( RT_PRINTER_SERVICE_SCALE_80MM, (QueueHandle_t)getPrinterQueueHandle() );
                    
                    PRINTF("\r\nCutter detected\r\n\r\n");
                }
                
                break;
            }
            case AC_INIT_DEVICE_TX_VERSION_: 
            {
                //PRINTF("CUTTER TASK RUNNING tx_version\r\n");
                if( sendACReqVersion( &cutterMgr_, &version ) ) 
                { 
                    if( version.index != MAX_VERSION_INDEX ) 
                    {
                        //PRINTF("\r\n!= MAX VERSION INDEX - INDEX = %d\r\n", version.index);
                        receiveCutterMsg( getVersionRxMsgSize( version.index ) );
                        
                        version.index++;
                    }
                    
                    actState_ = AC_INIT_DEVICE_RX_VERSION_;
                } 
                else 
                {
                    PRINTF("averyCutterTask(): sendACReqVersion() failed!\r\n" ); 
                }
                
                break;
            }
            case AC_INIT_DEVICE_RX_VERSION_: 
            {
                //PRINTF("CUTTER TASK RUNNING rx_version\r\n");
                if( cutterMgr_.msgReady ) 
                {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                                       
                    
                    /* do we have all version info? */
                    if( version.index == MAX_VERSION_INDEX ) 
                    {
                        version.index = 0;

                        /* tell the printer that we are ready to process cmds */
                        ICutterGeneric imsg;
                        imsg.msgType = _I_CUTTER_READY_FOR_CMD;
                        
                        BaseType_t result = xQueueSend( pIPMsgQueue, (void *)&imsg, 0 );
                        
                        if( result != pdPASS ) 
                        {
                            PRINTF("printerTask(): Failed to post Cutter message!\r\n" );       
                        }                            
                        
                        actState_ = AC_WAIT_FOR_COMMAND_;                          
                    } 
                    else 
                    {
                        /* get the next */                        
                        actState_ = AC_INIT_DEVICE_TX_VERSION_;
                    }
                }                
                break;
            }
            case AC_INIT_SEND_DOOR_STATUS_: 
            {            
                break;
            }
            
            case AC_INIT_WAIT_DOOR_STATUS_: 
            {
                break;
            }
            case AC_WAIT_FOR_COMMAND_: 
            {
                if(cutterMgr_.initSeq == true)
                {
                    cutterMgr_.initSeq = false;
                }
                
                if( pMsgQHandle_ != NULL ) 
                {                  
                    ICutterGeneric cMsg;
                    ICMessages cMsg2;
                    
                    int numMgs = uxQueueMessagesWaitingFromISR( pMsgQHandle_ );
                    
                    if( numMgs ) 
                    {
                        if( xQueueReceiveFromISR( pMsgQHandle_, &cMsg, 0 ) ) 
                        {             
                            cMsg2.generic.msgType = cMsg.msgType;
                            
                            //PRINTF("\r\nC msg type %d", cMsg2.generic.msgType);
                            
                            handleInternalMessage(&cMsg2);
                        } 
                    }
                } 
                
                if( cutPending == true )
                {
                    cutPending = false;
                    
                    actState_ = AC_PROCESS_CUT_;
                }
                
                break;
            }            
            case AC_PROCESS_CUT_ : 
            {
                //PRINTF("\r\naveryCutterTask(): initiate CUT");
                ACCutPaper steps;
                steps.speed = 9;            
                if( sendACCut( &cutterMgr_, &steps ) )  
                {
                    receiveCutterMsg( PAPER_CUT_RX_STEPS_MSG_SIZE_ ); 
                    actState_ = AC_PROCESS_CUT_RESPONSE_;
                } 
                else 
                {
                    PRINTF("averyCutterTask(): Failed to send cut command!\r\n" );
                }  
                
                break;
            }
            
            case AC_PROCESS_CUT_RESPONSE_: 
            {
                //PRINTF("averyCutterTask():  ac_process_cut_response\r\n");
                if( cutterMgr_.msgReady ) 
                {   
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );  
                }
                
                break;
            }
            case AC_PROCESS_HOME_: 
            {
                //PRINTF("ac_process_home\r\n");
                ACHome home;
                home.speed = 9;
 
                if( sendACHome( &cutterMgr_, &home ) ) 
                {
                    receiveCutterMsg( RETURN_HOME_RX_MSG_SIZE_  ); 
                    actState_ = AC_PROCESS_HOME_RESPONSE_;                    
                } 
                else 
                {
                    PRINTF("averyCutterTask(): Failed to send home command!\r\n" );
                } 
                
                break;
            }         
            case AC_PROCESS_HOME_RESPONSE_: 
            {
               //PRINTF("averyCutterTask() ac_process_home_response\r\n");
                if( cutterMgr_.msgReady ) 
                {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                 
                }  
                
                break;
            }
            case AC_ERROR_: 
            {
                PRINTF("CUTTER TASK RUNNING error\r\n");
                /* TO DO: replace with task deletion */
                /* sleep for a second */
                vTaskDelay( deepSleep );
                break;
            }
            case AC_TX_DEVICE_STATUS_: 
            {
                //PRINTF("CUTTER TASK RUNNING TX device_status\r\n");
                /* request the cutter status */
                if( sendACReqStatus( &cutterMgr_ ) ) 
                { 
                    receiveCutterMsg( DEFAULT_MSG_RX_SIZE_ );
                    actState_ = AC_RX_DEVICE_STATUS_;
                } 
                else 
                {
                    PRINTF("averyCutterTask(): sendACReqStatus() failed!\r\n" ); 
                }
                
                break;
            }
            case AC_RX_DEVICE_STATUS_: 
            {
                //PRINTF("CUTTER TASK RUNNING RX device_status\r\n");
                if( cutterMgr_.msgReady ) 
                {    
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                    
                }
                
                break;
            }
        }
        
        /* TFinkOyaneCutter */
        static bool PrSysInfoMsgSent = false;
        if(cutterDetected == true && PrSysInfoMsgSent == false) {   
           PrSysInfoMsgSent = true;
        }
        taskYIELD();
    }
    vTaskSuspend(NULL);  
}

/******************************************************************************/
/*!   \fn void handleInternalMessage( ICMessages *pMsg )

      \brief
        This function handles internal cutter messsages from the Printer task.
       
      \author
          Aaron Swift
*******************************************************************************/      
void handleInternalMessage( ICMessages *pMsg )
{  
    //PRINTF("\r\nhandleInternalMessage");
  
    switch( pMsg->generic.msgType ) 
    {
        case _I_CUTTER_CUT_CMD: 
        {
            if(actState_ == AC_WAIT_FOR_COMMAND_)
            {
                actState_ = AC_PROCESS_CUT_;
            }
            else
            {
                PRINTF("\r\n!= AC_WAIT_FOR_COMMAND_ %d", actState_);
                cutPending = true;
            }
              
            break;
        }
        case _I_CUTTER_HOME_CMD: 
        {
            if(actState_ == AC_WAIT_FOR_COMMAND_)
            {
                actState_ = AC_PROCESS_HOME_;
            }
          
            break;
        }
        case _I_CUTTER_REQ_STATUS: 
        {
            if(actState_ == AC_WAIT_FOR_COMMAND_)
            {
                actState_ = AC_TX_DEVICE_STATUS_;
            }
            else
            {
                PRINTF("\r\n!= AC_WAIT_FOR_COMMAND_ %d", actState_);
                statusPending = true;
            }
            
            
            break;
        }
        case _I_CUTTER_REQ_VERSION: 
        {

            break;
        }
        default: 
        {
            break;
        }
    }
}

/******************************************************************************/
/*!   \fn static bool receiveCutterMsg( unsigned char rxSize  )

      \brief
        This function receives cutter message from uart1 non blocking. 
       
      \author
          Aaron Swift
*******************************************************************************/      
static bool receiveCutterMsg( unsigned char rxSize  )
{
    lpuart_transfer_t xfer;    
    size_t result, rxBytes = 0;
    bool ready = false;
    xfer.rxData = &uartRxBfr[0];
    xfer.dataSize = rxSize;
    
    result = LPUART_TransferReceiveNonBlocking( LPUART1, &cutterHandle_, &xfer, &rxBytes );
  
    //result = LPUART_ReadBlocking(LPUART1, uartRxBfr, rxSize);
    
    
    //result = LPUART_ReadBlocking(LPUART1, &uartRxBfr[0], rxSize);

    //PRINTF("result = %d\r\n", result);
    
    
    if( result == kStatus_LPUART_RxBusy ) {
        PRINTF("receiveCutterMsg(): Receiver is busy!\r\n" );
    }
    else if(result == kStatus_LPUART_RxIdle)
    {
        PRINTF("receiveCutterMsg(): Receiver is idle!!\r\n" );
    }
    else if(result == kStatus_Success)
    {
        //PRINTF("receiveCutterMsg(): Receiver Success!!\r\n" );
        //cutterMgr_.msgReady = true;
    }
    else if(result == kStatus_LPUART_RxHardwareOverrun)
    {
        PRINTF("receiveCutterMsg(): kStatus_LPUART_RxHardwareOverrun\r\n" );
    }
    else {
        ready = true;
        PRINTF("receiveCutterMsg(): No success, idle, or busy\r\n");
    }
    return ready;
}

/******************************************************************************/
/*!   \fn static size_t getVersionRxMsgSize( unsigned char index )

      \brief
        This function returns version string size for given index. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static size_t getVersionRxMsgSize( unsigned char index )
{
    size_t size = 0;
    //PRINTF("\r\nversion index: %d", index);
    if( index == 0 ) 
        size = READ_VERSION0_RX_MSG_SIZE_;
    if( index == 1 ) 
        size = READ_VERSION1_RX_MSG_SIZE_;
    if( index == 2 ) 
        size = READ_VERSION2_RX_MSG_SIZE_;
    if( index == 3 ) 
        size = READ_VERSION3_RX_MSG_SIZE_;
    if( index == 4 ) 
        size = READ_VERSION4_RX_MSG_SIZE_;
    return size;
}

/******************************************************************************/
/*!   \fn static void stuffTxBuffer( unsigned char *pMsg, unsigned char size )

      \brief
        This function copies the data into the transmit buffer for a given 
        message size.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void stuffTxBuffer( unsigned char *pMsg, unsigned char size )
{
    if( pMsg != NULL ) {
        memcpy( &uartTxBfr[0], pMsg, size );
    } else {
        PRINTF("stuffTxBuffer(): pMsg is null!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn static void buildCutterMsg( ACCCMDS cmd, ACutterMsgs *pMsg, 
                                      unsigned short arg1, unsigned short arg2 )
      \brief
        This function copies the data into the transmit buffer for a given 
        message size.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void buildCutterMsg( ACCCMDS cmd, ACutterMsgs *pMsg, unsigned short arg1, 
                            unsigned short arg2 )
{
    unsigned char *pChar = (unsigned char *)&pMsg->header.config; 
    /* same args for all messages */
    pMsg->header.start          = AC_STX;      
    pMsg->header.slaveId        = CUTTER_SLAVE_ID;
    pMsg->header.type           = 'Q';
    /* all outgoing messages */
    pMsg->header.cr             = 'C';
    
    if( cmd == AC_CUT_ || cmd == AC_HOME_ ||cmd == AC_VERSION_ || cmd == AC_REQ_STATUS_ )
    {
        memcpy(pChar, "0001", 4);
        pChar += 4;
    }
    else
    {
        for( int i = 0; i < sizeof(long); i++ ) 
        { 
            *pChar++                = '0';
        }
    }
                
    switch( cmd ) 
    {
        case AC_CUT_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_CUT_PAPER;
            pMsg->header.length             = PAPER_CUT_MSG_SIZE;
                         
            if( arg2 <= MAX_SPEED ) {
                arg2 <<= 4;
                hexToAscii( (unsigned char *)&pMsg->body.cut.speed, (unsigned char)arg2  ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_ speed argument is out of bounds %d\r\n", arg2);
            }                
            break;
        }
        case AC_HOME_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_RETURN_HOME;
            pMsg->header.length             = RETURN_HOME_MSG_SIZE;
            /* bounds check our data */
            if( arg1 <= MAX_SPEED ) {                
                //hexToAscii( (unsigned char *)&pMsg->body.home.speed, arg1 );
                pMsg->body.home.speed = ( '30' + arg1 );
            } else {
                PRINTF( "buildCutterMsg(): AC_HOME_ speed argument is out of bounds %d\r\n", arg1);
            }
            break;
        }
        case AC_CUT_STEPS_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_CUT_PAPER_STEPS;
            pMsg->header.length             = PAPER_CUT_STEPS_MSG_SIZE;
            /* bounds check our data */
            if( arg1 <= MAX_DISTANCE ) {
                hexToAscii( (unsigned char *)&pMsg->body.cutSteps.distance, arg1 ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_STEPS_ distance argument is out of bounds %d\r\n", arg1);
            }                
            if( arg2 <= MAX_SPEED ) {
                arg2 <<= 4;
                hexToAscii( (unsigned char *)&pMsg->body.cutSteps.speed, arg2 ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_STEPS_ speed argument is out of bounds %d\r\n", arg2);
            }                                
            break;
        }
        case AC_READ_SPEED_: {           
            pMsg->header.command            = CUTTER_READ_SPEED;
            pMsg->header.length             = READ_PROFILE_MSG_SIZE;
            if( arg1 <= MAX_MARK ) {                
                hexToAscii( (unsigned char *)&pMsg->body.profile.data, arg1 );
            } else {
                PRINTF( "buildCutterMsg(): AC_READ_SPEED_ mark argument is out of bounds %d\r\n", arg1);
            }                
            break;
        }
        case AC_RESET_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_RESET;
            pMsg->header.length             = DEFAULT_MSG_SIZE;
            break;
        }
        case AC_REQ_STATUS_: {
            pMsg->header.command            = CUTTER_STATUS;
            pMsg->header.length             = DEFAULT_MSG_SIZE;
            break;
        }
        case AC_TEST_MODE_: {
            pMsg->header.command            = CUTTER_TEST_MODE;
            pMsg->header.length             = DEFAULT_MSG_SIZE;                
            break;
        }
        case AC_VERSION_: {
            pMsg->header.command            = CUTTER_VERSION;
            pMsg->header.length             = READ_VERSION_MSG_SIZE;
            if( arg1 <= MAX_VERSION_INDEX ) {
                pMsg->body.readVersion.index = ( '30' + arg1 );
            } else {
                PRINTF( "buildCutterMsg(): AC_VERSION_ version index argument is out of bounds %d\r\n", arg1);
            }
            break;
        }
        default: {
            PRINTF( "buildCutterMsg(): Unknown command %d\r\n", cmd );
        }
    }
}

/******************************************************************************/
/*!   \fn static void prepTransmitBfr( unsigned char *pBfr, unsigned char length )
                                     
      \brief
        This function preps the transmit buffer to send to device by calculating 
        payload crc and appending ending message character EXT.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void prepTransmitBfr( unsigned char *pBfr, unsigned char length )
{
    unsigned short crc = calcCrc16( pBfr, length );
    /* offset to end of message */
    pBfr += length;
    unsigned char a = 0;
    a = (unsigned char)( ( crc & 0xff00 ) >> 8 );
    hexToAscii( pBfr++, a );
    pBfr++;
    a = (unsigned char)( crc & 0x00ff );
    hexToAscii( pBfr++, a );
    pBfr++;
    *pBfr = AC_ETX;
    
}

/******************************************************************************/
/*!   \fn static void hexToAscii( unsigned char *pChar, unsigned char data )
                                     
      \brief
        This function converts the data character into ascii and assigns to the
        character pointer.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void hexToAscii( unsigned char *pChar, unsigned char data )
{
    unsigned char nibble = ( ( data & 0xF0 ) >> 4 );
    int i = 0;
    while( i < 2 ) {
        if( nibble < 10 ) {
            *pChar++ = '0' + nibble; 
        } else if( nibble < 16 ) {
            *pChar++ = nibble + 'A' - 10;
        } else {
            *pChar++ = '?'; 
        }
        i++;
        nibble = ( data & 0x0F );
    }
}

/******************************************************************************/
/*!   \fn static void asciiToChar( unsigned char c, unsigned char *pChar )
                                     
      \brief
        This function converts the nibblized ascii to binary
       
      \author
          Aaron Swift
*******************************************************************************/                
static unsigned char asciiToChar( unsigned char *pChar )
{
    unsigned char result;
    if( *(pChar + 1) >= '0' && *(pChar + 1) <= '9' ) {
        result = *(pChar + 1) - 0x30;
    }
    else if( *(pChar + 1) >= 'A' && *(pChar + 1) <= 'F' ) {										 		
        result = *(pChar + 1) - 0x37;
    } else {
        result = 0;
    }
      
    if( *pChar >= '0' && *pChar <= '9' ) {
        result += ( ( *pChar - 0x30 ) << 4 );
    } else if( *pChar >= 'A' && *pChar <= 'F' ) {										 		
        result += ( ( *pChar - 0x37 ) << 4 );
    } else {
        result = 0;
    }
    return result;       
}

/******************************************************************************/
/*!   \fn static void handleCutterMessage( CutterMgr *pMgr, unsigned char *pBfr )
                                     
      \brief
        This function parses the received cutter message.
               
      \author
          Aaron Swift
*******************************************************************************/                
static void handleCutterMessage( CutterMgr *pMgr, unsigned char *pBfr )
{
#if 0
    PRINTF("\r\ncutterMsg() - ");
    
    for(uint16_t rxCounter = 0; rxCounter < 100; rxCounter++)
    {
      PRINTF("%02X ", uartRxBfr[rxCounter]);
    }
    
    
    PRINTF("\r\n");
    
    uint8_t *p = uartRxBfr;

    // STX 
    PRINTF("STX        : 0x%02X\r\n", p[0]);

    /* Slave ID */
    PRINTF("Slave ID   : %c\r\n", p[1]);

    /* Command */
    PRINTF("Command    : %c\r\n", p[2]);

    /* Type */
    PRINTF("Type       : %c (%s)\r\n",
           p[3],
           (p[3] == 'U') ? "Update" :
           (p[3] == 'Q') ? "Enquiry" : "Unknown");

    /* CR */
    PRINTF("CR         : %c (%s)\r\n",
           p[4],
           (p[4] == 'R') ? "Response" :
           (p[4] == 'C') ? "Command" : "Unknown");

    /* Status / Config word (ASCII hex) */
    char statusStr[5] = { p[5], p[6], p[7], p[8], 0 };
    uint16_t status = (uint16_t)strtol(statusStr, NULL, 16);

    PRINTF("Status     : %s (0x%04X)\r\n", statusStr, status);
    PRINTF("Status bits:\r\n");

    /* Bit 0 */
    PRINTF("  Bit 0  Power Fail                 : %s\r\n",
           (status & (1 << 0)) ? "SET" : "clear");

    /* Bit 1 */
    PRINTF("  Bit 1  Message Error              : %s\r\n",
           (status & (1 << 1)) ? "SET" : "clear");

    /* Bit 2 */
    PRINTF("  Bit 2  Reboot Flag                : %s\r\n",
           (status & (1 << 2)) ? "SET" : "clear");

    /* Bit 3 */
    PRINTF("  Bit 3  Blade Safe                 : %s\r\n",
           (status & (1 << 3)) ? "SET (blade home)" : "clear");

    /* Bit 4�6 (unused) */
    PRINTF("  Bit 4  Unused                     : %s\r\n",
           (status & (1 << 4)) ? "SET" : "clear");
    PRINTF("  Bit 5  Unused                     : %s\r\n",
           (status & (1 << 5)) ? "SET" : "clear");
    PRINTF("  Bit 6  Unused                     : %s\r\n",
           (status & (1 << 6)) ? "SET" : "clear");

    /* Bit 7 */
    PRINTF("  Bit 7  Timeout Error              : %s\r\n",
           (status & (1 << 7)) ? "SET (cut timeout)" : "clear");

    /* Bit 8�11 (unused) */
    PRINTF("  Bit 8  Unused                     : %s\r\n",
           (status & (1 << 8)) ? "SET" : "clear");
    PRINTF("  Bit 9  Unused                     : %s\r\n",
           (status & (1 << 9)) ? "SET" : "clear");
    PRINTF("  Bit 10 Unused                     : %s\r\n",
           (status & (1 << 10)) ? "SET" : "clear");
    PRINTF("  Bit 11 Unused                     : %s\r\n",
           (status & (1 << 11)) ? "SET" : "clear");

    /* Bit 12�13 (unused) */
    PRINTF("  Bit 12 Unused                     : %s\r\n",
           (status & (1 << 12)) ? "SET" : "clear");
    PRINTF("  Bit 13 Unused                     : %s\r\n",
           (status & (1 << 13)) ? "SET" : "clear");

    /* Bit 14 */
    PRINTF("  Bit 14 Door Open - backwards      : %s\r\n",
           (status & (1 << 14)) ? "SET (door closed)" : "clear");

    /* Bit 15 (unused) */
    PRINTF("  Bit 15 Saddle - backwards         : %s\r\n",
           (status & (1 << 15)) ? "SET (saddle closed)" : "clear");

    /* Length */
    char lenStr[5] = { p[9], p[10], p[11], p[12], 0 };
    uint16_t msgLen = (uint16_t)strtol(lenStr, NULL, 16);

    PRINTF("Length     : %s (%u bytes total)\r\n", lenStr, msgLen);

    /* Payload (if any) */
    uint16_t payloadLen = msgLen - 18; /* 13 header + 4 CRC + ETX */
    if (payloadLen > 0)
    {
        PRINTF("Payload    : ");
        for (uint16_t i = 0; i < payloadLen; i++)
        {
            uint8_t c = p[13 + i];
            PRINTF("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("Payload    : <none>\r\n");
    }

    /* CRC */
    uint16_t crcIndex = msgLen - 5;
    char crcStr[5] = {
        p[crcIndex],
        p[crcIndex + 1],
        p[crcIndex + 2],
        p[crcIndex + 3],
        0
    };

    PRINTF("CRC        : %s\r\n", crcStr);

    /* ETX */
    PRINTF("ETX        : 0x%02X\r\n", p[msgLen - 1]);

    PRINTF("-------------------------------------------\r\n");
#endif
  
    ACHeader header;   
    if( parseMsgHeader( &header, pBfr ) ){
       
        /* TFinkOyaneCutter parseMsgHeader returned true so cutter is attached */
       cutterDetected = true;
       
        /* index to the body of the message */
        pBfr += ( DEFAULT_MSG_SIZE_ + 1);
        switch( header.command )
        {
            
            case 'c': {
                //PRINTF( "handleCutterMessage(): Cut c message %d\r\n", header.length );
                actState_ =  AC_WAIT_FOR_COMMAND_;
                break;
            }
            case 'h': {
                //PRINTF( "handleCutterMessage(): Home h message %d\r\n", header.length );
                break;
            }
            case 'k': {
                //PRINTF( "handleCutterMessage(): Cut k message %d\r\n", header.length );
                break;
            }
            case 'p': {
                //PRINTF( "handleCutterMessage(): Profile message %d\r\n", header.length );
                break;
            }
            case 'r': {
                //PRINTF( "handleCutterMessage(): Reset message %d\r\n", header.length );
                break;
            }
            case 's': {
                //PRINTF( "\r\nhandleCutterMessage(): Status message %d\r\n", header.length ); 
                showDeviceStatus( pMgr->deviceStatus );
                actState_ =  AC_WAIT_FOR_COMMAND_;
                break;
            }
            case 't': {
                //PRINTF( "handleCutterMessage(): Test mode message %d\r\n", header.length ); 
                break;
            }
            case 'v': {
                //PRINTF( "handleCutterMessage(): Version message %d\r\n", header.length );  
                
                /* parse the version message based on message length */
                if( header.length == READ_VERSION0_RX_MSG_SIZE_ ) {
                    parseVerPoductMsg( pMgr, pBfr );
                    //showCutterVersion( pMgr );
                } else if( header.length == READ_VERSION1_RX_MSG_SIZE_ ) {
                    parseVerFirmNumbMsg( pMgr, pBfr );
                    //showCutterVersion( pMgr );
                } else if( header.length == READ_VERSION2_RX_MSG_SIZE_ ) {
                    parseVerIssueMsg( pMgr, pBfr );
                    //showCutterVersion( pMgr );
                } else if( header.length == READ_VERSION3_RX_MSG_SIZE_ ) {
                    parseVerDateMsg( pMgr, pBfr );
                    //showCutterVersion( pMgr );
                }else if( header.length == READ_VERSION4_RX_MSG_SIZE_ ) {
                    parseVerTimeMsg( pMgr, pBfr );
                    showCutterVersion( pMgr );
                }
                else {
                    //PRINTF( "handleCutterMessage(): Unknown version msg! %d\r\n", header.length );
                }
                break;
            }
            default: {
                PRINTF( "handleCutterMessage(): Unknown msg! %d\r\n", header.length );
            }

        }
    }
    
    /* clear our msg flag */
    pMgr->msgReady = false;
    
    /* finished with message, clear our message buffer */
    memset( &uartRxBfr[0], 0xff, MAX_MESSGAE_SIZE );
    memset( &uartTxBfr[0], 0x00, MAX_MESSGAE_SIZE );
}

/******************************************************************************/
/*!   \fn static bool parseMsgHeader( ACHeader *pHeader, unsigned char *pMsg )
                                     
      \brief
        This function parses the received message header.
               
      \author
          Aaron Swift
*******************************************************************************/                
static bool parseMsgHeader( ACHeader *pHeader, unsigned char *pMsg )
{   
    bool valid = false;
    pHeader->start      = *pMsg++; 
    
    unsigned char b[] = { 0, 0 };    
    b[0] = *pMsg++;
    pHeader->slaveId    = atoi( (const char *)&b[0] );   
    
    pHeader->command    = *pMsg++;    
    pHeader->type       = *pMsg++;    
    pHeader->cr         = *pMsg++;    
    
    
#ifdef TFinkOyaneCutter
    unsigned char c = *pMsg++ - '30'; 
    *pMsg++;
    *pMsg++;
    //unsigned char x = asciiToChar( pMsg++ );
    unsigned char x = *pMsg++ - '30';        
    *pMsg++;
    *pMsg++;
    
    pHeader->config = c;
    pHeader->config <<= 12;
    pHeader->config |= x;
#else
    unsigned char x;
    unsigned short i;
    pHeader->config = 0;
    
    for(i = 0; i < 4; i++)
    {
       x  = *pMsg++; 
       if(x > 0x40)
          x = x - 0x37;   /* handle 'A' through 'F' */
       else
          x = x - 0x30;   /* handle '0' through '9' */
       
       pHeader->config |= x;   /* note config (transmitted message) = status (received message) */
       if(i < 3)
         pHeader->config <<= 4;
    } 
    
    pHeader->config |= 0x8000;  /*TFinkOyaneCutterToDo! delete - test only */
#endif
    
    parseHeaderStatus( &cutterMgr_, pHeader );
    
    pHeader->length = (unsigned long)asciiToChar( pMsg++ );    
    
    if( ( pHeader->start == AC_STX ) && ( pHeader->slaveId == 1 ) && 
        (  pHeader->cr == 'R' ) ) {
        valid = true;
    }
    return valid;
}

/******************************************************************************/
/*!   \fn static void parseHeaderStatus( CutterMgr *pMgr, ACHeader *pHeader )
                                     
      \brief
        This function parses the status and updates the cutter manager.
               
      \author
          Aaron Swift
*******************************************************************************/                
static void parseHeaderStatus( CutterMgr *pMgr, ACHeader *pHeader )
{  /* TFinkOyaneCutterToDo This code sets pMgr-error to the first error it
      encounters. Is the purpose to prioritize failures? So the most important 
      failure is handled first? */
    /* set any errors recorded in the status */
    if( ( pHeader->config & AC_POWER_FAILURE ) == AC_POWER_FAILURE ) {
        pMgr->error = AC_ERR_POWER_FAILURE_; 
        PRINTF("\r\nAC_POWER_FAILURE");
    } else {
        if( pMgr->error == AC_ERR_POWER_FAILURE_ )
            pMgr->error = AC_ERR_NONE_;
    }
    
    if( ( pHeader->config & AC_MSG_FAILURE ) == AC_MSG_FAILURE ) {
        PRINTF("\r\nAC_MSG_FAILURE");
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_MSG_FAILURE_;
    } else {
        if( pMgr->error == AC_ERR_MSG_FAILURE_ )
            pMgr->error = AC_ERR_NONE_;    
    }
    
    /*
    if( ( pHeader->config & AC_BLADE_HOME ) != AC_BLADE_HOME ) {
        //PRINTF("\r\n3");
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_BLADE_NOT_HOME_;
    } else {
        if( pMgr->error == AC_ERR_BLADE_NOT_HOME_ )
            pMgr->error = AC_ERR_NONE_;        
    }
    */
    
    if( ( pHeader->config & AC_TIME_OUT_ERROR ) == AC_TIME_OUT_ERROR ) { 
        PRINTF("\r\nAC_TIME_OUT_ERROR");
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_TIMEOUT_;
    } else {
        if( pMgr->error == AC_ERR_TIMEOUT_ )
            pMgr->error = AC_ERR_NONE_;            
    }
    
    /* TFinkToDoOyaneCutter  As of 6/6/25, door closed is a '1', which is opposite of the original cutter and 
            is opposite of AC_DOOR_OPEN */
    /*
    if( ( pHeader->config & AC_DOOR_OPEN ) == AC_DOOR_OPEN ) {
        //PRINTF("\r\n5");
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_DOOR_OPEN_;
    } else {
        if( pMgr->error == AC_ERR_DOOR_OPEN_ )
            pMgr->error = AC_ERR_NONE_;                
    }
    
    if( ( pHeader->config & AC_SADDLE_OPEN ) == AC_SADDLE_OPEN ) {
        //PRINTF("\r\n6");
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_SADDLE_OPEN_;
    } else {
        if( pMgr->error == AC_ERR_SADDLE_OPEN_ )
            pMgr->error = AC_ERR_NONE_;                
    }
    */
    
    pMgr->deviceStatus = (unsigned short)pHeader->config;
    
    /* if we have errors then show */
    if( pMgr->error != AC_ERR_NONE_ ) 
    {  
        PRINTF("parseHeaderStatus(): CUTTER ERROR\r\n");
               
        //showDeviceStatus( pMgr->deviceStatus );
        
        openCutterInterface( CUTTER_UART_BAUD );
        
        //pMgr->error = AC_ERR_NONE_;
        
        vTaskDelay( lightSleep );
        //vTaskDelay( deepSleep );
    } 
    else 
    {
        //PRINTF("Cutter status:  ready\r\n");    
    }
    
    /* determine our platten type */
    if( ( pHeader->config & PLATTEN_FULL_TYPE ) == PLATTEN_FULL_TYPE ) {
        pMgr->platten = FULL_PLATTEN_;
    } else {
        pMgr->platten = SLOTTED_PLATTEN_;
    }    
}

/******************************************************************************/
/*!   \fn static void parseVerPoductMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the product portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerPoductMsg( CutterMgr *pMgr, unsigned char *pMsg )
{    
    memcpy( &pMgr->version.productName[0], pMsg, MAX_PRODUCT_NAME_SIZE );    
}

/******************************************************************************/
/*!   \fn static void parseVerFirmNumbMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the firmware number portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerFirmNumbMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.firmware[0], pMsg, MAX_FIRMWARE_NUM_SIZE );     
}

/******************************************************************************/
/*!   \fn static void parseVerIssueMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the issue portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerIssueMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.issueDate[0], pMsg, MAX_FIRMWARE_ISSUE_SIZE );     
}

/******************************************************************************/
/*!   \fn static void parseVerDateMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the date portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerDateMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.date[0], pMsg, MAX_DATE_SIZE );     
}
                              
/******************************************************************************/
/*!   \fn static void parseVerTimeMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the time portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Chris King
*******************************************************************************/                        
static void parseVerTimeMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.time[0], pMsg, MAX_TIME_SIZE );     
}
        
/******************************************************************************/
/*!   \fn static void showDeviceStatus( unsigned short status )

      \brief
        This function parses and prints the status bits from the cutter. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static void showDeviceStatus( unsigned short status )
{
    if(status != 0x3008)
    {
        PRINTF("showDeviceStatus(): cutter status 0x%02x\r\n", status );
    }
    
    //PRINTF("showDeviceStatus(): cutter status 0x%02x\r\n", status );
    
    //PRINTF("\r\nAC %d", pMsgQHandle_);
    
    /*
    if( ( status & AC_POWER_FAILURE ) == AC_POWER_FAILURE ) 
    {
        PRINTF("Cutter Error: ac power failure\r\n");
    }
    
    if( ( status & AC_MSG_FAILURE ) == AC_MSG_FAILURE )
    {
        PRINTF("Cutter Error: message failure\r\n");
    }
    
    if( ( status & AC_REBOOT_FLAG ) == AC_REBOOT_FLAG ) 
    {
        PRINTF("Cutter Warning: boot flag\r\n");
    }
    
    if( ( status & AC_BLADE_HOME ) == AC_BLADE_HOME )
    {
        PRINTF("Cutter Blade: is home\r\n");
    }
    else
    {
        PRINTF("Cutter Blade: is NOT home\r\n");
    }
    
    if( ( status & AC_TIME_OUT_ERROR ) == AC_TIME_OUT_ERROR )
    {
        PRINTF("Cutter Error: cutter timeout\r\n");
    }
            
    if( ( status & AC_DOOR_OPEN ) != AC_DOOR_OPEN ) 
    {   //TFinkOyaneCutterToDo - ask rob to change bit to 1 when door is open 
        PRINTF("Cutter door: door is closed\r\n");
    } 
    else
    {
        PRINTF("Cutter Error: door is open\r\n");
    }
    
    if( ( status & AC_SADDLE_OPEN ) != AC_SADDLE_OPEN ) 
    {    // TFinkOyaneCutterToDo - ask rob to change bit to 1 when door is open 
        PRINTF("Cutter saddle: saddle is closed\r\n");
    } 
    else
    {
        PRINTF("Cutter Error: saddle is open\r\n");
    }
    */
    
    getCutterHome();
    getCutterSaddleInterlock();
    getCutterDoorInterlock();
    getCutterJammed();
}

/******************************************************************************/
/*!   \fn static void showDeviceStatus( unsigned short status )

      \brief
        This function prints the cutter version strings. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static void showCutterVersion( CutterMgr *pMgr )
{
    PRINTF("Cutter product name: %s\r\n", &(pMgr->version.productName[0]) );    
    PRINTF("Cutter model: %s\r\n", &(pMgr->version.firmware[0]) );    
    PRINTF("Cutter firmware version: %s\r\n", &(pMgr->version.issueDate[0]) );    
    PRINTF("Cutter issue date: %s\r\n", &(pMgr->version.date[0]) ); 
    PRINTF("Cutter issue time: %s\r\n", &(pMgr->version.time[0]) );
}

/******************************************************************************/
/*!   \fn static bool openCutterInterface( unsigned short baud ) 

      \brief
        This function opens the uart interface to the cutter. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static bool openCutterInterface( unsigned short baud )
{
    lpuart_config_t     config;    
    bool                status = false;
    
    LPUART_GetDefaultConfig( &config );
    config.baudRate_Bps         = baud;
    config.dataBitsCount        = kLPUART_EightDataBits;
    config.parityMode           = kLPUART_ParityEven;
    config.stopBitCount         = kLPUART_OneStopBit;
    config.enableTx             = true;
    config.enableRx             = true;

    long initStatus = LPUART_Init( LPUART1, &config, BOARD_DebugConsoleSrcFreq() );
    if( initStatus == kStatus_Success ) {
        LPUART_TransferCreateHandle( LPUART1, &cutterHandle_, uart1Callback, NULL );
        status = true;
    } else {
        PRINTF("openCutterInterface(): Could not open interface!\r\n" );   
    }
    NVIC_SetPriority( LPUART1_IRQn, 2 );
    
    //#define UART_RX_BFR_SIZE 128
    
    //LPUART_TransferStartRingBuffer(LPUART1, &cutterHandle_, uartRxBfr, UART_RX_BFR_SIZE);
    
    return status;        
}

/******************************************************************************/
/*!   \fn bool isCutterInterlockClosed( void )

        \brief
        This function returns true if the label cutter interlock is closed.

        \author
        Aaron Swift
*******************************************************************************/
bool isCutterInterlockClosed( void )
{  
    return ( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN ) != AC_DOOR_OPEN ) );
}

/******************************************************************************/
/*!   \fn static void uart1Callback( LPUART_Type *base, lpuart_handle_t *pHandle, 
                                     status_t status, void *pData )

      \brief
        This function is the callback for uart2. 
       
      \author
          Aaron Swift
*******************************************************************************/                
void uart1Callback( LPUART_Type *base, lpuart_handle_t *pHandle, 
                           status_t status, void *pData )
{
    if ( status == kStatus_LPUART_TxIdle ) {
        BaseType_t reschedule;
        
        cutterMgr_.txReady = true;
        memset( (void *)&uartTxBfr[0], 0, MAX_MESSGAE_SIZE );
        xSemaphoreGiveFromISR( cMutex_, &reschedule );
        
        portYIELD_FROM_ISR( reschedule );
    } 
    if (status == kStatus_LPUART_RxIdle)
    {
        cutterMgr_.msgReady = true;
    }
    else if( ( status != kStatus_LPUART_TxBusy ) || 
              ( status != kStatus_LPUART_RxBusy ) ) {
#if 1                  
        if( status == kLPUART_RxDataRegFullFlag )
        {
            PRINTF("uart1Callback() kLPUART_RxDataRegFullFlag : Error: %d!\r\n", status );
        }        
                  
        if( status == kStatus_LPUART_Error  )
        {
            PRINTF("uart1Callback(): Error: %d!\r\n", status );
        }
          
        if( status == kStatus_LPUART_RxRingBufferOverrun  )
        {
            PRINTF("uart1Callback(): Error: %d!\r\n", status );
        }
        
        if( status == kStatus_LPUART_RxHardwareOverrun  )
        {
            PRINTF("uart1Callback(): kStatus_LPUART_RxHardwareOverrun: %d!\r\n", status );
            PRINTF("\r\n\r\nRESET CUTTER INTERFACE\r\n\r\n");
            openCutterInterface( CUTTER_UART_BAUD );
            
            if( actState_ == AC_CUT_ || actState_ == AC_PROCESS_CUT_ || actState_ == AC_PROCESS_CUT_RESPONSE_)
            {
                cutPending = true;
                actState_ = AC_WAIT_FOR_COMMAND_;
            }
            else if( actState_ == AC_TX_DEVICE_STATUS_ || actState_ == AC_RX_DEVICE_STATUS_ )
            {
                //statusPending = true;
                actState_ = AC_WAIT_FOR_COMMAND_;
            }
        }
        
        if( status == kStatus_LPUART_NoiseError  )
            PRINTF("uart1Callback(): kStatus_LPUART_NoiseError: %d!\r\n", status );            
        if( status == kStatus_LPUART_FramingError  )
            PRINTF("uart1Callback(): kStatus_LPUART_FramingError: %d!\r\n", status );            
        if( status == kStatus_LPUART_ParityError  )
            PRINTF("uart1Callback(): kStatus_LPUART_ParityError: %d!\r\n", status );            
        if( status == kStatus_LPUART_Timeout  )
            PRINTF("uart1Callback(): kStatus_LPUART_Timeout: %d!\r\n", status );                        
#endif    
      }
}

/******************************************************************************/
/*!   \fn static unsigned short calcCrc16( unsigned char *pBlob, unsigned short length )

      \brief
        This function calculates the message 16 bit crc value.
        The value is calculated starting after the STX and excluding the 
        CRC and ETX fields.
       
      \author
          Aaron Swift
*******************************************************************************/                
static unsigned short calcCrc16( unsigned char *pBlob, unsigned short length )
{
    unsigned short checkSum = 0;
    unsigned char index = 0;
    while( length-- ) {              
        /* reverse order checksum */
        checkSum = ( checkSum >> 8 )^crc16[ (unsigned char)( checkSum^ *pBlob++ ) ];    
    }
    return checkSum;       
}

static uint16_t asciiHexToU16(char *ascii)
{
    uint16_t value = 0;
    for(int i = 0; i < 4; i++)
    {
        value <<= 4;
        if(ascii[i] >= '0' && ascii[i] <= '9')
            value |= (ascii[i] - '0');
        else if(ascii[i] >= 'A' && ascii[i] <= 'F')
            value |= (ascii[i] - 'A' + 10);
    }
    return value;
}

bool getCutterHome( void )
{
    //PRINTF("\r\ngetCutterHome() %d", ( ( cutterMgr_.deviceStatus & AC_BLADE_HOME ) == AC_BLADE_HOME ));
  
    return ( ( ( cutterMgr_.deviceStatus & AC_BLADE_HOME ) == AC_BLADE_HOME ) );
}
                                  
bool getCutterSaddleInterlock( void )
{
    //PRINTF("\r\ncutter status 0x%02x\r\n", cutterMgr_.deviceStatus );
  
    if(getCutterHome() == true)
    {
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_OPEN )
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_INTERLOCKS_OPEN 0x%02x", AC_INTERLOCKS_OPEN);
            return false;
        }
        
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_CLOSED )
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_INTERLOCKS_CLOSED 0x%02x", AC_INTERLOCKS_CLOSED);
            return true;
        }
       
        if(( ( ( cutterMgr_.deviceStatus & AC_SADDLE_OPEN_DOOR_CLOSED ) == AC_SADDLE_OPEN_DOOR_CLOSED ) ))
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_SADDLE_OPEN_DOOR_CLOSED");
            return false;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN_SADDLE_CLOSED) == AC_DOOR_OPEN_SADDLE_CLOSED ) ))
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_DOOR_OPEN_SADDLE_CLOSED");
            return true;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_SADDLE_OPEN ) == AC_SADDLE_OPEN ) ))
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_SADDLE_OPEN");
            return false;
        }
    }
    else
    {
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_OPEN_BNH )
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_INTERLOCKS_OPEN 0x%02x", AC_INTERLOCKS_OPEN_BNH );
            return false;
        }
        
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_CLOSED_BNH )
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_INTERLOCKS_CLOSED 0x%02x", AC_INTERLOCKS_CLOSED_BNH );
            return true;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_SADDLE_OPEN_DOOR_CLOSED_BNH ) == AC_SADDLE_OPEN_DOOR_CLOSED_BNH ) ))
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_SADDLE_OPEN_DOOR_CLOSED");
            return false;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN_SADDLE_CLOSED_BNH ) == AC_DOOR_OPEN_SADDLE_CLOSED_BNH ) ))
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_DOOR_OPEN_SADDLE_CLOSED");
            return true;
        } 
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN ) == AC_DOOR_OPEN ) ))
        {
            //PRINTF("\r\ngetCutterSaddleInterlock() AC_DOOR_OPEN");
            return false;
        }
    }
    
    return true;
}

bool getCutterDoorInterlock( void )
{
    //PRINTF("\r\ncutter status 0x%02x\r\n", cutterMgr_.deviceStatus );
  
    if(getCutterHome() == true)
    {
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_OPEN )
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_INTERLOCKS_OPEN 0x%02x", AC_INTERLOCKS_OPEN );
            return false;
        }
        
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_CLOSED )
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_INTERLOCKS_CLOSED 0x%02x", AC_INTERLOCKS_CLOSED );
            return true;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_SADDLE_OPEN_DOOR_CLOSED ) == AC_SADDLE_OPEN_DOOR_CLOSED ) ))
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_SADDLE_OPEN_DOOR_CLOSED");
            return true;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN_SADDLE_CLOSED ) == AC_DOOR_OPEN_SADDLE_CLOSED ) ))
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_DOOR_OPEN_SADDLE_CLOSED");
            return false;
        } 
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN ) == AC_DOOR_OPEN ) ))
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_DOOR_OPEN");
            return false;
        }
    }
    else
    {
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_OPEN_BNH )
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_INTERLOCKS_OPEN 0x%02x", AC_INTERLOCKS_OPEN_BNH );
            return false;
        }
        
        if( cutterMgr_.deviceStatus == AC_INTERLOCKS_CLOSED_BNH )
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_INTERLOCKS_CLOSED 0x%02x", AC_INTERLOCKS_CLOSED_BNH );
            return true;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_SADDLE_OPEN_DOOR_CLOSED_BNH ) == AC_SADDLE_OPEN_DOOR_CLOSED_BNH ) ))
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_SADDLE_OPEN_DOOR_CLOSED");
            return true;
        }
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN_SADDLE_CLOSED_BNH ) == AC_DOOR_OPEN_SADDLE_CLOSED_BNH ) ))
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_DOOR_OPEN_SADDLE_CLOSED");
            return false;
        } 
        
        if(( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN ) == AC_DOOR_OPEN ) ))
        {
            //PRINTF("\r\ngetCutterDoorInterlock() AC_DOOR_OPEN");
            return false;
        }
    }
    
    return true;
}

bool getCutterJammed( void )
{
    return false;
}

bool getCutterDetected( void )
{
    return cutterDetected;
}

uint8_t getCutterState( void )
{
    return actState_;
}

QueueHandle_t getCutterQHandle( void )
{
    return pMsgQHandle_;
}
    