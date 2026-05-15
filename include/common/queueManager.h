#ifndef QUEUEMANAGER_H
#define QUEUEMANAGER_H
#include "FreeRTOS.h"
#include "queue.h"
#include "threadManager.h"
#include "hobartWeigherMessages.h"
#include "hobartPrinterMessages.h"
#include "internalMessages.h"
#include "deviceProperties.h"


/* USB IN queue dimensions */
#define MAX_USB_IN_LENGTH       90
#define MAX_USB_IN_DEPTH        64

/* USB OUT queue dimensions */
#define MAX_USB_OUT_LENGTH      30
#define MAX_USB_OUT_DEPTH       64

/* USB OUT BLK queue dimensions */
#define MAX_USB_BLK_OUT_LENGTH  2
#define MAX_USB_BLK_OUT_DEPTH   512

/* Thread Manager queue dimensions */
#define MAX_TM_MSG_LENGTH       20
#define MAX_TM_MSG_DEPTH        sizeof(ManagerMsg)

/* both weigher and printer max size for avery is 64 bytes */
#define MAX_AVERY_W_MSG_LENGTH  64      
#define MAX_AVERY_P_MSG_LENGTH  64

/* Weigher queue dimensions */
#define MAX_WG_MSG_LENGTH       20
#define MAX_WG_MSG_DEPTH        sizeof(WgMessage)

/* Avery Weigher queue dimensions */
#define MAX_AWG_MSG_LENGTH       20      /* TO DO: */
#define MAX_AWG_MSG_DEPTH        MAX_AVERY_W_MSG_LENGTH

/* Weigher counts queue dimensions */
#define MAX_WG_CNT_LENGTH       20
#define MAX_WG_CNT_DEPTH        sizeof(unsigned long)

/*printer queue dimensions */
#define MAX_PR_MSG_LENGTH       20
#define MAX_PR_MSG_DEPTH        sizeof(PrMessage)

/* Avery Printer queue dimensions */
#define MAX_APR_MSG_LENGTH       20
#define MAX_APR_MSG_DEPTH        MAX_AVERY_P_MSG_LENGTH

/*printer cmd queue dimensions */
#define MAX_PR_CMD_LENGTH       50
#define MAX_PR_CMD_DEPTH        sizeof(PrCommand)

/* valueMax transaction queue dimensions */
#define MAX_VM_QUEUE_LENGTH     20
#define MAX_VM_QUEUE_DEPTH        sizeof(valueMaxTransactionID)

/* internal task message queue dimensions */
#define MAX_CR_MSG_LENGTH       5                       /* Cutter */
#define MAX_CR_MSG_DEPTH        sizeof(ICMessages)

#define MAX_IPR_MSG_LENGTH      5                      /* Printer */
#define MAX_IPR_MSG_DEPTH       sizeof(ICMessages)

/* internal task message queue dimensions */
#define MAX_MM_MSG_LENGTH       5                      /* Motor Manager */ 
#define MAX_MM_MSG_DEPTH        sizeof(IMMessages)

/* dot wear queue dimensions */
#define MAX_DW_MSG_LENGTH       20
#define MAX_DW_MSG_DEPTH        sizeof(PrMessage)

/* test queue dimensions */
#define MAX_UT_MSG_LENGTH       20
#define MAX_UT_MSG_DEPTH        sizeof(WgMessage)

#if 0 /* TO DO: add when ready */
/*****************************************************************************/
/* boot upgrader queue dimensions */
#define MAX_BOOT_UPGRADER_MSG_LENGTH 20
#define MAX_BOOT_UPGRADER_MSG_DEPTH sizeof(UpgraderMessage)
/*****************************************************************************/
#endif

/*************************** common public functions **************************/
/******************************************************************************/
void initializeQueueManager( PeripheralModel_t model ); 
QueueHandle_t getUsbInPrQueueHandle( void );
QueueHandle_t getUsbInWrQueueHandle( void );
QueueHandle_t getUsbOutPrQueueHandle( void );
QueueHandle_t getUsbOutWrQueueHandle( void );

QueueHandle_t getWeigherQueueHandle( void );
QueueHandle_t getThreadManagerQueueHandle( void );
QueueHandle_t getWeigherCntsQueueHandle( void );
QueueHandle_t getPrinterQueueHandle( void );
QueueHandle_t getCutterQueueHandle( void );
QueueHandle_t getInternalPrinterQueueHandle( void );
QueueHandle_t getVMQueueHandle( void );
QueueHandle_t getMMQueueHandle( void );

bool addThreadManagerMsg( unsigned char * pMsg );

int getUsbInQueueLength( void );
int getUsbOutQueueLength( void );
int getPrinterMsgQueueLength( void );
int getWeigherMsgQueueLength( void );
int getWeigherCntsQueueLength( void );
bool addThreadManagerMsg( unsigned char * pMsg );

unsigned int getQueueDepth( QueueHandle_t handle_ );
unsigned int getQueueSize( QueueHandle_t handle_ );
/******************************************************************************/
/******************************************************************************/




/*************************** avery public functions ***************************/
/******************************************************************************/
QueueHandle_t getAveryWeigherQueueHandle( void );
QueueHandle_t getAveryPrinterQueueHandle( void );
QueueHandle_t getCutterQueueHandle( void );
/******************************************************************************/
/******************************************************************************/



/************************** hobart's public functions *************************/
/******************************************************************************/
QueueHandle_t getWeigherQueueHandle( void );
QueueHandle_t getWeigherCntsQueueHandle( void );
QueueHandle_t getPrinterQueueHandle( void );
QueueHandle_t getInternalPrinterQueueHandle( void );
QueueHandle_t getVMQueueHandle( void );
QueueHandle_t getMMQueueHandle( void );
QueueHandle_t getPrCommandQueueHandle( void );
/******************************************************************************/
/******************************************************************************/
#endif
