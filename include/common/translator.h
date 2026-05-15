#ifndef TRANSLATOR_H
#define TRANSLATOR_H
#include "deviceProperties.h"
#include "hobartWeigherMessages.h"
#include "hobartPrinterMessages.h"
#include "averyProtocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>

/***************************** common defines *********************************/
/******************************************************************************/
#define translator_task_PRIORITY ( configMAX_PRIORITIES -1 )

#define DOT_CHECKER_TOTAL_DOTS          640
#define DOT_CHECKER_DOTS_PER_PACKET     56
#define DOT_CHECKER_FINAL_PACKET_DOTS   24

typedef struct
{
    unsigned int msgSize; 
    bool handlingMsg;
}TransMSG;
/******************************************************************************/
/******************************************************************************/




/****************************** avery defines *********************************/
/******************************************************************************/
#if 0
/* packet structure */
typedef struct
{
	unsigned char stx;
	unsigned char cmd;
	unsigned short blkNum;
	unsigned short numBlks;
	unsigned char pad;
	unsigned char len;
}AVHeader;

#define PACKET_DATA_SIZE	( MAX_PACKET_SIZE - sizeof( AVHeader ) )
#endif

#define MAX_PACKET_SIZE		64      /* our endpoint size */
/* message control characters */
#define T_STX                    0x02
#define T_WSTX                   0x62
#define T_ETX                    0x03
#define T_LF                     0x0a
#define T_CR                     0x0d


/* avery endpoint sizes */
#define MAX_AV_USB_FRAME_SIZE                   16

#if 0
/* production test structures */
typedef struct
{
	unsigned char portNum;
	unsigned char pinNum;
	unsigned char state;
}ProdTestPortPin;

typedef struct
{
	unsigned char channel;
}ProdTestADC;

typedef struct
{
	unsigned char channel;
	unsigned char value;
}ProdTestLedDrive;

typedef struct
{
	unsigned long len;
	unsigned char dirn;
	unsigned char motorNum;
}ProdTestMotorDrive;

typedef struct
{
	unsigned long check;
}ProdTestReset;

typedef struct
{
	unsigned short protocol;
	unsigned char  numBytes;        /* number of bytes of extra status expect by main board */
}ExtraStatus;


/* the union for all the different packets */
typedef union
{
	ProdTestPortPin		ptPortPin;
	ProdTestADC		ptAdc;
	ProdTestLedDrive	ptLedDrive;
	ProdTestMotorDrive	ptMotorDrive;
	ProdTestReset		ptReset;
	ExtraStatus		ptExtraStatus;
	unsigned char		data[PACKET_DATA_SIZE];
}PacketData;

// first and last block packet struct
typedef struct
{
    /* header information */
    unsigned char  stx;
    unsigned char  cmd;
    unsigned char  pad;
    unsigned char  len;
    unsigned char  data[PACKET_DATA_SIZE];
    unsigned short blkNum;
    unsigned short numBlks;

    ProdTestPortPin    ptPortPin;
    ProdTestADC        ptAdc;
    ProdTestLedDrive   ptLedDrive;
    ProdTestMotorDrive ptMotorDrive;
    ProdTestReset      ptReset;
    ExtraStatus        ptExtraStatus;
}Packet_t;

#define SIZE_IN_NIBBLES(a)		( a << 1 )

typedef struct _BINARYHEADER_TAG
{
        unsigned short messageSize;	
        unsigned short cvn;		
        unsigned char slaveID;		
        unsigned short cmd;		
        char action;			
        char type;			
        unsigned short status;		
        unsigned short sequenceNum;	
}BinaryHeader_t;
#pragma pack (push, onthewire, 1)
typedef struct _ASCIIHEADER_TAG
{
    unsigned char messageSize[SIZE_IN_NIBBLES(sizeof(((BinaryHeader*)0)->messageSize ))];	/* size of the message block including the header */
    unsigned char cvn[SIZE_IN_NIBBLES(sizeof( ((BinaryHeader*)0)->cvn ))];		        /* command version number */
    unsigned char slaveID[SIZE_IN_NIBBLES(sizeof(((BinaryHeader*)0)->slaveID))];		/* id of sending device */
    unsigned char cmd[SIZE_IN_NIBBLES(sizeof(((BinaryHeader*)0)->cmd))];		/* value describing the command */
    char action;								                /* What action to take ('U', 'Q', 'X') */
    char type;									                /* the type of message ('C', 'R', 'A') */
    unsigned char status[SIZE_IN_NIBBLES(sizeof(((BinaryHeader*)0)->status))];			/* status of sending device */
    unsigned char sequenceNum[SIZE_IN_NIBBLES(sizeof(((BinaryHeader*)0)->sequenceNum))];	/* self sequencing message counter */
}AsciiHeader; 
#pragma pack (pop, onthewire)
#endif
/******************************************************************************/
/******************************************************************************/




/****************************** hobart defines *********************************/
/******************************************************************************/
/* this list must be same as in backend */
typedef enum
{
    UNKNOWNTYPE_,
    G_WEIGHER_,
    G_PRINTER_,
    G_DBOOT_,
    G_WRAPPER_,
    G_CLA_           
}HDeviceType;

/* LEGACY_PROTOCOL describes the USB messaging communication scheme which is made up of a header packet followed by a data packet. 
   This is the messaging format used on legacy peripherals, Atmel based.*/
#define LEGACY_PROTOCOL                         1       
/* GLOBAL_PROTOCOL omits the use of header packet and only sends data packet */
/* #define GLOBAL_PROTOCOL                      1 */


#pragma pack( 1 )
#ifdef GLOBAL_PROTOCOL
typedef struct
{
    HDeviceType         device;
    unsigned char       stationId;              /* unique system id (ex. first printer, second printer ) */
    unsigned char       msgType;
    unsigned long       msgSize;                /* message body size */
    unsigned char       frameNumber;            /* current frame number */
    unsigned char       numberOfFrames;         /* total frames needed to complete the message */
    unsigned short      lastFrameSize;          /* total bytes in the last frame */
}USBHeader;

typedef struct
{
    USBHeader hdr;    
    unsigned char *pMsg;
}HUSBMessage;

#else 
typedef struct
{
    unsigned short       sourcePhysAddr;         /* unused */
    unsigned short       sourceLogicalAddr;
    unsigned short       DestPhysAddr;           /* unused */
    unsigned short       DestLogicalAddr;
    unsigned short       msgSize;        
}USBHeader;

typedef union
{
    PrMessage prMsg;
    WgMessage wgMsg;    
}HBMessage;

typedef struct
{
    USBHeader hdr;    
    unsigned char body[ sizeof(HBMessage) ];
}HUSBMessage;

#define LGCY_LOGICAL_ADDR_PRNTR                 0x24
#define LGCY_LOGICAL_ADDR_GPRNTR                0x25
#define LGCY_LOGICAL_ADDR_WGR                   0x06
#define LGCY_LOGICAL_ADDR_GWGR                  0x66
#endif
#pragma pack( push,1 )
#pragma pack( pop )

#define MAX_USB_INT_FRAME_SIZE                  64
#define MAX_USB_FRAME_SIZE                      512     /* this must be the same size of the pipe! */


#define HALF_SECOND                             100     /* systicks */

typedef struct
{
    PrStatField         field;                          /* status field to be tracked */
    unsigned long       time;                           /* time status message was sent to host */
}PrStatTracker;

/*************************** common public functions **************************/
/******************************************************************************/
BaseType_t createTranslator( DEVICESUBCLASS_t sClass,
                             QueueHandle_t wgMsgQueue, 
                             QueueHandle_t prMsgQueue, 
                             QueueHandle_t UsbInPrMsgQueue,
                             QueueHandle_t UsbInWrMsgQueue,
                             QueueHandle_t UsbOutPrMsgQueue, 
                             QueueHandle_t UsbOutWrMsgQueue);

TaskHandle_t getTranslatorHandle( void );
/******************************************************************************/
/******************************************************************************/




/************************** hobart's public functions *************************/
/******************************************************************************/

/* printer messages */
void sendPrWakeup( PrWakeup *pWakeMsg);
void sendFakePrWakeup( void );
void sendPrSysInfo( PrSysInfo *pSysInfo );
void sendPrConfig( Pr_Config *pConfig );
void sendPrStatus( PrStatusInfo *pStatus, bool interrupt );
void sendPrSensors( PrSensors *pSensors );
void sendPrVersion( PrVersion *pVersion );
void sendPrHeadType( PrHead *pType );
void sendPrCutterStatus( PrCutterStatus *pStatus );
BaseType_t sendPrCutterStatusFromISR( PrCutterStatus *pStatus );
void sendPrTransferReady( bool ready );
void sendPrFactoryDlftsComplete( bool status );
void sendPrHeadDotStatus( int size );
void sendPrGapCalStatus( PrGapCalStatus *pPrGapCalStatus );
void sendPrPeelLog(PrPeelLog *pPrPeelLog);
void sendPrTUCalStatus( PrGapCalStatus *pPrGapCalStatus );
void sendLowLabelMinMax( short minPeeling, short maxPeeling, short minStreaming, short maxStreaming );
void sendTakeLabelError( bool error );
void sendPrHeadCalResponse( PrHeadCalResponse* response );
void sendCutterVersion( PrCutterVersion *pVer );
void sendPrGlobalCutterStatus( PrGblCutterStatus *pStatus );
void sendPrTUCalStatusFromISR( PrGapCalStatus *pPrGapCalStatus );

/* avery printer messages */
void sendAVRespByte( bool result );
void sendAVData( unsigned char *pBfr, unsigned short length  );

/* weigher messages */
void sendWgWakeup( WgWakeup *pWakeMsg );
void sendWgSysInfo( WgSysInfo *pSysInfo );
void sendWgStatus( WgStatus *pStatusMsg );
void sendWgConfig( WgCfg *pConfigMsg );
void sendWgSystemInfo( WgInfo *pInfo );
void sendWgVersion( WgVersion *pVersion );
void sendCat3Statistics( WgCat3Statistics *pStatsMsg );
void sendCat3Records( WgCat3Records *pRecordsMsg );
void sendCat3RecordStatus( WgCat3RecordStatus *pStatusMsg );
void sendCat3LogEraseComplete( void );
void sendWgVmAssembly( WgVMWrAssembly *pVmAssembly );
void sendWgVmVersion( WgVMWrVersion *pVmVersion );
void sendWgVmSerial( WgVMWrSerial *pVmSerial );
void sendWgVmDate( WgVMWrDate *pVmDate );
void sendWgVmXGain( WgVMWrOffsetGain *pXFactor );
void sendWgVmYGain( WgVMWrOffsetGain *pYFactor );
void sendWgVmZGain( WgVMWrOffsetGain *pZFactor );
void sendWgVmXZeroRef( float *zero_ref );
void sendWgVmYZeroRef( float *zero_ref );
void sendWgVmZZeroRef( float *zero_ref );
void sendWgCurrentGains( WgVMCurrent *pCurrent );
void sendWgFactoryDlftsComplete( bool status );

void initPrHeader( unsigned char *pHdr, unsigned short size );
void initWrHeader( unsigned char *pHdr, unsigned short size );
/******************************************************************************/
/******************************************************************************/

/*************************** common private functions **************************/
/******************************************************************************/
static void translatorTask( void *pvParameters );
static void translateUNPrinterMessage( unsigned char *pBfr );
static void translateUNWeigherMessage( unsigned char *pBfr );
/******************************************************************************/
/******************************************************************************/




/************************** hobart's private functions *************************/
/******************************************************************************/
static void translateHBPrinterMessage( unsigned char *pBfr );
static void translateHBWeigherMessage( unsigned char *pBfr );
static unsigned int translateFrameHdr( unsigned char *pBfr, USBHeader *pHdr );
static bool isWeigherMessage( HDeviceType type );
static bool isPrinterMessage( HDeviceType type );

#ifdef LEGACY_PROTOCOL
static void t_handleWeigherMessage( unsigned char *pMsg_, unsigned int msgSize );
static void t_handlePrinterMessage( unsigned char *pMsg_, unsigned int msgSize );
#else
static void t_handleWeigherMessage( HUSBMessage *pMsg_ );
static void t_handlePrinterMessage( HUSBMessage *pMsg_ );
#endif

/* handle printer messages */
static void t_handlePrReqWakeup( void );
static void t_handlePrWriteConfig( unsigned char *pFrame );
static void t_handlePrReqConfig( void );
static void t_handlePrReqStatus( void );
static void t_handlePrReqSensors( void );
static void t_handlePrReqVersion( void );
static void t_handlePrReqCalibration( void );
static void t_handlePrReqHeadType( void );
static void t_handlePrMode( unsigned char mode );
static void t_handlePrDisable( void );
static void t_handlePrEnable( void );
static void t_handlePrReset( void );
static void t_handlePrEnableTakeup( void );
static void t_handlePrCommand( unsigned char *pFrame );
static void t_handlePrCutterCut( void );
static void t_handlePrTeach( unsigned char *pFrame );
static void t_handlePrReqLabelTransfer( unsigned char *pFrame  );
static void t_handlePrMask( unsigned char *pFrame );
static void t_handlePrPCBA_ID( unsigned char *pFrame );
static void t_handlePrNomSize( unsigned char *pFrame );
static void t_handlePrTest( unsigned char *pFrame );
static void t_handlePrRam( unsigned char *pFrame );
static void t_handlePrCutterHome( void );
static void t_handlePrReqCutterStatus( void );
static void t_handlePrReqDotWear( void );
static void t_handlePrFactoryDefaults( void );
static void t_handlePrStartGapCal( void );
static void t_handlePrGapCalNext( void );
static void t_handlePrStartTUCal( void );
static void t_handlePrTUCalNext( void );
static void t_handlePrUseContinuousStock( void );
static void t_handleStationId( unsigned char order );
static void t_handlePrCutterDefault( void );
static void t_handlePrLabelSize( unsigned char *pFrame );
static void t_handlePrReqDotStatus( void );
static void t_handlePrHeadPower( unsigned char *pFrame );
static void t_handleWgReqSysInfo( void );
static void t_handleReqSysInfo( void );
static void t_handlePrSetTime( unsigned char *pFrame );
static void t_handlePrStopTime( unsigned char *pFrame );
static void t_handlePrSetHTGapSize( unsigned char *pFrame );
static void t_handlePrSetGTGapSize( unsigned char *pFrame );
static void t_handlePrSetLowLabelMinMax( unsigned char *pFrame );
static void t_handlePrCalPrintheadResistance( unsigned char *pFrame );


/* handle weigher messages */
static void t_handleReqWakeup( void );
static void t_handleReqConfig( void );
static void t_handleModeMsg( unsigned char mode );
static void t_handleWriteConfig( unsigned char *pFrame );
static void t_handleReqStatus( void );
static void t_handleEnable( void );
static void t_handleDisable( void );
static void t_handleRezero( void );
static void t_handleReset( void );
static void t_handleReqWeight( void );
static void t_handleControl( unsigned char cntrl );
static void t_handleReqVersion( void );
static void t_handleReqInfo( void );
static void t_handleResetDefaultCfg( void );
static void t_handleReqCat3Statistics( void );
static void t_handleCat3DateTime( unsigned char *pFrame );
static void t_handleReqCat3Page( unsigned char page, unsigned char index );
static void t_handleReqCat3NextRecord( void );
static void t_handleReqCat3EraseLog( void );
static void t_handleCat3RecordWrite( unsigned char *pFrame  );
static void t_handleValueMaxRequest( unsigned char *pFrame );
static void t_handleValueMaxWriteAssembly( unsigned char *pFrame );
static void t_handleValueMaxWriteVersion( unsigned char *pFrame );
static void t_handleValueMaxWriteSerial( unsigned char *pFrame );
static void t_handleValueMaxWriteDate( unsigned char *pFrame );
static void t_handleWriteVMFit( unsigned char *pFrame );
/******************************************************************************/
/******************************************************************************/




/*************************** avery private functions **************************/
/******************************************************************************/
static void translateAVPrinterMessage( unsigned char *pBfr );
static void translateAVWeigherMessage( unsigned char *pBfr );
static void translateToBinary( BinaryHeader_t *pDest, AsciiHeader *pSrc );
static bool isValidAVPacket( unsigned char *pBuffer );
/******************************************************************************/
/******************************************************************************/




/*************************** common public functions **************************/
/******************************************************************************/
unsigned short charToShort( unsigned char *pData );
unsigned short nCharToShort( unsigned char *pData );
unsigned long charToLong( unsigned char *pData );
unsigned long nCharToLong( unsigned char *pData );
unsigned char getCharFromShort( unsigned short *pWord, unsigned char index );
unsigned char getCharFromLong( unsigned long *pLong, unsigned char index );
/******************************************************************************/
/******************************************************************************/




/************************** avery public functions *************************/
/******************************************************************************/
unsigned char buildResponsePacket( Packet_t *pMsg, AVMSG_t cmd, unsigned short blkNum,
                                   unsigned short numBlks, void *pData, unsigned char dataLen );

void sendAVPrinterMsg(  unsigned char *pMsg, unsigned short length );
void sendAVWeigherMsg(  unsigned char *pMsg, unsigned short length );
uint32_t formatSendWGMsg( BinaryHeader_t *pHeader, void *pData, uint16_t length );
unsigned char fromAsciiHex8( unsigned char *pData, unsigned char val );
unsigned short fromAsciiHex16( unsigned char *pData, unsigned short val );
unsigned long fromAsciiHex32( unsigned char *pData, unsigned long val );
unsigned short binBlockToAscii( unsigned char *pData, unsigned char *pSrc, unsigned short length );
void fromAsciiHexBlock( unsigned char *pData, unsigned char *pSrc, unsigned short length );
void encryptChar( unsigned char *pChar );
unsigned char toAsciiHex( unsigned char *pData, unsigned char len, int val );
unsigned char stoAsciiHex( unsigned char *pData, unsigned char len, int val );
void longToAsciiHex( unsigned char *pData, unsigned char len, long val );
void auditLogToAsciiHex( unsigned char *pData, unsigned char *pSrc );
/******************************************************************************/
/******************************************************************************/
#endif