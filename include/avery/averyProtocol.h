#ifndef AVERY_PROTOCOL_H
#define AVERY_PROTOCOL_H
#include <stdint.h>

/* avery message types ( TO DO: consolidate with hobart messages )*/
#define AVERY_PROTOCOL          2

typedef enum
{
    AV_STX = 2,
    AV_ETX,
    AV_FIRSTVALIDCMD, 
    AV_GETPRINTERSTATUSCMD = AV_FIRSTVALIDCMD,
    AV_GETAPPFIRMWARENUM,                          /* 5 */
    AV_CHANGEMODE,         
    AV_ADJUSTPRINTDENSITY, 
    AV_PRINTLABEL, 
    AV_FORMFEED,   
    AV_GETLABELLENGTH, 
    AV_SETLABELLENGTH, 
    AV_GETLABELGAPLENGTH, 
    AV_SETLABELGAPLENGTH, 
    AV_CALGAPSENSORBIAS, 
    AV_SETGAPSENSORBIAS, 
    AV_CALMEDIASENSORBIAS, 
    AV_SETMEDIASENSORBIAS, 
    AV_CALTAKEUPSENSORBIAS, 
    AV_SETTAKEUPSENSORBIAS, 
    AV_CALTAKEUPSENSORSPAN,                          
    AV_SETTAKEUPSENSORSPAN,                          
    AV_CALTAKENSENSORTHRESHOLD, 
    AV_SETTAKENSENSORTHRESHOLD, 
    AV_GETPCBREV, 
    AV_OPENCASHDRAWER,     
    AV_SETPORTPIN,  
    AV_READPORTPIN, 
    AV_READADC,    
    AV_DRIVEMOTOR,                
    AV_SETLOGOLEDMODE,     
    AV_FORCERESET, 
    AV_PROGRAM, 
    AV_PRNTHEAD_DIAG, 
    AV_PEEK,          
    AV_GETBOOTFIRMWARENUM,  
    AV_GETAPPFIRMWAREDATE,  
    AV_GETBOOTFIRMWAREDATE, 
    AV_PRINTLABELNOPARK, 
    AV_GETPRINTERDEBUG,  
    AV_GETMODE,                 
    AV_GETPRINTDENSITY,         
    AV_GETGAPSENSORBIAS,        
    AV_GETMEDIASENSORBIAS,      
    AV_GETTAKEUPSENSORBIAS,     
    AV_GETTAKEUPSENSORSPAN,     
    AV_GETTAKENSENSORTHRESHOLD, 
    AV_GETLOGOLEDMODE,          
    AV_GETEXTRAPRINTERSTATUSCMD, 
    AV_SETSTROBELEN,             
    AV_SETTAKENSENSORMODE, 
    AV_GETTAKENSENSORMODE, 
    AV_SETPRINTERID, 
    AV_GETPRINTERID, 
    AV_SETPAPEROUTTHRESHOLD, 
    AV_GETPAPEROUTTHRESHOLD, 
    AV_GETMINLABELEDGEDIFF, 
    AV_SETMINLABELEDGEDIFF, 
    AV_SETPRINTSPEED, 
    AV_SETLINERLESSLABELPRINTER, 
    AV_GETCUTTERFIRMWARENUM,      
    AV_GETCUTTERFIRMWAREDATE,     
    AV_SETPAPERCUTTERCONFIG,      
    AV_INITIALPAPERCUT,           
    AV_GETPAPERCUTTERSTATUSCMD,   
    AV_TESTPAPERCUT,              
    AV_PAPERCUTTERINTERFACERESET, 

    AV_GETPROCESSORTYPE,         
    AV_PROGRAMWITHPROCESSORTYPE, 

    AV_INVALID_COMMAND 
}AVMSG_t;


typedef enum 
{
    IT103MSG_NO_MESSAGE,	
    IT103MSG_VERSION,
    IT103MSG_WEIGHT,
    IT103MSG_FILTER,
    IT103MSG_GRAVITY,
    IT103MSG_KEY,	
    IT103MSG_RESET,
    IT103MSG_STATUS,
    IT103MSG_UPDATE_FIRMWARE,
    IT103MSG_USER_CAPACITY,	
    IT103MSG_INCLINOMETER,
    IT103MSG_AUDIT,
    IT103MSG_FIXED_CONFIG,
    IT103MSG_VALIDATION_LIB,	
    IT103MSG_LOADCELL_CALIB,
    IT103MSG_CHANGE_MODE,
    T103MSG_INCLINOMETER_FACTORS,
    T103MSG_INCLINOMETER_CALIBRATION,
    IT103MSG_MANUFACTURE,
    IT103MSG_COMPENSATED_TILT,
    IT103MSG_CREEP_FACTORS,
    IT103MSG_LEGAL_VERIFICATION,
    IT103MSG_TEMPERATURE_OFFSET,
    IT103MSG_CALIBRATION_TEMPERATURE,
    IT103MSG_TEMPERATURE_COEFFICIENTS,
    IT103MSG_DATETIME,
    IT103MSG_MAX_MESSAGES
}AVWGMSG_t; 

typedef struct
{
    unsigned char  stx;
    unsigned char  cmd;
    unsigned short blkNum;
    unsigned short numBlks;
    unsigned char  pad;
    unsigned char  len;
}AVMSGHEADER_t;

#define FULL_SPEED_USB
#ifdef FULL_SPEED_USB
/* message defines */
#define MAX_AVPAYLOAD_SIZE        64     /* full speed max size of bulk endpoint */
#define PACKET_DATA_SIZE    ( MAX_AVPAYLOAD_SIZE - sizeof( AVMSGHEADER_t ) )
#else
#define MAX_AVPAYLOAD_SIZE        512    /* high speed max size of bulk endpoint */
#define PAYLOAD_SIZE    ( MAX_AVPAYLOAD_SIZE - sizeof( AVMSGHEADER_t ) )
#endif

#define BITS_PER_PIXEL				4
#define PRINTHEAD_BYTES_80MM                    80
#define PACKETS_PER_LINE			( ( PRINTHEAD_BYTES_80MM * BITS_PER_PIXEL + PACKET_DATA_SIZE -1 ) / PACKET_DATA_SIZE )  /* requires just over 5 packets to give (640 * 4)/8 bits of grey */
#define GREY_BUF_SIZE				( PACKETS_PER_LINE * PACKET_DATA_SIZE )
#define BYTES_PER_BITMAPLINE		        ( GREY_BUF_SIZE/BITS_PER_PIXEL )


typedef struct
{
    AVMSGHEADER_t  header;
    unsigned char  msgData[PACKET_DATA_SIZE];  
}AVMSGPAYLOAD_t;

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
    unsigned char  numBytes; 
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
    unsigned char messageSize[SIZE_IN_NIBBLES(sizeof(((BinaryHeader_t*)0)->messageSize ))];	/* size of the message block including the header */
    unsigned char cvn[SIZE_IN_NIBBLES(sizeof( ((BinaryHeader_t*)0)->cvn ))];		        /* command version number */
    unsigned char slaveID[SIZE_IN_NIBBLES(sizeof(((BinaryHeader_t*)0)->slaveID))];		/* id of sending device */
    unsigned char cmd[SIZE_IN_NIBBLES(sizeof(((BinaryHeader_t*)0)->cmd))];		        /* value describing the command */
    char action;								                /* What action to take ('U', 'Q', 'X') */
    char type;									                /* the type of message ('C', 'R', 'A') */
    unsigned char status[SIZE_IN_NIBBLES(sizeof(((BinaryHeader_t*)0)->status))];			/* status of sending device */
    unsigned char sequenceNum[SIZE_IN_NIBBLES(sizeof(((BinaryHeader_t*)0)->sequenceNum))];	/* self sequencing message counter */
}AsciiHeader; 
#pragma pack (pop, onthewire)



#define MAX_AV_INTERNAL_SIZE    64
/* internal avery printer message */
typedef struct
{
    AVMSG_t  cmd; 
    unsigned char msgData[ MAX_AV_INTERNAL_SIZE ];
    unsigned short msgLength;
}AVIMSG_t;

/* internal avery weigher message */
typedef struct
{
    BinaryHeader_t header;
    unsigned char msgData[ MAX_AV_INTERNAL_SIZE ];
    unsigned short msgLength;
}AVIWGMSG_t;
/* bcc is the size of the ascii crc16 */
#define	BCC_LEN		           4

typedef enum 
{
        M_SUCCESS               =  0,
        M_TOO_BIG	        = -1,
        M_TIMEOUT       	= -2,
        M_INVALID_DATA	        = -3,
        M_BAD_PARAMETER	        = -4,
        M_INVALID_ID	        = -5,
        M_INVALID_ACTION	= -6,
        M_INVALID_TYPE		= -7,
        M_UNKNOWN_MESSAGE	= -8,
        M_BUSY			= -9,
        M_NO_DATA		= -10,
        M_INVALID_CVN		= -11,
        M_INTERNAL_ERROR	= -12,
        M_WRITE_PROTECTED	= -13,
        M_NO_MEMORY		= -14,
        M_NO_ROOM		= -15,
        M_DATA_CORRUPT		= -16,
        M_MESSAGE_MISMATCH      = -17,
        M_SHUTTING_DOWN		= -1000
}AVMSGStatus_t;

#define SIZE_IN_NIBBLES(a) ( a << 1 )

        

typedef enum  
{
    FILTERED_UNCOMPENSATED	= 0x00,
    WITHOUT_FILTER		= 0x01,
    FILTERED_COMPENSATED	= 0x02,
    MAX_WEIGHT_SUBCOMMANDS
}AVWeightType_t;        

/* weigher message structures */
typedef struct
{
    uint8_t type;
    uint32_t weight;
    uint32_t status;
}BinWeight_t;

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t type[ SIZE_IN_NIBBLES( sizeof( ((BinWeight_t*)0)->type )) ];
    uint8_t weight[SIZE_IN_NIBBLES( sizeof( ((BinWeight_t*)0)->weight )) ];
    uint8_t status[SIZE_IN_NIBBLES( sizeof( ((BinWeight_t*)0)->status )) ];
}ASCIIWeight_t;
#pragma pack (pop, onthewire)

typedef struct
{
    uint32_t status;
}BinStatus_t;

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t status[ SIZE_IN_NIBBLES(sizeof( ((BinStatus_t*)0)->status )) ];
}ASCIIStatus_t;	
#pragma pack (pop, onthewire)

#define VER_LEN         29
typedef struct
{
    uint8_t type;
    uint8_t firmwareNumber[ VER_LEN + 1 ];
    uint8_t creationDate[ VER_LEN + 1 ];
    uint8_t creationTime[ VER_LEN + 1 ];
}BinVersion_t;

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t type[ SIZE_IN_NIBBLES( sizeof( ((BinVersion_t*)0)->type ) ) ];
    uint8_t firmwareNumber[ SIZE_IN_NIBBLES( sizeof( ((BinVersion_t*)0)->firmwareNumber ) ) ];
    uint8_t creationDate[ SIZE_IN_NIBBLES( sizeof( ((BinVersion_t*)0)->creationDate ) ) ];
    uint8_t creationTime[ SIZE_IN_NIBBLES( sizeof( ((BinVersion_t*)0)->creationTime ) ) ];
}ASCIIVersion_t;	
#pragma pack (pop, onthewire)

typedef struct 
{
    uint8_t type;  
}BinFilter_t;

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t type[ SIZE_IN_NIBBLES( sizeof( ((BinFilter_t*)0)->type ) ) ];
}ASCIIFilter_t;	
#pragma pack (pop, onthewire)

typedef struct
{
    uint8_t type;
    uint16_t dataX;
    uint16_t dataY;
}BinInclin_t;

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t type[ SIZE_IN_NIBBLES( sizeof( ( (BinInclin_t*)0)->type ) ) ];
    uint8_t dataX[ SIZE_IN_NIBBLES( sizeof( ( (BinInclin_t*)0)->dataX ) ) ];
    uint8_t dataY[ SIZE_IN_NIBBLES( sizeof( ( (BinInclin_t*)0)->dataY ) ) ];
}ASCIIInclin_t;	
#pragma pack (pop, onthewire)

typedef struct
{
    uint8_t mode;   
}BinMode_t;

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t mode[ SIZE_IN_NIBBLES(sizeof( ( (BinMode_t*)0)->mode ) ) ];
}ASCIIMode_t;
#pragma pack (pop, onthewire)



/* audit limitations */
#define AB_MIN_AUDIT_NUM                1
#define AB_MAX_AUDIT_NUM		60000
#define AB_MAX_AUDIT_LOGS		1000
#define AB_AUDIT_EVENT_NOT_DEFINED	0xFFFF
#define AB_MAX_AUDIT_DATA		24
#define AB_PRESERVED_AUDIT_COUNT	100

typedef struct
{
    uint32_t dateTime;	   	
    uint16_t eventNumber;	
    uint8_t  paramID;
    uint8_t  subID;
    uint8_t  data[ AB_MAX_AUDIT_DATA ];
}BinAudit_t; 

#pragma pack (push, onthewire, 1)
typedef struct
{
    uint8_t dateTime[ SIZE_IN_NIBBLES(sizeof( ( (BinAudit_t*)0)->dateTime ) ) ];
    uint8_t eventNumber[ SIZE_IN_NIBBLES(sizeof( ( (BinAudit_t*)0)->eventNumber ) ) ];
    uint8_t paramID[ SIZE_IN_NIBBLES(sizeof( ( (BinAudit_t*)0)->paramID ) ) ];
    uint8_t subID[ SIZE_IN_NIBBLES(sizeof( ( (BinAudit_t*)0)->subID ) ) ];
    uint8_t data[ SIZE_IN_NIBBLES(sizeof( ( (BinAudit_t*)0)->data ) ) ];
}ASCIIAudit_t;	
#pragma pack (pop, onthewire)


#ifndef htons
#define htons(a) ((((a) >> 8) & 0x00ff) | (((a) << 8) & 0xff00))
#endif

#ifndef ntohs
#define ntohs(a) htons((a))
#endif




#endif