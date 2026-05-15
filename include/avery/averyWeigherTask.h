#ifndef AVERYWEIGHERTASK_H
#define AVERYWEIGHERTASK_H
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "averyProtocol.h"
#include "weigher.h"
#include "averyWeigherCfg.h"

#define weigher_task_PRIORITY ( configMAX_PRIORITIES - 1 )
/* weighing manager */
typedef struct
{

    int32_t     weightFromInterrupt;     /* Filtered value as calculated via the ADC interrupt */
    int32_t     uncompensatedWeight;     /* Filter weight WITHOUT tilt/gravity compensation */
    int32_t     compensatedWeight;       /* Filter weight WITH tilt/gravity compensation */
    int32_t     compensatedTiltX;        /* Compensated tilt correction */
    int32_t     compensatedTiltY;        /* Compensated tilt correction */
    int16_t     filteredIncX;            /* 14 bit filtered X acceleration value */
    int16_t     filteredIncY;            /* 14 bit filtered Y acceleration value */
    uint16_t    rawTemperature;          /* The temperature read from the inclinometer in raw counts */
    uint16_t    calTemp;                 /* Temperature at which calibrated in counts */
    int16_t     tempDegrees;             /* Temperature converted  to degrees c */
    int16_t     tempOffset;              /* Temperature offset correction */
    int16_t     zeroTC;                  /* Zero TC in counts/degrees C */
    int16_t     spanTC;                  /* Span TC in counts/degrees C */
    uint32_t    inversionOffset;         /* Initial values will change depending on the asset value */
    uint32_t    normalOffsetCorrection;  /* Initial values will change depending on the asset value */
    uint32_t    reverseOffsetCorrection; /* Initial values will change depending on the asset value */
    uint8_t     cellVariant;             /* Some cells physical features are different */
    uint8_t     polarity;                /* Used to indicate the polarity of the load cell */
    float       creepComp;               /* Creep compensation */

    /* Filtering data */
    uint8_t filterCount;
    uint8_t stabilityCount;
    uint8_t bufferIndex;

    int32_t filterBuffer[ 6 ];
    int32_t filteredRawWeight;
}AVWeigherMgr_t;

enum CELL_POLARITY
{
    NORMAL,
    INVERTED
};    
    
/* avery defines */
#define MANU_VARIANT_INVERTED      0x01
#define MANU_VARIANT_TILT_DISABLED 0x02

enum HOST_DEFS
{
    T_UPDATE            = 'U',
    T_ENQUIRY           = 'Q',
    T_CLEAR	            = 'X',
    T_COMMAND	        = 'C',
    T_RESPONSE	        = 'R',
    T_ACKNOWLEDGE       = 'A',
};

BaseType_t createAveryWeigherTask( WeigherStyle style, QueueHandle_t msgQueue );
TaskHandle_t getAveryWeigherHandle( void );
void initADCFilter( FlashWeighCfg_t *pFlash );
uint32_t getFilteredUnCompensatedWeight( void );
uint32_t getFilteredCompensatedWeight( void );
uint32_t getRawUnFilteredWeight( void );
float getAveryAcceleration( int16_t rawTilt, int32_t offset );
float getTempInDegrees( uint16_t rawTemp );
bool applyAverageFilter( void );
void applyWeightFilter( void );
void applyCompensation( void );
void processWeight( uint32_t *pRawWeight );

/* message handlers */
void processVersionMsg( AVIWGMSG_t *pMsg, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processStatusMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processWeightMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processFixedCfgMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processMFGMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processLCCalMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processModeMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processFilterMsg( uint8_t *pData, int8_t action, uint16_t *pSendMsgLen, int16_t *pHostMsg );
void processAuditMsg( int8_t action, ASCIIAudit_t *pLog,  uint16_t *pSendMsgLen, int16_t *pHostMsg );
#endif




