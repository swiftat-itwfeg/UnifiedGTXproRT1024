#ifndef AVERYWEIGHERCFG_H
#define AVERYWEIGHERCFG_H
#include "stdint.h"
#include <stdbool.h>
#include "averyScaleCfg.h"

typedef struct
{
    uint8_t     type;
    uint8_t     unused; 
    uint16_t    checksum;
} WGFilterData_t;

/* user capacity */
typedef struct
{
    int32_t     fullLoad;
    int32_t     stdTareRange;
    int32_t     freeTareRange;
    int32_t     changeOver;
    uint16_t    balance10;
    uint16_t    balance4;
    uint16_t    powerupBalance;
    int16_t     increment1;
    int16_t     increment2;
    uint8_t     cellType;
    uint8_t     cellOutput;
    uint8_t     unitsOfMeasure;
    uint8_t     decimalPlaces;
    uint32_t    reserved; 
    uint16_t    checksum;
}WGCapacity_t; 

typedef struct
{
    uint16_t    vlChecksum;     /* validation library checksum */
    uint16_t    tuChecksum;     /* traced update checksum */
    uint32_t    checksum;
} WGValidChk_t;

/* default loadcell calibration */     
#define NOMINAL_CAL_ZERO_LOAD 3000000
#define NOMINAL_CAL_FULL_LOAD 6000000    
typedef struct
{
    uint32_t zeroLoad;
    uint32_t fullLoadSpan;
    uint32_t checksum;    
}LoadCellCalib_t; 

/* the default gravity factor for the Soho site */
#define SOHO_GRAVITY_FACTOR     9812870

typedef struct
{
    uint32_t factoryFactor;
    uint32_t siteFactor;
    uint32_t checksum; 
}GravityData_t;

/* defaults */
#define INC_OFFSET_VAL          1024 
#define Dk                      (INC_OFFSET_VAL << 3)
#define ABOUT_20_DEGREES_C      174
typedef struct
{
    int32_t xOffset;
    int32_t yOffset;
    int32_t tcal;
    uint32_t checksum; 
}IncOffset_t;

#define INC_SENS_VAL            1638 
#define Sk                      (INC_SENS_VAL << 3)
#define RADS_TO_DEGREES         57.29578
#define TOk                     197
#define TSk                     -1.083

typedef struct
{
    int32_t zeroOffset;         /* System induced offset */
    int32_t xk1;                /* X axis correction coefficent	for linearity error in the load */
    int32_t xk2;                /* X axis correction coefficent	for the squared errors in the loadcell */                                
    int32_t yk;                 /* Y axis correction coefficent */
    int32_t tcal;               /* Inclinometer temperature during calibration */
    int32_t xtc;                /* X Inclinometer temperature coefficent */
    int32_t ytc;                /* Y Inclinometer temperature coefficent */
    uint32_t checksum;          /* Actually U16, but needed for alignment */
}IncCalib_t;     

#define WG_VERSION_LEN		29

typedef struct
{
    uint8_t  capacity;
    uint8_t  variant; 
    uint8_t  quality;
    uint8_t  unused;                        
    uint8_t  serialNumber[ WG_VERSION_LEN + 1 ]; 
    uint8_t  creationDate[ WG_VERSION_LEN + 1 ]; 
    uint8_t  creationTime[ WG_VERSION_LEN + 1 ]; 
    uint16_t checksum;
}WGManfacture_t; 

/* creep compensation */
#define MIN_CREEP_TEMPERATURE     -10
#define NOMINAL_CREEP_TEMPERATURE 20
#define MAX_CREEP_TEMPERATURE     40
typedef struct
{
    int32_t creepHi;
    int32_t creepLo;
    int32_t timeConstHi;
    int32_t timeConstLo;
    uint32_t checksum;
}WGCreep_t; 

typedef struct
{
    int32_t     temperatureOffset;
    uint32_t    checksum;
}WGTempOffset_t;

typedef struct
{
    int32_t     calibrationTemperature;
    uint32_t    checksum;
}WGCalTemp_t; 

typedef struct
{
    int32_t     zeroTC;
    int32_t     spanTC;
    uint32_t    checksum;
}WGTempCoef_t;

typedef struct
{
    LoadCellCalib_t     loadCellCalib;
    GravityData_t       gravityData;
    WGFilterData_t      filterData;
    IncOffset_t         incOffsets;     /* 0 degree inclinomter offsets */
    IncCalib_t          incCalib;       /* data obtained when calibrating the loadcell/inclinometer */
    WGManfacture_t      manufact;
    WGCreep_t           creep;          /* Creep factors */
    WGValidChk_t        valib;          /* Validation library */
    WGTempOffset_t      tempOffset;     /* Temperature offset */
    WGCalTemp_t         calTemp;        /* Calibration temperature */
    WGTempCoef_t        tempCo;         /* Temperature coefficients */
    WGCapacity_t        wgCap;          /* User programmable capcaity info */
 }AVWGCFG_t;             

uint16_t crc16( uint8_t *pData, uint32_t Length );
bool isLoadCellCalVaid( FlashLoadCellCalib_t *pLCCal );
bool isManufactureValid( FlashMFG_t *pMfg );
bool isGravityValid( FlashGravityData_t *pGravity );
bool isFilterDataValid( FlashFilterData_t *pFilterData );
bool isCreepValid( FlashCreep_t *pCreep );
bool isValidationLibValid( FlashValidation_t *pValid );
bool isTempOffsetValid( FlashTempOffset_t *pTemp );
bool isCalTempValid( FlashCalTemp_t *pCalTemp );
bool isTempCoefValid( FlashTempCoef_t *pTempCoef );
bool isIncOffsetValid( FlashInclOffset_t *pIncOffset );
bool isIncCalValid( FlashIncClib_t *pIncCal );
#endif