#ifndef AVERYSCALECFG_H
#define AVERYSCALECFG_H
#include "stdint.h"

#define MAX_FIXED_CONFIG_ROW		4
#define MAX_FIXEDCONFIG_COLUMNS	        64

typedef struct
{
    uint8_t     data[MAX_FIXED_CONFIG_ROW][MAX_FIXEDCONFIG_COLUMNS];
    uint32_t    checksum;       
}FlashFixedCfg_t; 

typedef struct
{
    FlashFixedCfg_t     fixedConfig;
    uint16_t    sensorToPheadDistance;                          /* distance in tenth mm */
    uint8_t     printerId;				        /* id number for this scale */
    uint8_t     labelEdgeVal;                                   /* how big is an edge */
    uint8_t     paperDetectVal;                                 /* trigger point between paper and paper out */
    uint8_t     gapSensorBackingVal;                            /* analog comparator when backing paper is detected */
    uint8_t     gapSensorLabelVal;                       	/* analog comparator when backing and label paper is detected */
    uint8_t     labelPrintDensity;
    uint8_t     receiptPrintDensity;
    uint8_t     gapCal;				                /* calibration value for gap sensor */
    uint8_t     labelTkCal;					/* calibration value for label taken sensor */
    uint8_t     labelEdgeMinDiff;				/* the min diff between max & min gap sense reading that constitutes an edge */
    uint8_t     mediaSensorCalVal;
    uint8_t     takenSensorMode;
    uint32_t    takenSensorThreshold;
}FlashPrCFG_t;

/* internal flash weigher configuration */
typedef struct
{
    uint8_t     type;
    uint8_t     unused; 
    uint16_t    checksum;
}FlashFilterData_t; 

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
}FlashWeigherCap_t; 

  
typedef struct
{
    uint16_t    vlChecksum;     /* validation library checksum */
    uint16_t    tuChecksum;     /* traced update checksum */
    uint32_t    checksum;
}FlashValidation_t;

typedef struct
{
    uint32_t    zeroLoad;
    uint32_t    fullLoadSpan;
    uint32_t    checksum;    
}FlashLoadCellCalib_t; 

typedef struct
{
    uint32_t    factoryFactor;
    uint32_t    siteFactor;
    uint32_t    checksum;
}FlashGravityData_t; 

typedef struct
{
    int32_t     xOffset;
    int32_t     yOffset;
    int32_t     tCal;
    uint32_t    checksum; 
}FlashInclOffset_t; 

typedef struct
{
    int32_t     zeroOffset;         /* system induced offset */
    int32_t     xk1;                /* x axis correction coefficent	for linearity error in the load */
    int32_t     xk2;                /* x axis correction coefficent	for the squared errors in the loadcell */                     
    int32_t     yk;                 /* y axis correction coefficent */
    int32_t     tCal;               /* inclinometer temperature during calibration */
    int32_t     xtc;                /* x Inclinometer temperature coefficent */
    int32_t     ytc;                /* y Inclinometer temperature coefficent */
    uint32_t    checksum;   
}FlashIncClib_t;     

#define FL_VERSION_LEN          29
typedef struct
{
    uint8_t     capacity;
    uint8_t     variant; 
    uint8_t     quality;
    uint8_t     unused;                        
    uint8_t     serialNumber[FL_VERSION_LEN + 1]; 
    uint8_t     creationDate[FL_VERSION_LEN + 1]; 
    uint8_t     creationTime[FL_VERSION_LEN + 1]; 
    uint16_t    checksum;
}FlashMFG_t; 

typedef struct
{
    int32_t     creepHi;
    int32_t     creepLo;
    int32_t     timeConstHi;
    int32_t     timeConstLo;
    uint32_t    checksum;
}FlashCreep_t; 

typedef struct
{
    int32_t     temperatureOffset;
    uint32_t    checksum;
}FlashTempOffset_t;

typedef struct
{
    int32_t     calibrationTemperature;
    uint32_t    checksum;
}FlashCalTemp_t; 

typedef struct
{
    int32_t     zeroTC;
    int32_t     spanTC;
    uint32_t    checksum;
}FlashTempCoef_t;


typedef struct
{
    FlashFixedCfg_t             fixedConfig;
    FlashLoadCellCalib_t        loadCellCalib;
    FlashGravityData_t          gravityData;
    FlashFilterData_t           filterData;
    FlashInclOffset_t           incOffsets;     /* 0 degree inclinomter offsets */
    FlashIncClib_t              incCalib;       /* loadcell cal data */
    FlashMFG_t                  manufact;
    FlashCreep_t                creep;          /* creep factors */
    FlashValidation_t           valib;          /* validation library */
    FlashTempOffset_t           tempOffset;     /* temperature offset */
    FlashCalTemp_t              calTemp;        /* calibration temperature */
    FlashTempCoef_t             tempCo;         /* temperature coefficients */
    FlashWeigherCap_t           wgCap;          /* user programmable capcaity info */
}FlashWeighCfg_t;                             

#define MID_DENSITY_IDX_DEFAULT    5                     
#define LABEL_SENSOR_DEFAULT_VAL   50           /* from service.c */
#define PAPER_DETECT_DEFAULT_VAL   100
#define PRINT_DENSITY_DEFAULT      100
#define SENSOR_TO_PHEAD_DIST_MM    350          /* ie 35.0mm */
#endif