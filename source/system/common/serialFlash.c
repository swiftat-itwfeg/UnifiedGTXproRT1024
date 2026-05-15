#include "serialFlash.h"
#include "w25x10cl.h"
#include "cat3Audit.h"
#include "averyScaleCfg.h"
//#include "globalPrinter.h"
//#include "globalWeigher.h"
#include "weigher.h"
#include "threadManager.h"
#include "deviceProperties.h"
#include "vendor.h"
#include "semphr.h"
#include "fsl_debug_console.h"
#include <string.h>
#include "printHead.h"
#include "lp5521.h"
#include "takeupMotor.h"

/*********************************** common ***********************************/
/******************************************************************************/
static SemaphoreHandle_t sMutex_;
/******************************************************************************/
/******************************************************************************/


/************************** common public functions  **************************/
/******************************************************************************/

/*******************************************************************************/
/*!   \fn bool initSerialFlashMutex( void )
 
      \brief
        public: This function creates a semaphore for synchronized access to 
                the serial flash. 

      \author
          Aaron Swift
*******************************************************************************/ 
bool initSerialFlashMutex( void )
{
    bool result = false;
    
    sMutex_ = xSemaphoreCreateMutex(); 
    if( !sMutex_ ) {         
        PRINTF("initSerialFlashMutex(): Failed to create mutex!\r\n" );
        assert( 0 );
    } else {
        result = true;
    }
    return result;  
}

/*******************************************************************************/
/*!   \fn void getLockSerialFlash( void )
 
      \brief
        public: This function obtains the lock on the serial flash.                

      \author
          Aaron Swift
*******************************************************************************/ 
bool getLockSerialFlash( void )
{
    bool result = false;
    if( xSemaphoreTake( sMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) { 
        result = true;  
    }
    return result;    
}


/*******************************************************************************/
/*!   \fn void releaseLockSerialFlash( void )
 
      \brief
        public: This function releases the lock on the serial flash.                

      \author
          Aaron Swift
*******************************************************************************/ 
void releaseLockSerialFlash( void )
{
    xSemaphoreGive( sMutex_ );  
}


/*******************************************************************************/
/*!   \fn bool getDeviesProperties( DEVICE_PROPERTIES_t *pProperties )
 
      \brief
        public: This function reads the device properties stored in sector 3
                page 6 and return result of operation. the calling function 
                should call getLastError upon failure of this function.

      \return   result

      \author
          Aaron Swift
*******************************************************************************/ 
bool getDeviesProperties( DEVICE_PROPERTIES_t *pProperties )
{
    bool result = false;
    DEVICE_PROPERTIES_t prop;
    
    /* device properties starts at sector 3 page 6*/
    unsigned long addr = SECTOR3_BASE_ADDR  + ( PAGE_SIZE * 6 );

    if( readSerialFlash( addr, (uint8_t *)&prop, sizeof( DEVICE_PROPERTIES_t ) ) )
    {   
        memcpy( pProperties, &prop, sizeof( DEVICE_PROPERTIES_t ) ); 
        result = true;       
    }    
    return result;
}

/******************************************************************************/
/*!   \fn bool setDeviesProperties( DEVICE_PROPERTIES_t *pProperties )                                                             
 
      \brief
        This function writes the device properties to the serial flash 
        at sector 3 page 6 and returns the results of the write operation.

      \param DEVICE_PROPERTIES_t pointer to a device properties object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool setDeviesProperties( DEVICE_PROPERTIES_t *pProperties )
{
   bool result = false;
    
    /* device properties starts at sector 3 page 6 */
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 6 );
    if( writeSerialFlash( (unsigned char *)pProperties, addr, sizeof( DEVICE_PROPERTIES_t ), SECTOR_3 ) ) {
        result = true;        
    }    
    return result;  
}

/******************************************************************************/
/*!   \fn bool eraseDeviesProperties( void )                                                             
 
      \brief
        This function erases the device properties information stored in the 
        serial flash on sector 3 page 6 and returns results of erase operation.

      \param 

      \author
          Aaron Swift
*******************************************************************************/ 
bool eraseDeviesProperties( void )
{
    bool result = false;
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 6 );   
    /* erase the page */
    if( erasePage( addr, SECTOR_3 ) ) {
        result = true;    
    }
    return result;
}


/******************************************************************************/
/******************************************************************************/


/************************** hobart's public functions *************************/
/******************************************************************************/

/*******************************************************************************/
/*!   \fn bool getSerialWgConfiguration( WgConfiguration *pWeighConfig )
 
      \brief
        public: This function reads the weigher configuration from the serial flash.
                if the read operation completes without error then the read data is 
                copied into the *pWeighConfig. if the read operation completes with errors
                then the function will not copy the read data and will return false.
                the calling function should call getLastError upon failure of this function.
                

      \param WgConfiguration *pWeighConfig          
        
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
bool getSerialWgConfiguration( WgConfiguration *pWeighConfig )
{
    bool result = false;
    WgConfiguration wConfig;
    
    /* weigher configuration starts at sector 3 */
    unsigned long addr = SECTOR3_BASE_ADDR;

    if( readSerialFlash( addr, (uint8_t *)&wConfig, sizeof(WgConfiguration) ) )
    {   
        memcpy( pWeighConfig, &wConfig, sizeof( WgConfiguration ) ); 
        result = true;       
    }    
    return result;
}


/*!   \fn bool setSerialWgConfiguration( WgConfiguration *pWeighConfig )
 
      \brief
        public: This function writes the weigher configuration to the serial flash
                sector 3 page 0 and return result of operation. the calling function 
                should call getLastError upon failure of this function.
                

      \param WgConfiguration *pWeighConfig          
        
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
bool setSerialWgConfiguration( WgConfiguration *pWeighConfig )
{
    bool result = false;

    /* weigher configuration starts at sector 3 */
    unsigned long addr = SECTOR3_BASE_ADDR;

    if( writeSerialFlash( (unsigned char *)pWeighConfig, addr, sizeof(WgConfiguration), SECTOR_3 ) )
    {
        result = true;        
    }    
    return result;
}

/******************************************************************************/
/*!   \fn bool getSerialWeigherMFGInfo( WeigerMFGInfo *pWeigherMFGInfo )
 
      \brief
        public: This function reads the weigher manufacturing information from the serial flash.
                if the read operation completes without error then the read data is 
                copied into the *pWeigherMFGInfo. if the read operation completes with errors
                then the function will not copy the read data and will return false.
                the calling function should call getLastError upon failure of this function.
                

      \param WeigerMFGInfo *pWeigherMFGInfo          
        
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
bool getSerialWeigherMFGInfo( WeigerMFGInfo *pWeigherMFGInfo )
{
    bool result = false;
    WeigerMFGInfo tmpMFGInfo;
    
    /* weigher info starts at sector 3, page 1*/
    unsigned long addr = SECTOR3_BASE_ADDR + PAGE_SIZE;
    
    if( readSerialFlash(addr, (uint8_t *)&tmpMFGInfo, sizeof(WeigerMFGInfo)) )
    {   
        memcpy( pWeigherMFGInfo, &tmpMFGInfo, sizeof(WeigerMFGInfo) ); 
        result = true;
    }
    
    return result;
}

/******************************************************************************/
/*!   \fn bool setSerialWeigherMFGInfo( WeigerMFGInfo *pWeigherMFGInfo )
 
      \brief
        public: This function writes the weigher information to the serial flash
                sector 3 page 1 and return result of operation. the calling function 
                should call getLastError upon failure of this function.
                

      \param WgConfiguration *pWeighConfig          
        
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
bool setSerialWeigherMFGInfo( WeigerMFGInfo *pWeigherMFGInfo )
{
    bool result = false;
    
    /* weigher configuration starts at sector 3 page 0 
       weigher info start on the next page within the sector */
    unsigned long addr = SECTOR3_BASE_ADDR + PAGE_SIZE;
    if( writeSerialFlash( (unsigned char *)pWeigherMFGInfo, addr, sizeof(WeigerMFGInfo), SECTOR_3 ) )
    {
        result = true;        
    }    
    return result;
}

/******************************************************************************/
/*!   \fn bool setSerialWgDfltConfig( WgConfiguration *pWeighConfig )                                                              
 
      \brief
        This function set a default configuration to the serial flash 
        and returns the results.

      \param pWeighConfig pointer to a weigher configuration object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool setSerialWgDfltConfig( WgConfiguration *pWeighConfig )
{    
    pWeighConfig->center_of_maintenance_zone = 100068;
    pWeighConfig->gain_factor = 698741;   
    pWeighConfig->scale_factor = 5;  
        
    /* 300000 counts = 15Kg or 30lb. 2g = 40 counts, 0.005lb = 50 counts */   
    pWeighConfig->prepack_motion_count  = 5;
    pWeighConfig->initialize_zero_time  = 1;
    pWeighConfig->small_motion_limit    = 19;   //sof-3291 from 15 -> 10
    pWeighConfig->large_motion_limit    = 100;
    pWeighConfig->large_motion_count    = 36;
    pWeighConfig->small_motion_count    = 12; //8;
    pWeighConfig->no_motion_count       = 34;
    
    /* 30.09 lbs */ 
    pWeighConfig->max_weight            = 300900;
    pWeighConfig->value_max_on_off      = 0;
            
    /*********************************************************************
    * The assumption is made that the previously configured weigher type *
    * is loaded into the incoming config structure. If this weigher type *
    * is garbage, the eep will be defaulted to a PRIMARY WEIGHER,        *
    * otherwise it will be configured as it was previously. There is a   *
    * possibility that a secondary weigher with a blown configuration    *
    * could be made a primary weigher.                                   *
    *********************************************************************/
    if (pWeighConfig->weigher_type != SECONDARY_WEIGHER)
        pWeighConfig->weigher_type = PRIMARY_WEIGHER;
   
    /* reset cat1 information      Jan 1, 2018 12:00AM */
    pWeighConfig->last_calibration_date         = 1514782800;    
    pWeighConfig->last_configuration_date       = 1514782800;
    pWeighConfig->number_of_calibrations        = 0;
    pWeighConfig->number_of_configurations      = 0; 

    /*Don't set load cell to default. Code has no way of knowing which LC is.
     Installed. Should be set in Mfr or by service tech only! tmf 4/2010
    */     
    pWeighConfig->flags = WEIGH_MODE_AVOIR | DUALRANGE_NETWEIGHTLIMITCHECK;
    pWeighConfig->min_weight_to_print = 10;   /* 0.02lb or 10g */
    
    pWeighConfig->filter_speed = SLOW_FILTER_SPEED;
    if( getProductId() == 0x260 ) {
        pWeighConfig->weigher_model =  GL_SERVICE_SCALE_WEIGHER; 
    } else {
        pWeighConfig->weigher_model =  G_SERVICE_SCALE_WEIGHER; 
    }
    
     
    return( setSerialWgConfiguration( pWeighConfig ) );
}

/******************************************************************************/
/******************************************************************************/

/************************** avery's public functions *************************/
/******************************************************************************/
/******************************************************************************/
/*!   \fn void setAWgConfigDefaults( FlashWeighCfg_t *pNVCfg )                                                         

      \brief
        This function sets the avery weigher configuration to there default 
        values.
        
      \author
          Aaron Swift
*******************************************************************************/
void setAWgConfigDefaults( FlashWeighCfg_t *pNVCfg )
{
    if( pNVCfg != NULL ) {
        /* until we get default settings form Avery */
        memset( pNVCfg, 0, sizeof( FlashWeighCfg_t ) );
        
        /* TO DO: ask bal what these values should be? */
        pNVCfg->loadCellCalib.fullLoadSpan = 0;
        pNVCfg->loadCellCalib.zeroLoad = 0;         
        pNVCfg->loadCellCalib.checksum = 0;
        
        pNVCfg->gravityData.siteFactor = 0;
        pNVCfg->gravityData.factoryFactor = 0;
        pNVCfg->gravityData.checksum = 0;
          
        pNVCfg->filterData.type = 0;
        pNVCfg->filterData.unused = 0;
        pNVCfg->filterData.checksum = 0;
          
        pNVCfg->incOffsets.tCal = 0;
        pNVCfg->incOffsets.xOffset = 0;
        pNVCfg->incOffsets.yOffset = 0;
        pNVCfg->incOffsets.checksum = 0;

        pNVCfg->incCalib.zeroOffset = 0;
        pNVCfg->incCalib.ytc = 0;
        pNVCfg->incCalib.yk = 0;
        pNVCfg->incCalib.xtc = 0;
        pNVCfg->incCalib.xk2 = 0;
        pNVCfg->incCalib.xk1 = 0;
        pNVCfg->incCalib.tCal = 0;
        pNVCfg->incCalib.checksum = 0;

        pNVCfg->manufact.capacity = 0;
        memset( &pNVCfg->manufact.creationDate[0], 0, (FL_VERSION_LEN+ 1) );
        memset( &pNVCfg->manufact.serialNumber[0], 0, (FL_VERSION_LEN+ 1) );
        memset( &pNVCfg->manufact.creationTime[0], 0, (FL_VERSION_LEN+ 1) );
        pNVCfg->manufact.quality = 0;       
        pNVCfg->manufact.unused = 0;
        pNVCfg->manufact.variant = 0;
        pNVCfg->manufact.checksum = 0;

        pNVCfg->creep.timeConstLo = 0;
        pNVCfg->creep.timeConstHi = 0;
        pNVCfg->creep.creepLo = 0;
        pNVCfg->creep.creepHi = 0;
        pNVCfg->creep.checksum = 0;
        
        pNVCfg->valib.vlChecksum = 0; 
        pNVCfg->valib.tuChecksum = 0;
        pNVCfg->valib.checksum = 0;

        pNVCfg->tempOffset.temperatureOffset = 0;
        pNVCfg->tempOffset.checksum = 0;

        pNVCfg->calTemp.calibrationTemperature = 0;
        pNVCfg->calTemp.checksum = 0;

        pNVCfg->tempCo.spanTC = 0;
        pNVCfg->tempCo.zeroTC = 0;
        pNVCfg->tempCo.checksum = 0; 

        pNVCfg->wgCap.balance10 = 0;
        pNVCfg->wgCap.balance4 = 0;
        pNVCfg->wgCap.cellOutput = 0;
        pNVCfg->wgCap.cellType = 0;
        pNVCfg->wgCap.changeOver = 0;
        pNVCfg->wgCap.decimalPlaces = 0;
        pNVCfg->wgCap.freeTareRange = 0;
        pNVCfg->wgCap.fullLoad = 0;
        pNVCfg->wgCap.increment1 = 0;
        pNVCfg->wgCap.increment2 = 0;
        pNVCfg->wgCap.powerupBalance = 0;
        pNVCfg->wgCap.reserved = 0;
        pNVCfg->wgCap.stdTareRange = 0;
        pNVCfg->wgCap.unitsOfMeasure = 0;
        pNVCfg->wgCap.checksum = 0;
    } else {
        PRINTF("setConfigDefaults(): pNVCfg is NULL!\r\n");
    }
}

/******************************************************************************/
/*!   \fn void setAPrConfigDefaults( FlashPrCFG_t *pPrNVCfg )                                                         

      \brief
        This function sets the avery printer configuration to there default 
        values.
        
      \author
          Aaron Swift
*******************************************************************************/
void setAPrConfigDefaults( FlashPrCFG_t *pPrNVCfg )
{
    /* set printer configuration to it's default values */
    pPrNVCfg->gapSensorBackingVal = 0;
    pPrNVCfg->gapSensorLabelVal = 0;
    pPrNVCfg->labelEdgeVal = LABEL_SENSOR_DEFAULT_VAL;
    pPrNVCfg->labelPrintDensity = MID_DENSITY_IDX_DEFAULT;
    pPrNVCfg->paperDetectVal = PAPER_DETECT_DEFAULT_VAL;
    pPrNVCfg->labelPrintDensity = MID_DENSITY_IDX_DEFAULT;
    pPrNVCfg->receiptPrintDensity = PRINT_DENSITY_DEFAULT;
    pPrNVCfg->sensorToPheadDistance = SENSOR_TO_PHEAD_DIST_MM;   
}

/******************************************************************************/
/*!   \fn bool readAPrConfigFromFlash( FlashPrCFG_t *pPrNVCfg )                                                         

      \brief
        This function reads the Printer config section of the serial flash.
        if the read operation completes without error then the read data is 
        copied into the *pPrNVCfg and returns true.        
        
      \author
          Aaron Swift
*******************************************************************************/ 
bool readAPrConfigFromFlash( FlashPrCFG_t *pPrNVCfg )
{
    bool result = false;
    FlashPrCFG_t cfg;
    /* printer configuration starts at sector 3 page 2 */
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 2 );
    
    if( readSerialFlash(addr, (uint8_t *)&cfg, sizeof( FlashPrCFG_t ) ) ) {   
        memcpy( pPrNVCfg, &cfg, sizeof( FlashPrCFG_t ) ); 
        result = true;
    }
    return result;
}

/******************************************************************************/
/*!   \fn bool readAWgConfigFromFlash( FlashWeighCfg_t *pNVCfg  )                                                         

      \brief
        This function reads the Weigher config section of the serial flash.
        if the read operation completes without error then the read data is 
        copied into the *pNVCfg and returns true.     
        
      \author
          Aaron Swift
*******************************************************************************/ 
bool readAWgConfigFromFlash( FlashWeighCfg_t *pNVCfg  )
{
    bool result = false;
    FlashWeighCfg_t cfg;
    /* weigher configuration starts at sector 3 */
    unsigned long addr = SECTOR3_BASE_ADDR;
    
    if( readSerialFlash(addr, (uint8_t *)&cfg, sizeof( FlashWeighCfg_t ) ) ) {   
        memcpy( pNVCfg, &cfg, sizeof( FlashWeighCfg_t ) ); 
        result = true;
    }
    return result;
}

/******************************************************************************/
/*!   \fn bool writeAPrConfigToFlash( FlashPrCFG_t *pPrNVCfg )                                                         

      \brief
        This function checks the Printer config section of the flash for blank
        condition. If blank, the function will program the Printer configuration
        region of the flash and verify programming. Function returns true if
        seccessful. 
        
      \author
          Aaron Swift
*******************************************************************************/ 
bool writeAPrConfigToFlash( FlashPrCFG_t *pPrNVCfg )
{
    bool result = false;
    /* printer configuration starts at sector 3 page 2 */
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 2 );
    if( writeSerialFlash( (unsigned char *)pPrNVCfg, addr, sizeof(FlashPrCFG_t), SECTOR_3 ) ) {
        result = true;        
    }       
    return result;  
}

/******************************************************************************/
/*!   \fn bool writeAWgConfigToFlash( FlashWeighCfg_t *pNVCfg )                                                         

      \brief
        This function writes the weigher configuration to the serial flash
        sector 3 page 0 and return result of operation. the calling function 
        should call getLastError upon failure of this function.        

      \author
          Aaron Swift
*******************************************************************************/ 
bool writeAWgConfigToFlash( FlashWeighCfg_t *pNVCfg )
{
    bool result = false;
    /* weigher configuration starts at sector 3 */
    unsigned long addr = SECTOR3_BASE_ADDR;

    if( writeSerialFlash( (unsigned char *)pNVCfg, addr, sizeof(FlashWeighCfg_t), SECTOR_3 ) )
    {
        result = true;        
    }    
    return result;
}

/******************************************************************************/
/*!   \fn void showAPrConfiguration( FlashPrCFG_t *pPrNVCfg )                                                         

      \brief
        This function dumps the contents of the avery printer configuration to the 
        terminal.        
        
      \author
          Aaron Swift
*******************************************************************************/
void showAPrConfiguration( FlashPrCFG_t *pPrNVCfg )
{
    pPrNVCfg->gapSensorBackingVal = 0;
    pPrNVCfg->gapSensorLabelVal = 0;
    pPrNVCfg->labelEdgeVal = LABEL_SENSOR_DEFAULT_VAL;
    pPrNVCfg->labelPrintDensity = MID_DENSITY_IDX_DEFAULT;
    pPrNVCfg->paperDetectVal = PAPER_DETECT_DEFAULT_VAL;
    pPrNVCfg->labelPrintDensity = MID_DENSITY_IDX_DEFAULT;
    pPrNVCfg->receiptPrintDensity = PRINT_DENSITY_DEFAULT;
    pPrNVCfg->sensorToPheadDistance = SENSOR_TO_PHEAD_DIST_MM;   
  
    PRINTF("Avery Scale Flash Printer Configuration: ########################################\r\n");
    PRINTF("gapSensorBackingVal: %d\r\n", pPrNVCfg->gapSensorBackingVal );
    PRINTF("gapSensorLabelVal: %d\r\n", pPrNVCfg->gapSensorLabelVal );
    PRINTF("labelEdgeVal: %d\r\n", pPrNVCfg->labelEdgeVal );
    PRINTF("labelPrintDensity: %d\r\n", pPrNVCfg->labelPrintDensity );
    PRINTF("paperDetectVal: %d\r\n", pPrNVCfg->paperDetectVal );  
    PRINTF("labelPrintDensity: %d\r\n", pPrNVCfg->labelPrintDensity );
    PRINTF("receiptPrintDensity: %d\r\n", pPrNVCfg->receiptPrintDensity );
    PRINTF("sensorToPheadDistance: %d\r\n", pPrNVCfg->sensorToPheadDistance );
    PRINTF("Avery Scale Flash Printer Configuration: ########################################\r\n");
    PRINTF("\r\n"); 
    PRINTF("\r\n");
}

/******************************************************************************/
/*!   \fn void showAWgConfiguration( nvInternal_t *pNVCfg )                                                       

      \brief
        This function dumps the contents of the weigher configuration to the 
        terminal.        
        
      \author
          Aaron Swift
*******************************************************************************/
void showAWgConfiguration( FlashWeighCfg_t *pNVCfg )
{  
    PRINTF("Avery Scale Weigher Flash Configuration: ########################################\r\n");    
    
    PRINTF("loadCellCalib.zeroLoad: 0x%04X\r\n", pNVCfg->loadCellCalib.zeroLoad );
    PRINTF("loadCellCalib.fullLoadSpan: 0x%04X\r\n", pNVCfg->loadCellCalib.fullLoadSpan );
    PRINTF("loadCellCalib.checksum 0x%04X\r\n", pNVCfg->loadCellCalib.checksum );
    
    PRINTF("gravityData.siteFactor 0x%04X\r\n", pNVCfg->gravityData.siteFactor );
    PRINTF("gravityData.factoryFactor 0x%04X\r\n", pNVCfg->gravityData.factoryFactor );
    PRINTF("gravityData.checksum 0x%04X\r\n", pNVCfg->gravityData.checksum );
    
    PRINTF("filterData.type 0x%01X\r\n", pNVCfg->filterData.type );
    PRINTF("filterData.unused 0x%01X\r\n", pNVCfg->filterData.unused );
    PRINTF("filterData.checksum 0x%02X\r\n", pNVCfg->filterData.checksum );
    
    PRINTF("incOffsets.tCal: 0x%04X\r\n", pNVCfg->incOffsets.tCal );
    PRINTF("incOffsets.xOffset: 0x%04X\r\n", pNVCfg->incOffsets.xOffset );
    PRINTF("incOffsets.yOffset: 0x%04X\r\n", pNVCfg->incOffsets.yOffset );
    PRINTF("incOffsets.checksum: 0x%04X\r\n", pNVCfg->incOffsets.checksum );
    
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.tCal );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.xk1 );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.xk2 );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.xtc );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.yk );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.ytc );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.zeroOffset );
    PRINTF("incCalib 0x%04X\r\n", pNVCfg->incCalib.checksum );


    PRINTF("manufact.quality: 0x%01X\r\n", pNVCfg->manufact.quality );
    PRINTF("manufact.variant: 0x%01X\r\n", pNVCfg->manufact.variant );
    PRINTF("manufact.checksum: 0x%02X\r\n", pNVCfg->manufact.checksum );
    
    PRINTF("creep.creepHi: 0x%04X\r\n", pNVCfg->creep.creepHi );
    PRINTF("creep.creepLo: 0x%04X\r\n", pNVCfg->creep.creepLo );
    PRINTF("creep.timeConstHi: 0x%04X\r\n", pNVCfg->creep.timeConstHi );
    PRINTF("creep.timeConstLo: 0x%04X\r\n", pNVCfg->creep.timeConstLo );
    PRINTF("creep.checksum: 0x%04X\r\n", pNVCfg->creep.checksum );
    
    PRINTF("valib.tUChecksum: 0x%02X\r\n", pNVCfg->valib.tuChecksum );
    PRINTF("valib.vLChecksum: 0x%02X\r\n", pNVCfg->valib.vlChecksum );
    PRINTF("valib.checksum: 0x%04X\r\n", pNVCfg->valib.checksum );
    
    PRINTF("tempOffset.temperatureOffset: 0x%04X\r\n", pNVCfg->tempOffset.temperatureOffset );
    PRINTF("tempOffset.checksum: 0x%04X\r\n", pNVCfg->tempOffset.checksum );
    
    PRINTF("calTemp.calibrationTemperature: 0x%04X\r\n", pNVCfg->calTemp.calibrationTemperature );
    PRINTF("calTemp.checksum: 0x%04X\r\n", pNVCfg->calTemp.checksum );
    
    PRINTF("tempCo.spanTC 0x%04X\r\n", pNVCfg->tempCo.spanTC );
    PRINTF("tempCo.zeroTC 0x%04X\r\n", pNVCfg->tempCo.zeroTC );
    PRINTF("tempCo.checksum 0x%04X\r\n", pNVCfg->tempCo.checksum );
    
    PRINTF("wgcap.balance10: 0x%02X\r\n", pNVCfg->wgCap.balance10 );
    PRINTF("wgcap.balance4: 0x%02X\r\n", pNVCfg->wgCap.balance4 );
    PRINTF("wgcap.cellOutput: 0x%01X\r\n", pNVCfg->wgCap.cellOutput );
    PRINTF("wgcap.cellType: 0x%01X\r\n", pNVCfg->wgCap.cellType );
    PRINTF("wgcap.changeOver: 0x%04X\r\n", pNVCfg->wgCap.changeOver );
    PRINTF("wgcap.decimalPlaces: 0x%01X\r\n", pNVCfg->wgCap.decimalPlaces );
    PRINTF("wgcap.freeTareRange: 0x%04X\r\n", pNVCfg->wgCap.freeTareRange );
    PRINTF("wgcap.fullLoad: 0x%04X\r\n", pNVCfg->wgCap.fullLoad );
    PRINTF("wgcap.increment1: 0x%02X\r\n", pNVCfg->wgCap.increment1 );
    PRINTF("wgcap.increment2: 0x%02X\r\n", pNVCfg->wgCap.increment2 );
    PRINTF("wgcap.powerupBalance: 0x%02X\r\n", pNVCfg->wgCap.powerupBalance );
    PRINTF("wgcap.reserved: 0x%04X\r\n", pNVCfg->wgCap.reserved );
    PRINTF("wgcap.stdTareRange: 0x%04X\r\n", pNVCfg->wgCap.stdTareRange );
    PRINTF("wgcap.unitsOfMeasure: 0x%01X\r\n", pNVCfg->wgCap.unitsOfMeasure );
    PRINTF("wgcap.checksum: 0x%02X\r\n", pNVCfg->wgCap.checksum );
   
    PRINTF("Avery Scale Weigher Flash Configuration: ########################################\r\n");    
}

/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*!   \fn bool getSerialPrConfiguration(Pr_Config *pPrConfig)                                                             
 
      \brief
        This function reads the printer configuration to the serial flash 
        and returns the results of the read operation.

      \param pPrConfig pointer to a printer configuration object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool getSerialPrConfiguration(Pr_Config *pPrConfig)
{
   bool result = false;
    
    /* printer config starts at sector 3, page 2*/
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 2);
    
    if( readSerialFlash(addr, (uint8_t *)pPrConfig, sizeof(Pr_Config)) ) {   
        result = true;
    }
    
    return result;  
}

/******************************************************************************/
/*!   \fn bool setSerialPrConfiguration(Pr_Config *pPrConfig)                                                             
 
      \brief
        This function writes the printer configuration to the serial flash 
        and returns the results of the write operation.

      \param pPrConfig pointer to a printer configuration object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool setSerialPrConfiguration(Pr_Config *pPrConfig)
{
   bool result = false;
    
    /* printer configuration starts at sector 3 page 2 */
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 2 );
    if( writeSerialFlash( (unsigned char *)pPrConfig, addr, sizeof(Pr_Config), SECTOR_3 ) ) {
        result = true;        
    }    
    return result;  
}

/******************************************************************************/
/*!   \fn bool setSerialPrDfltConfiguration( Pr_Config *pPrConfig )                                                              
 
      \brief
        This function set a default configuration to the serial flash 
        and returns the results.

      \param PrConfiguration pointer to a printer configuration object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool setSerialPrDfltConfiguration( Pr_Config *pPrConfig )
{
    Pr_Config oldConfig;
    getSerialPrConfiguration(&oldConfig);
  
    PRINTF("setSerialPrDfltConfiguration()\r\n");
    PRINTF("config pre-default\r\n");
    PRINTF("instance: %d\r\n", pPrConfig->instance );
    PRINTF("label_width: %d\r\n", pPrConfig->label_width );
    PRINTF("shootThroughCal: %d\r\n", pPrConfig->shootThroughCal );
    PRINTF("out_of_media_count: %d\r\n", pPrConfig->out_of_media_count );
    PRINTF("contrast_adjustment: %d\r\n", pPrConfig->contrast_adjustment ); 
    PRINTF("expel_position: %d\r\n", pPrConfig->expel_position );
    PRINTF("peel_position: %d\r\n", pPrConfig->peel_position );
    PRINTF("retract_position: %d\r\n", pPrConfig->retract_position );
    PRINTF("media_sensor_type: %d\r\n", pPrConfig->media_sensor_type );
    PRINTF("printheadResistance: %d\r\n", pPrConfig->printheadResistance );
    PRINTF("verticalPosition: %d\r\n", pPrConfig->verticalPosition );
    PRINTF("backingPaper: %d\r\n", pPrConfig->backingPaper );
    PRINTF("backingAndlabel: %d\r\n", pPrConfig->backingAndlabel );
    PRINTF("labelCalCnts: %d\r\n", pPrConfig->labelCalCnts );
    PRINTF("noLabelCalCnts: %d\r\n", pPrConfig->noLabelCalCnts );
    PRINTF("cutterEnabled: %d\r\n", pPrConfig->cutterEnabled );
    PRINTF("takeupDriveCurrent: %d\r\n", pPrConfig->takeupDriveCurrent );
    PRINTF("takeupMaxTension: %d\r\n", pPrConfig->takeupMaxTension );
    PRINTF("takeupMinTension: %d\r\n", pPrConfig->takeupMinTension );
  
    if (pPrConfig->instance != SECONDARY_PRINTER) {
        pPrConfig->instance = PRIMARY_PRINTER;
    }

#if 0   /* turn this On to simulate blank eep  */
	memset(pPrConfig, 0xff, sizeof(Pr_Config) );
	memset(&oldConfig, 0xff, sizeof(Pr_Config) );
#endif	
	
    pPrConfig->instance			        = 0;
    pPrConfig->label_width                      = UFW_LABEL_STOCK;   /* service scale stock */
    pPrConfig->contrast_adjustment              = 3;
    pPrConfig->peel_position                    = 75;  
    pPrConfig->retract_position                 = -200;         
    pPrConfig->expel_position                   = 15; 
    pPrConfig->printheadResistance              = 0;
    pPrConfig->verticalPosition                 = 34;
    
    pPrConfig->shootThroughCal                  = oldConfig.shootThroughCal; 
    pPrConfig->out_of_media_count               = oldConfig.out_of_media_count;
    
    pPrConfig->backingPaper                     = oldConfig.backingPaper;
    pPrConfig->backingAndlabel                  = oldConfig.backingAndlabel;
    pPrConfig->labelCalCnts                     = oldConfig.labelCalCnts;
    pPrConfig->noLabelCalCnts                   = oldConfig.noLabelCalCnts;

    pPrConfig->takeupDriveCurrent               = CC_SEVEN_POINT_FIVE;
    pPrConfig->takeupMaxTension                 = MIN_TU_CAL_DELTA_CNTS;
    pPrConfig->takeupMinTension                 = MIN_TU_CAL_DELTA_CNTS;
  
    PRINTF("\r\n\r\n");
    PRINTF("setSerialPrDfltConfiguration()\r\n");
    PRINTF("config post-default\r\n");
    PRINTF("instance: %d\r\n", pPrConfig->instance );
    PRINTF("label_width: %d\r\n", pPrConfig->label_width );
    PRINTF("shootThroughCal: %d\r\n", pPrConfig->shootThroughCal );
    PRINTF("out_of_media_count: %d\r\n", pPrConfig->out_of_media_count );
    PRINTF("contrast_adjustment: %d\r\n", pPrConfig->contrast_adjustment ); 
    PRINTF("expel_position: %d\r\n", pPrConfig->expel_position );
    PRINTF("peel_position: %d\r\n", pPrConfig->peel_position );
    PRINTF("retract_position: %d\r\n", pPrConfig->retract_position );
    PRINTF("media_sensor_type: %d\r\n", pPrConfig->media_sensor_type );
    PRINTF("printheadResistance: %d\r\n", pPrConfig->printheadResistance );
    PRINTF("verticalPosition: %d\r\n", pPrConfig->verticalPosition );
    PRINTF("backingPaper: %d\r\n", pPrConfig->backingPaper );
    PRINTF("backingAndlabel: %d\r\n", pPrConfig->backingAndlabel );
    PRINTF("labelCalCnts: %d\r\n", pPrConfig->labelCalCnts );
    PRINTF("noLabelCalCnts: %d\r\n", pPrConfig->noLabelCalCnts );
    PRINTF("cutterEnabled: %d\r\n", pPrConfig->cutterEnabled );
    PRINTF("takeupDriveCurrent: %d\r\n", pPrConfig->takeupDriveCurrent );
    PRINTF("takeupMaxTension: %d\r\n", pPrConfig->takeupMaxTension );
    PRINTF("takeupMinTension: %d\r\n", pPrConfig->takeupMinTension );      
    
    return( setSerialPrConfiguration( pPrConfig ) );
}

/******************************************************************************/
/*!   \fn bool getSerialPrInfo( PrInfo *prInfo )                                                              
 
      \brief
        This function reads the print information stored in the serial flash
        on sector 3 page 5 and returns results of read operation.

      \param PrInfo pointer to a printer info object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool getSerialPrInfo( PrInfo *prInfo )
{
   bool result = false;
   PrInfo info;
    
    /* printer info starts at sector 3, page 5*/
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 5 );
    
    if( readSerialFlash(addr, (uint8_t *)&info, sizeof(PrInfo)) )
    {   
        memcpy( prInfo, &info, sizeof(PrInfo) ); 
        result = true;
    }
    
    return result;    
}

/******************************************************************************/
/*!   \fn bool getSerialPrInfo( PrInfo *prInfo )                                                              
 
      \brief
        This function is only executed once at time of manufacture. The firmware
        will detect if a cutter is installed and set and save the print information.
        THis is being done because the cutter install bit electrically is not reliable 
        when the interlock is power. When the interlock is opened, power is cut
        to  the PIC controller on the cutter assembly and the install pin is then 
        in the not installed state.  
       
      \param PrInfo pointer to a printer info object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool setSerialPrInfo( PrInfo *prInfo )
{
    bool result = false;
    
    /* printer configuration starts at sector 3 page 5 */
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 5 );
    if( writeSerialFlash( (unsigned char *)prInfo, addr, sizeof(PrInfo), SECTOR_3 ) )
    {
        result = true;        
    }    
    return result;      
}

/******************************************************************************/
/*!   \fn bool eraseSerialCutterBit( void )                                                             
 
      \brief
        This function erases the print information stored in the serial flash
        on sedtor 3 page 5 and returns results of erase operation.

      \param 

      \author
          Aaron Swift
*******************************************************************************/ 
bool eraseSerialCutterBit( void )
{
    bool result = false;
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 5 );   
    /* erase the page */
    if( erasePage(addr, SECTOR_3) ) {
        result = true;    
    }
    return result;
}

/******************************************************************************/
/*!   \fn bool getPageChecksums(FPMBLC3Checksums *pChecksums)                                                            
 
      \brief
        This function reads the section checksums stored in the serial flash
        on sector 3 page 4 and returns results of read operation.

      \param pChecksums pointer to a checksum object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool getPageChecksums(FPMBLC3Checksums *pChecksums)
{
    bool result = false;
    
   /* page checksums starts at sector 3, page 4*/
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 4 );
    FPMBLC3Checksums temp;
    
    if( readSerialFlash(addr, (uint8_t *)&temp, sizeof(FPMBLC3Checksums)) )
    {   
        memcpy( pChecksums, &temp, sizeof(FPMBLC3Checksums) ); 
        result = true;
    }
    
    return result;    
}

/******************************************************************************/
/*!   \fn bool getPageChecksums(FPMBLC3Checksums *pChecksums)                                                            
 
      \brief
        This function writes the section checksums stored in the serial flash
        on sedtor 3 page 4 and returns results of write operation.

      \param pChecksums pointer to a checksum object.

      \author
          Aaron Swift
*******************************************************************************/ 
bool setPageChecksums(FPMBLC3Checksums *pChecksums)
{

   bool result = false;
    
    /* page checksums starts at sector 3 page 4 */
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 4 );
    if( writeSerialFlash( (unsigned char *)pChecksums, addr, sizeof(FPMBLC3Checksums), SECTOR_3 ) )
    {
        result = true;        
    }    
    return result;   
}

/******************************************************************************/
/*!   \fn unsigned short calculateChecksum( void *buffer, unsigned long size )                                                           
 
      \brief
 
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short calculateChecksum( void *buffer, unsigned long size )
{
    unsigned short accumulator = 0;
    unsigned long i;
    unsigned char *this_byte;
    
    this_byte = buffer;
    
    for (i = 0; i < size; i++)
    {
        accumulator += *this_byte++;
    }
    
    return (accumulator);
}


void testConfiguration()
{
#if 0
    WgConfiguration config;
    WgConfiguration sConfig;
    WeigerMFGInfo info; 
    unsigned char partNum[PART_NUM_SIZE] = {'0','0','-','4','4','7','1','5','2'};
    unsigned char rev[BOARD_REV_SIZE] = {'A','0'};
    unsigned char serialNum[SERIAL_NUM_SIZE] =  {'4','5','-','1','2','3','4','-','5','6','7'};
    memset( &config, 0, sizeof(WgConfiguration) );
    memset( &sConfig, 0, sizeof(WgConfiguration ));
    memset( &info, 0, sizeof(WeigerMFGInfo) );
   
    setSerialWgDfltConfig(&config);   
    getSerialWgConfiguration(&sConfig);
    setSectorLock( SECTOR_PROTECT_3 );  
    info.readOnly = true;          
    memcpy( &info.partNumber[0], &partNum[0], PART_NUM_SIZE );
    memcpy( &info.boardRevision[0], &rev[0], BOARD_REV_SIZE );
    memcpy( &info.serialNumber[0], &serialNum[0], SERIAL_NUM_SIZE ); 
    info.loadCellType = LOADCELL_TI1232; 
    info.checkSum = 12345;
    
    setSerialWeigherMFGInfo(&info);
    getSerialWeigherMFGInfo(&info);    
    setSectorLock( SECTOR_PROTECT_3 );  
#else
        /* added for test erase the weigher config */    
        unsigned long addr = SECTOR3_BASE_ADDR;
        erasePage(addr, SECTOR_3);

        /* added for test erase the printer config */
        addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 2);
        erasePage(addr, SECTOR_3);

#endif
}
