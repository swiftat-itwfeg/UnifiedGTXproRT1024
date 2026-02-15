#include "serialFlash.h"
#include "w25x10cl.h"
#include "cat3Audit.h"
#include "globalPrinter.h"
#include "globalWeigher.h"
#include "weigher.h"
#include "threadManager.h"
#include "vendor.h"
#include "semphr.h"
#include "fsl_debug_console.h"
#include <string.h>
#include "printHead.h"
#include "lp5521.h"
#include "takeupMotor.h"

static SemaphoreHandle_t sMutex_;

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

bool getLockSerialFlash( void )
{
    bool result = false;
    if( xSemaphoreTake( sMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) { 
        result = true;  
    }
    return result;    
}



void releaseLockSerialFlash( void )
{
    xSemaphoreGive( sMutex_ );  
}



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
   //Pr_Config tmpCfg;
    
    /* printer config starts at sector 3, page 2*/
    unsigned long addr = SECTOR3_BASE_ADDR + ( PAGE_SIZE * 2);
    
    if( readSerialFlash(addr, (uint8_t *)pPrConfig, sizeof(Pr_Config)) )
    {   
        /*if we have read the data without errors then get the data and copy 
        pTmpCfg = ( Pr_Config * )getReadData(4);  */

        //if( pTmpCfg != NULL )
        //{
            //memcpy( pPrConfig, pTmpCfg, sizeof(Pr_Config) ); 
            result = true;
        //}
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
    if( writeSerialFlash( (unsigned char *)pPrConfig, addr, sizeof(Pr_Config), SECTOR_3 ) )
    {
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
    PRINTF("media_sensor_adjustment: %d\r\n", pPrConfig->media_sensor_adjustment );
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
    PRINTF("takeup_sensor_drive_current: %d\r\n", pPrConfig->takeup_sensor_drive_current );
    PRINTF("takeup_sensor_max_tension_counts: %d\r\n", pPrConfig->takeup_sensor_max_tension_counts );
    PRINTF("takeup_sensor_min_tension_counts: %d\r\n", pPrConfig->takeup_sensor_min_tension_counts );
  
    if (pPrConfig->instance != SECONDARY_PRINTER) {
        pPrConfig->instance = PRIMARY_PRINTER;
    }

#if 0//turn this On to simulate blank eep 
	memset(pPrConfig, 0xff, sizeof(Pr_Config) );
	memset(&oldConfig, 0xff, sizeof(Pr_Config) );
#endif	
	
	pPrConfig->instance							= 0;
    pPrConfig->label_width                      = UFW_LABEL_STOCK;   /* service scale stock */
    pPrConfig->contrast_adjustment              = 3;
    pPrConfig->peel_position                    = 75;  
    pPrConfig->retract_position                 = -200;         
    pPrConfig->expel_position                   = 15; 
    pPrConfig->printheadResistance              = 0;
    pPrConfig->verticalPosition                 = 34;
    
    pPrConfig->media_sensor_adjustment          = oldConfig.media_sensor_adjustment; 
    pPrConfig->out_of_media_count               = oldConfig.out_of_media_count;
    
    pPrConfig->backingPaper                     = oldConfig.backingPaper;
    pPrConfig->backingAndlabel                  = oldConfig.backingAndlabel;
    pPrConfig->labelCalCnts                     = oldConfig.labelCalCnts;
    pPrConfig->noLabelCalCnts                   = oldConfig.noLabelCalCnts;

	pPrConfig->takeup_sensor_drive_current      = CC_SEVEN_POINT_FIVE;
    pPrConfig->takeup_sensor_max_tension_counts = MIN_TU_CAL_DELTA_CNTS;
    pPrConfig->takeup_sensor_min_tension_counts = MIN_TU_CAL_DELTA_CNTS;
  
    /*
    pPrConfig->media_sensor_adjustment          = 0; 
    pPrConfig->out_of_media_count               = 0;
    pPrConfig->backingPaper                     = 0;
    pPrConfig->backingAndlabel                  = 0;
    pPrConfig->labelCalCnts                     = 0;
    pPrConfig->noLabelCalCnts                   = 0;
    pPrConfig->takeup_sensor_drive_current      = 0;
    pPrConfig->takeup_sensor_max_tension_counts = 0;
    pPrConfig->takeup_sensor_min_tension_counts = 0;
    */
    
    PRINTF("\r\n\r\n");
    PRINTF("setSerialPrDfltConfiguration()\r\n");
    PRINTF("config post-default\r\n");
    PRINTF("instance: %d\r\n", pPrConfig->instance );
    PRINTF("label_width: %d\r\n", pPrConfig->label_width );
    PRINTF("media_sensor_adjustment: %d\r\n", pPrConfig->media_sensor_adjustment );
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
    PRINTF("takeup_sensor_drive_current: %d\r\n", pPrConfig->takeup_sensor_drive_current );
    PRINTF("takeup_sensor_max_tension_counts: %d\r\n", pPrConfig->takeup_sensor_max_tension_counts );
    PRINTF("takeup_sensor_min_tension_counts: %d\r\n", pPrConfig->takeup_sensor_min_tension_counts );      
    
    return( setSerialPrConfiguration( pPrConfig ) );
}
#if 1
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
#endif
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
