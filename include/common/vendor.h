#ifndef VENDOR_H
#define VENDOR_H
#include <stdbool.h>

/****************************** hobart defines *********************************/
/******************************************************************************/

/* product id's */
#define GLOBAL_SCALE_HB_GT_PROD_ID              0x260   /* global service scale board */
#define GLOBAL_SCALE_HB_DT_PROD_ID              0x270   /* global dt service scale board */
#define GLOBAL_SCALE_HB_PRINTER_PROD_ID         0x280   /* global printer */
#define GLOBAL_SCALE_HB_WEIGHER_PROD_ID         0x290   /* global weigher */
/******************************************************************************/
/******************************************************************************/

/****************************** avery defines *********************************/
/******************************************************************************/

#define GLOBAL_SCALE_AV_XPRO_PROD_ID            0x7573   
#define GLOBAL_SCALE_AV_XONE_PROD_ID            0x7572
#define GLOBAL_SCALE_AV_WEIGHER_PROD_ID         0x7570
#define GLOBAL_SCALE_AV_PRINTER_PROD_ID         0x7571     
      
      
#define SEPARATOR "\0"
#define PRINTER_APP_VERSION_NUM  "[MAJOR_VERSION].[MINOR_VERSION].[REVISION]"
#define PRINTER_APP_FIRMWARE_NUM "ABR30-000485 v[MAJOR_VERSION].[MINOR_VERSION].[REVISION].[GIT_CURRENT_HASH][GIT_COMMITS_FLAG][GIT_DIRTY_FLAG]"

#define WEIGHER_APP_VERSION_NUM  "[MAJOR_VERSION].[MINOR_VERSION].[REVISION]"
#define WEIGHER_APP_FIRMWARE_NUM "ABR30-000485 v[MAJOR_VERSION].[MINOR_VERSION].[REVISION].[GIT_CURRENT_HASH][GIT_COMMITS_FLAG][GIT_DIRTY_FLAG]"

#define BOOT_PRINTER_VERSION    "[MAJOR_VERSION].[MINOR_VERSION].[REVISION]"
#define BOOT_WEIGHER_VERSION    "[MAJOR_VERSION].[MINOR_VERSION].[REVISION]"

/* static version string definition */
#define STATIC_PRINTER_VERSION_STRING \
    "NGS\0"                           \
    "1.0.99\0"                         \
    "ABR30-000485 v1.0.99\0" __DATE__ "\0" __TIME__

#define STATIC_BOOT_PRINTER_VERSION_STRING \
    "NGS\0"                           \
    "1.0.99\0"                         \
    "ABR30-000486 v1.0.99\0" __DATE__ "\0" __TIME__
      
#define STATIC_WEIGHER_VERSION_STRING \
    "NGS\0"                           \
    "1.0.0\0"                         \
    "ABR30-000497 v1.0.0\0" __DATE__ "\0" __TIME__

/* static boot version string definition */
#define STATIC_BOOT_WEIGHER_VERSION_STRING \
    "NGS\0"                                \
    "1.0.0\0"                              \
    "ABR30-000498 v0.0.0\0" __DATE__ "\0" __TIME__

typedef struct
{
    char *number;
    char *firmware;
    char *date;
    char *time;
} version_t;

typedef enum
{
    AV_VERSION_APP,
    AV_VERSION_BOOT 
}AV_VERSION_TYPE_t;

#define OLD_FIRMWARE_LEN		39
#define REQ_APP_VERSION                 0x50
#define REQ_BOOT_VERSION                0x51
typedef struct 
{
    unsigned char oldFirmwareNumber[40];
    unsigned char *pCreationDate;
    unsigned char *pCreationTime;
    unsigned char *pFrimwareNumber;  
}VersionInfo_t;
/******************************************************************************/
/******************************************************************************/




/************************** hobart's public functions *************************/
/******************************************************************************/
unsigned short getProductId( void );
unsigned char getPrinterSoftwareIDMajor( void );
unsigned char getPrinterSoftwareIDMinor( void );
unsigned char getPrinterSoftwareIDEng( void );
unsigned char getWeigherSoftwareIDMajor( void );
unsigned char getWeigherSoftwareIDMinor( void );
unsigned char getWeigherSoftwareIDEng( void );
unsigned char getBootVerMajor( void );
unsigned char getBootVerMinor( void );
unsigned char getBootVerEng( void );
unsigned char getHardwareIDMajor( void );
unsigned char getHardwareIDMinor( void );
unsigned char getPad1( void );
unsigned long long getAppVectorStart( void );
void printSWVersions(void);
/******************************************************************************/
/******************************************************************************/




/*************************** avery public functions **************************/
/******************************************************************************/
unsigned char getAssetNumber( void );
void get_avery_pr_app_version(  VersionInfo_t *pVersion );
void get_avery_wg_app_version( VersionInfo_t *pVersion );
void get_avery_wg_boot_version( VersionInfo_t *pVersion );
version_t get_version(void *version_string);
unsigned char getHardwareVer(void);
bool isTiltEnabled(void);
/******************************************************************************/
/******************************************************************************/
#endif
