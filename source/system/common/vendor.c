#include "vendor.h"
#include "threadManager.h"
#include "fsl_debug_console.h"
#include "developmentSettings.h"

    

const char buildDate[16] =  __DATE__;
const char buildTime[16] =  __TIME__;
  
#pragma location = "VENDOR_PSW_MAJOR"    /* version string: x.x.x */
const unsigned char printerSwM = 1; 
#pragma location = "VENDOR_PSW_MINOR"
const unsigned char printerSwm = 4; 
#pragma location = "VENDOR_PSW_ENG"
const unsigned char printerSwe = 15;
#pragma location = "VENDOR_WSW_MAJOR"    /* version string: x.x.x */
const unsigned char weigherSwM = 1; 
#pragma location = "VENDOR_WSW_MINOR"
const unsigned char weigherSwm = 2;
#pragma location = "VENDOR_WSW_ENG"
const unsigned char weigherSwe = 2;
#pragma location = "VENDOR_PAD1"
const unsigned char pad1 = 0xFF;
#pragma location = "VENDOR_HW_MAJOR"   
const unsigned char deviceHwM = 0;   /* no longer used */
#pragma location = "VENDOR_HW_MINOR"
const unsigned char deviceHwm = 0;    /* no longer used */
#pragma location="VENDOR_APP_START"
const unsigned long appVectorStart = 0x60040000;


/******************** printer revision history ********************************/
/*  1.0.8: Changed label taken threshold and removed changes for 10K pull     */
/*         down bias.                                                         */
/*  1.0.11 Support bias change to label taken sensor.                         */
/*  1.0.12 Flip strobe pwm from high to low as per Hung ( energy reduction )  */
/*         added expel to print operation for reliable expel distance.        */
/*  1.0.13 Paper takeup at end of print cycle.                                */
/*  1.0.14 Missing print line after print buffer rollover. Chris's paper      */
/*         takeup and MOC values                                              */
/*  1.0.15 Merge of chris's paper takeup                                      */
/*  1.0.17 Weihger usb fix for stall on bootup?                               */
/*  1.0.18 White line on label types over 3.5" ( rollover )                   */
/*  1.0.19 Chris's takeup changes and filter sending weigher status as to     */
/*         not spam the backend with weigher status                           */
/*  1.0.20 Timer not initialized in Weigher autozero and intializeZero        */
/*         functions                                                          */
/*  1.0.21 Timer not initialized in small or large motion detection           */
/*  1.0.22 Backend crash when cassette removed on boot of the scale           */
/*  1.0.24 80mm support                                                       */
/*  1.0.34 03/20/2024                                                         */
/*                                                                            */
/******************************************************************************/


/******************** printer revision history ********************************/
/** ver  date    time   engr     description                                 **/
/*  1.2 5/14/24         T Fink   1st Released version. Tested at NTEP         */
/*  1.3 5/21/24 ~11:06  T Fink   2nd Released version. Major code cleanup &   */
/*      logic rewrite but the code that does calculations didn't change       */
/******************************************************************************/
#pragma required=printerSwM
#pragma required=printerSwm
#pragma required=printerSwe
#pragma required=weigherSwM
#pragma required=weigherSwm
#pragma required=weigherSwe
#pragma required=pad1
#pragma required=deviceHwM
#pragma required=deviceHwm
#pragma required=appVectorStart


void printSWVersions(void)
{
    PRINTF("Build Date: %s\r\n", buildDate);
    PRINTF("Build Time: %s\r\n", buildTime);
    PRINTF("PrinterSWVersion: %d.%d.%d\r\n",printerSwM,printerSwm,printerSwe);
    PRINTF("WeigherSWVersion: %d.%d.%d\r\n",weigherSwM,weigherSwm,weigherSwe); 
}

/* hobart product id's */
#define G_SCALE_UNKNOWN_PRODUCT_ID      0x0999   /* unknown scale product id */
#define G_SCALE_GT_PRODUCT_ID           0x2000   /* hobart global scale gt model*/
#define G_SCALE_DT_PRODUCT_ID           0x2010   /* hobart global scale dt model*/
#define G_SCALE_WG_PRODUCT_ID           0x2011   /* hobart global weigher only model*/
#define G_SCALE_PR_PRODUCT_ID           0x2012   /* hobart global printer only model*/


/* avery's product id's */
#define G_SCALE_XPRO_PRODUCT_ID         0X7573
#define G_SCALE_XONE_PRODUCT_ID         0X7574  
#define G_SCALE_XTI_PRODUCT_ID          0X7570  /* avery weigher only: t103 and t103+d */
#define G_SCALE_XTRA_PRODUCT_ID         0x7571  /* avery printer only */ 
#define G_SCALE_XTRA_C_PRODUCT_ID       0x7572  /* avery printer only clamshell*/

/****************************** avery defines *********************************/
/******************************************************************************/
char *printer_version_string = {
	"NGS"                     SEPARATOR 
	PRINTER_APP_VERSION_NUM   SEPARATOR
	PRINTER_APP_FIRMWARE_NUM  SEPARATOR
	__DATE__                  SEPARATOR
	__TIME__
};

char *weigher_version_string = { 
	"NGS"                     SEPARATOR 
	WEIGHER_APP_VERSION_NUM   SEPARATOR 
	WEIGHER_APP_FIRMWARE_NUM  SEPARATOR 
	__DATE__                  SEPARATOR
	__TIME__
};

char *boot_version_string = {
	"NGS"                     SEPARATOR 
	BOOT_PRINTER_VERSION      SEPARATOR
	BOOT_WEIGHER_VERSION      SEPARATOR
	__DATE__                  SEPARATOR
	__TIME__
};
char *static_printer_version_string = STATIC_PRINTER_VERSION_STRING;
char *static_boot_printer_version =   STATIC_BOOT_PRINTER_VERSION_STRING;
char *static_weigher_version_string = STATIC_WEIGHER_VERSION_STRING;
char *static_boot_weigher_version   = STATIC_BOOT_WEIGHER_VERSION_STRING;
/******************************************************************************/
/******************************************************************************/

unsigned short getProductId( void )
{
    unsigned short id = 0;
    /* our product id is based off of which model we are */
    PeripheralModel_t model = getMyModel();  

    switch( model )
    {   
        case GLOBAL_SCALE_UNKNOWN: {
            id = 0;
            break;
        }
        case GLOBAL_SCALE_AV_XPRO: {
            id = GLOBAL_SCALE_AV_XPRO_PROD_ID;
            break;          
        }
        case GLOBAL_SCALE_AV_XONE: {
            id = GLOBAL_SCALE_AV_XONE_PROD_ID;
            break;          
        }
        case GLOBAL_SCALE_AV_WEIGHER: {
            id = GLOBAL_SCALE_AV_WEIGHER_PROD_ID;
            break;
        }
        case GLOBAL_SCALE_AV_PRINTER: {
            id = GLOBAL_SCALE_AV_PRINTER_PROD_ID;
            break;
        }
        case GLOBAL_SCALE_HB_GT: {
            id = GLOBAL_SCALE_HB_GT_PROD_ID;
            break;
        }
        case GLOBAL_SCALE_HB_DT: {
            id = GLOBAL_SCALE_HB_DT_PROD_ID;
            break;
        }
        case GLOBAL_SCALE_HB_WEIGHER: {
            id = GLOBAL_SCALE_HB_WEIGHER_PROD_ID; 
            break;
        }
        case GLOBAL_SCALE_HB_PRINTER: {
            id = GLOBAL_SCALE_HB_PRINTER_PROD_ID; 
            break;
        }        
        default: {
            PRINTF("getProductId(): Error model %d unsupported!!\r\n", model);
            id = 0;
        }
    }    
    return id;
}


unsigned char getPrinterSoftwareIDMajor( void )
{
    return printerSwM;
}

unsigned char getPrinterSoftwareIDMinor( void )
{
    return printerSwm;
}

unsigned char getPrinterSoftwareIDEng( void )
{
    return printerSwe;
}

unsigned char getWeigherSoftwareIDMajor( void )
{
    return weigherSwM;
}

unsigned char getWeigherSoftwareIDMinor( void )
{
    return weigherSwm;
}

unsigned char getWeigherSoftwareIDEng( void )
{
    return weigherSwe;
}

unsigned char getHardwareIDMajor( void )
{
    return deviceHwM;  
}
unsigned long long getAppVectorStart( void )
{
    return appVectorStart;
}


unsigned char getHardwareIDMinor( void )
{
    return deviceHwm;
}

unsigned char getPad1()
{
    return pad1;
}

unsigned char getAssetNumber( void )
{
  return (unsigned char) (
      ( GPIO_PinRead( MODEL_TYPE_D_GPIO, MODEL_TYPE_D_PIN ) << 3 ) 
    + ( GPIO_PinRead( MODEL_TYPE_C_GPIO, MODEL_TYPE_C_PIN ) << 2 )                           
    + ( GPIO_PinRead( MODEL_TYPE_B_GPIO, MODEL_TYPE_B_PIN ) << 1 ) 
    + ( GPIO_PinRead( MODEL_TYPE_A_GPIO, MODEL_TYPE_A_PIN ) << 0 ) );  
}

void get_avery_pr_app_version(  VersionInfo_t *pVersion )
{
    version_t v_app = get_version( static_printer_version_string );
    pVersion->pCreationDate = (unsigned char *)v_app.date;
    pVersion->pCreationTime = (unsigned char *)v_app.time;
    pVersion->pFrimwareNumber = (unsigned char *)v_app.firmware;
  
}

void get_avery_pr_boot_version(  VersionInfo_t *pVersion )
{
    version_t v_app = get_version( static_boot_printer_version );
    pVersion->pCreationDate = (unsigned char *)v_app.date;
    pVersion->pCreationTime = (unsigned char *)v_app.time;
    pVersion->pFrimwareNumber = (unsigned char *)v_app.firmware;  
}

void get_avery_wg_app_version( VersionInfo_t *pVersion )
{
    version_t v_app = get_version( static_weigher_version_string );
    pVersion->pCreationDate = (unsigned char *)v_app.date;
    pVersion->pCreationTime = (unsigned char *)v_app.time;
    pVersion->pFrimwareNumber = (unsigned char *)v_app.firmware;
}

void get_avery_wg_boot_version( VersionInfo_t *pVersion )
{
    version_t v_app = get_version( static_boot_weigher_version );
    pVersion->pCreationDate = (unsigned char *)v_app.date;
    pVersion->pCreationTime = (unsigned char *)v_app.time;
    pVersion->pFrimwareNumber = (unsigned char *)v_app.firmware;  
}

version_t get_version(void *version_string)
{
    version_t version;
    static char *undefined = "\0";
    
    version.number = version.firmware = version.date = version.time = undefined;
	
    if( version_string != NULL ) {
        char *p = (char *)version_string;
        /* check if the string list begin by the 'key'. */ 
        /* don't seach inside code or erased flash area */
        if( strcmp(p, "NGS") == 0 ) {
            /* skip this key */
            int l = strlen(p);
            p += (l + 1);
            l = strlen(p);
            if( l > 0 ) {
                /* get the version number */
                version.number = p++;
                p += l;
                l = strlen(p);
                if( l > 0 ) {
                    /* get the firmware number */
                    version.firmware = p++;
                    p += l;
                    l = strlen(p);
                    if( l > 0 ) {
                        /* get the build date */
                        version.date = p++;
                        p += l;
                        l = strlen(p);
                        if( l > 0 ) {
                            /* get the build time */
                            version.time = p++;
                        }
                    }
                }
            }
        }
    }
    return version;  
}
