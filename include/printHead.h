#ifndef PRINTHEAD_H
#define PRINTHEAD_H
#include <stdbool.h>
#include "fsl_common.h"
#include "fsl_common_arm.h"

typedef enum
{
    UNKNOWN_HEAD,
    TDK753_OHM,
    TDK800_OHM,
    TDK849_OHM,
    KYOCERA753_OHM,
    KYOCERA800_OHM,
    KYOCERA849_OHM,
    ROHM_72MM_800_OHM,
    ROHM_80MM_650_OHM,
    ROHM_56MM_GF41A
}HeadType_t;

typedef enum
{
    UNKNOWN_MANUFACTURE,
    TDK,
    KYOCERA_GLAZE_CUSTOM,
    KYOCERA_GLAZE_THIN
}HEAD_MANUFACTURE;

#define TDK_HEAD_CUSTOM_GLAZE_MIN       0      /* 0v +- 10% resolution .01289 */
#define TDK_HEAD_CUSTOM_GLAZE_MAX       10
#define KYOCERA_THIN_GLAZE_MIN          154    /* 2.2v +- 10% resolution .01289 */
#define KYOCERA_THIN_GLAZE_MAX          188
#define KYOCERA_CUSTOM_GLAZE_MIN        221    /* 3.16v +- 10% resolution .01289 */    
#define KYOCERA_CUSTOM_GLAZE_MAX        270

#define MAX_TABLE_ENTRIES       7
#define AVERAGE_TEMP_INDEX      3
#define MID_RANGE_CONTRAST      3
#define MAX_CONTRAST            7

/* steps per label size "inches" */ 
#define STEPS_PER_LENGTH1_00            203
#define STEPS_PER_LENGTH1_50            304
#define STEPS_PER_LENGTH1_75            355
#define STEPS_PER_LENGTH2_00            406
#define STEPS_PER_LENGTH2_37            481
#define STEPS_PER_LENGTH2_50            508
#define STEPS_PER_LENGTH3_00            609
#define STEPS_PER_LENGTH3_50            711
#define STEPS_PER_LENGTH4_00            812
#define STEPS_PER_LENGTH4_50            914
#define STEPS_PER_LENGTH5_00            1015
#define STEPS_PER_LENGTH5_50            1117
#define STEPS_PER_LENGTH6_00            1218
#define STEPS_PER_LENGTH6_50            1320
#define STEPS_PER_LENGTH7_00            1421
#define STEPS_PER_LENGTH7_50            1523
#define STEPS_PER_LENGTH8_00            1624
#define STEPS_PER_LENGTH8_50            1726
#define STEPS_PER_LENGTH9_00            1827
#define STEPS_PER_LENGTH9_50            1929
#define STEPS_PER_LENGTH10_00           2030

/* steps per label size "inches" */             //normal printhead cover, peeling, streaming
#define STEPS_PER_LENGTH1_00_HT /*0*/           225 //dont have
#define STEPS_PER_LENGTH1_50_HT /*1*/           300 //dont have 
#define STEPS_PER_LENGTH1_75_HT /*2*/           375 //415 //425
#define STEPS_PER_LENGTH2_00_HT /*3*/           450 //dont have
#define STEPS_PER_LENGTH2_37_HT /*4*/           525 //550 //560
#define STEPS_PER_LENGTH2_50_HT /*5*/           600 //dont have
#define STEPS_PER_LENGTH3_00_HT /*6*/           640 //675 //685
#define STEPS_PER_LENGTH3_50_HT /*7*/           740 //780 //800
#define STEPS_PER_LENGTH4_00_HT /*8*/           840 //875 //900
#define STEPS_PER_LENGTH4_50_HT /*9*/           940 //dont have
#define STEPS_PER_LENGTH5_00_HT /*10*/          1040 //1090 //1120
#define STEPS_PER_LENGTH5_50_HT /*11*/          1140 //1190 //1240
#define STEPS_PER_LENGTH6_00_HT /*12*/          1200 //dont have
#define STEPS_PER_LENGTH6_50_HT /*13*/          1300 //1375 //1410
#define STEPS_PER_LENGTH7_00_HT /*14*/          1450 //1530 //1570
#define STEPS_PER_LENGTH7_50_HT /*15*/          1550 //1600 //1645
#define STEPS_PER_LENGTH8_00_HT /*16*/          1650 //1720 //1761
#define STEPS_PER_LENGTH8_50_HT /*17*/          1750 //1800 //1850
#define STEPS_PER_LENGTH9_00_HT /*18*/          1850 //1900 //1940     
#define STEPS_PER_LENGTH9_50_HT /*19*/          1970 //2030 //2080      
#define STEPS_PER_LENGTH10_00_HT /*20*/         2110 //2110 //2160      

#define N_PRINTER_LINES_80MM				640   /* 72mm => 704 = printer lines per buffer 3.5" label length */ //80mm
#define HALF_IMAGE_BUFFER_LINE_COUNT_80MM	320   /* Image buffer is updated by backend in halves */

#define N_PRINTER_LINES_72MM				704   /* 711 = printer lines per buffer 3.5" label length */           //72mm
#define HALF_IMAGE_BUFFER_LINE_COUNT_72MM	384   /* Image buffer is updated by backend in halves */
	   
/* we will be supporting 3 different printheads */
#define PRINTER_HEAD_SIZE_80MM          80      /* bytes */
#define PRINTER_HEAD_SIZE_72MM          72      /* bytes */
#define PRINTER_HEAD_SIZE_56MM          56      /* bytes */
//#define PRINTER_HEAD_SIZE_80MM          56      /* bytes */
#define HEAD_DOTS_56MM                  448
#define HEAD_DOTS_72MM                  576     
#define HEAD_DOTS_80MM                  640  
//#define HEAD_DOTS_80MM                  448

#define PRINTER_HEAD_SIZE_PREPACK       85

/* potential fix for sof-5965 change n_print_lines to 704 when ready */
#define PRINTER_BUFFER_SIZE_72MM        50688   /* divisable by 512 and 72!!! = 704 lines*/      
#define PRINTER_BUFFER_SIZE_80MM        51200   /* 80 transfers at 640 = 51200 */        

#define PH_SPI_INT_PRIORITY             1

typedef enum
{
    RT_SERVICE_72MM,
    RT_SERVICE_80MM,   
    K_PREPACK_80MM
}PrinterEnv;

typedef struct
{
    unsigned char headType;
    unsigned short headSize;	   
}HeadStyle;

typedef struct
{
    unsigned char *pBottom;                     /* pointer to bottom of image buffer */
    unsigned char *pTop;                        /* pointer to top of image buffer */
    unsigned long bufferSize;                   /* overall size of image buffer */
    unsigned long numXfersToFill;               /* how many usb transfers to fill image buffer */
    unsigned long index;                        /* image buffer index */
    unsigned long remainder;                    /* last bit of image buffer to fill */
    unsigned long labelImageSize;               /* size of the label image to print in bytes */
    unsigned long rollOverOffset;               /* since our image buffer is not a multiple of 512 */
    bool          rollover;                     /* will the label image cause a rollover of the buffer */
}ImageBfrMgr;

void initializeImageBfr( ImageBfrMgr *pMgr );
void intializePrintHead( unsigned int contrast,  PrinterEnv env );

AT_QUICKACCESS_SECTION_CODE( void copyBulkToImageBfr( unsigned char *pBulk, unsigned int size ) );
void setLabelImageSizeMgr( unsigned long size );
void clearHistory( PrinterEnv env );
void initializePrintHeadSPI( void );
void initializeDma( void );
unsigned char *getImageBuffer( void );
int getImageBufferSize( void );
AT_QUICKACCESS_SECTION_CODE( HeadType_t getPrintHeadType( void ) );

unsigned short getHalfSltTime( void );
unsigned int getSltTime( HeadType_t type, unsigned int contrast );
unsigned int getSltSizingTime( HeadType_t type );
unsigned int getCompLevel( HeadType_t type );


AT_QUICKACCESS_SECTION_CODE(int getPrintheadTemperatureInCelsius( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned char getHeadStyleType( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getHeadStyleSize( void ));
AT_QUICKACCESS_SECTION_CODE(void setHeadTimings( void ));
AT_QUICKACCESS_SECTION_CODE(void updatePrintTimings( void ));
AT_QUICKACCESS_SECTION_CODE(void updateSLTTime( uint16_t nextSpeed ));
AT_QUICKACCESS_SECTION_CODE(void setHeadPower( bool state ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getPwmStartTime( unsigned char index ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getPwmDutyCycle( unsigned char index ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getHistoryTime( unsigned char index ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getAdjacencyTime( unsigned char index ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getCurrentLineTime( unsigned char index ));
unsigned long byteSwapLong( unsigned long data );
AT_QUICKACCESS_SECTION_CODE(void history( unsigned char *pLine ));
AT_QUICKACCESS_SECTION_CODE(unsigned char *getCurrentPrintDataLine( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned char *getPreviousPrintDataLine( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned char *getNextPrintDataLine( void ));
AT_QUICKACCESS_SECTION_CODE(void resetPrintDataLine( void ));
void initializeSelfCheck( void );
AT_QUICKACCESS_SECTION_CODE(unsigned char *getFirstHistoryLine( void ));
void testHeadWear( void );
void showHeadType( void );
#endif