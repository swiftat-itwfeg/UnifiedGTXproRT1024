#ifndef DEVICEPROPERTIES_H
#define DEVICEPROPERTIES_H


typedef enum
{
    UNKNOWN_CLASS,
    AVERY_WEIGHER_PRINTER,
    AVERY_WEIGHER,
    AVERY_PRINTER,
    HOBART_WEIGHER_PRINTER,
    HOBART_WEIGHER,
    HOBART_PRINTER,
    UN_WEIGHER_PRINTER,
    UN_WEIGHER,
    UN_PRINTER,
    BLANK_CLASS         = 255
}DEVICECLASS_t;

typedef enum
{
    UNKNOWN_SUBCLASS,
    AV_XPRO,
    AV_XONE,
    AV_WEIGHER,
    AV_PRINTER,
    HB_GT,
    HB_DT,
    HB_WEIGHER,
    HB_PRINTER,
    UNIFIED_WEIGHER_PRINTER,
    UNIFIED_WEIGHER,
    UNIFIED_PRINTER,
    BLANK_SUBCLASS      = 255
}DEVICESUBCLASS_t;


typedef struct
{
    DEVICECLASS_t       class; 
    DEVICESUBCLASS_t    subClass; 
    unsigned char       versionMajor;
    unsigned char       versionMinor;
    unsigned char       versionEng;
}DEVICE_PROPERTIES_t;

/* different scale models this firmware supports */ 
typedef enum
{
    GLOBAL_SCALE_UNKNOWN,
    GLOBAL_SCALE_AV_XPRO,
    GLOBAL_SCALE_AV_XONE,
    GLOBAL_SCALE_AV_WEIGHER,
    GLOBAL_SCALE_AV_PRINTER,
    GLOBAL_SCALE_HB_GT,
    GLOBAL_SCALE_HB_DT,
    GLOBAL_SCALE_HB_WEIGHER,
    GLOBAL_SCALE_HB_PRINTER
}PeripheralModel;
#endif