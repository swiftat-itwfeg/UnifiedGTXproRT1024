#ifndef LABEL_H
#define LABEL_H


struct label_position
{
    struct label_position   *next;
    short                   position;
};
typedef struct label_position LabelPosition;

typedef enum 
{
    _UNDEFINED_LABEL_SIZE,
    _SIZE_1_75,
    _SIZE_2_37,
    _SIZE_3_00,
    _SIZE_3_50,
    _SIZE_4_00,
    _SIZE_4_50,
    _SIZE_5_00,
    _SIZE_5_50,
    _SIZE_6_00,
    _SIZE_6_50,
    _SIZE_7_00,
    _SIZE_7_50,
    _SIZE_8_00,
    _SIZE_8_50,
    _SIZE_9_00,
    _SIZE_9_50,
    _SIZE_10_00,
    _SIZE_CONTINUOUS    
}LabelSizeType;

typedef enum
{
    _UNKNOWN_STOCK,
    _HT_STOCK,           /* large gap between labels */
    _GT_STOCK            /* smaller gap between labels */
}StockType;

#endif