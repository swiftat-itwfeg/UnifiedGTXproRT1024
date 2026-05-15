#ifndef HOBART_PROTOCOL_H
#define HOBART_PROTOCOL_H

#define HOBART_PROTOCOL         3
typedef struct
{
    unsigned short       sourcePhysAddr;         /* unused */
    unsigned short       sourceLogicalAddr;
    unsigned short       DestPhysAddr;           /* unused */
    unsigned short       DestLogicalAddr;
    unsigned short       msgSize;        
}HBHEADER_t;

/* message defines */
#define MAX_PAYLOAD_SIZE        512     /* max size of bulk endpoint */
#define HBPAYLOAD_SIZE    ( MAX_PAYLOAD_SIZE - sizeof( HBHEADER_t ) )

typedef struct
{
    HBHEADER_t header;    
    unsigned char msgData[ HBPAYLOAD_SIZE ];
}HBMessage_t;


#endif