#ifndef UNIFIED_PROTOCOL_H
#define UNIFIED_PROTOCOL_H

#define UNIFIED_PROIOCOL        4
typedef struct
{
    unsigned char  protocol;            /* Avery, Hobart, Unified ( Global ) */
    unsigned char  msgType;             /* ex: ReqStatus, Print, ect. */
    unsigned short msgSize;             /* message body size */
    unsigned short frameNumber;         /* current frame number */
    unsigned short NumOfframes;         /* total frames needed to complete message */
    unsigned short lastFrameSize;	/* total bytes in the last frame */
}UNMSGHEADER_t;

/* message defines */
#define MAX_UNPAYLOAD_SIZE        512     /* max size of bulk endpoint */
#define UNPAYLOAD_SIZE    ( MAX_UNPAYLOAD_SIZE - sizeof( UNMSGHEADER_t ) )

typedef struct
{
    UNMSGHEADER_t  header;
    unsigned char  msgData[UNPAYLOAD_SIZE];  
}UNMSGPAYLOAD_t;


#endif