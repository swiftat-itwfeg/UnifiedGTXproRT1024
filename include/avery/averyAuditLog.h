#ifndef AVERYAUDITLOG_H
#define AVERYAUDITLOG_H
#include "stdint.h"
#include <stdbool.h>
#include "averyProtocol.h"
typedef struct
{
    bool           initialized;          /* flash initialized and ready for use */
    uint32_t       flashStartAddr;      /* start of the audit log */
    uint32_t       flashEndAddr;        /* ending address of audit log */
    uint32_t       flashPageSize;       /* flash page size */
    uint32_t       flashSectorSize;     /* flash sector size */    
}FlashAudit_t;


uint32_t getAuditLogStart( void );
int16_t getAuditLog( uint16_t entry, BinAudit_t *pLog );
int16_t addAuditEntry( BinAudit_t *pEntry );
int16_t resetAuditLog( void );
uint16_t getAuditLogEntryCount( void );

#endif