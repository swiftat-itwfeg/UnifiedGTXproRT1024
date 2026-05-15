#ifndef AVERYPRINTER_H
#define AVERYPRINTER_H
#include "averyPrinterCfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include <stdbool.h>

#define printer_task_PRIORITY ( configMAX_PRIORITIES - 1 )

#define DEFAULT_CUTTER_SPEED			9
#define MIN_CUTTER_SPEED			0
#define MAX_CUTTER_SPEED			9
#define DEFAULT_CUT_DISTANCE			82
#define MIN_CUT_DISTANCE			30
#define MAX_CUT_DISTANCE			255
#define CUTTER_LOCKOUT_TIMEOUT			5000

enum PRINTER_PAPER_MODE
{
	PPM_SEPARATE,
	PPM_CONTINUOUS_BACKWIND,
	PPM_CONTINUOUS_NO_BACKWIND,
};


/* prototypes */
BaseType_t createAveryPrinterTask( QueueHandle_t msgQueue );
PRCfg_t *getAveryPrinterCfg( void );
bool isClamShell( void );
bool isContinuousPrint( void );

/* private functions */
static void averyPrinterTask( void *pvParameters );

#endif