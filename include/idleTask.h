#ifndef IDLETASK_H
#define IDLETASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//#define idle_task_PRIORITY ( configMAX_PRIORITIES -3 )
#define idle_task_PRIORITY ( configMAX_PRIORITIES - 1 )

/* prototypes */
BaseType_t createIdleTask( void );
#endif