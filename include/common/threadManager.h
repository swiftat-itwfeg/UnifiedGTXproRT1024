#ifndef THREADMANAGER_H
#define THREADMANAGER_H
#include "deviceProperties.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include <stdbool.h>

#define TASK_COUNT 7

#define manager_task_PRIORITY ( configMAX_PRIORITIES -1 )

typedef enum {
    F_WEIGHERTHREAD,
    F_PRINTERTHREAD,
    F_TRANSLATORTHREAD,
    F_FLEXCANTHREAD,
    F_BOARDTESTTHREAD
}F_Threads;  

typedef enum{
  SUSPEND,
  RESUME,
  TERMINATE,
  LIST,
  CREATE
}ManagerCommand;

typedef enum{
  T_WEIGHER,
  T_PRINTER,
  T_USB,
  T_TRANSLATOR,
  T_SENSORS,
  T_CUTTER,
  T_VALUEMAX,
  T_DOT_WEAR,
  T_IDLE,
  T_UNIT_TEST,
  LAST_TASK 
}ManagedThreads;

/* These define which bit correlates to which permission */
#define SUSPEND_RESUMEMASK      (0x01 << 0)
#define TEMINATEMASK            (0x01 << 1)
#define CREATEMASK              (0x01 << 2)

/*Permission fields to allow the thread manager to manipulate tasks*/
#define WEIGHERPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK)
#define PRINTERPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK)
#define USBPERMISSIONS                  (SUSPEND_RESUMEMASK)
#define TRANSLATORPERMISSIONS           (SUSPEND_RESUMEMASK)
#define SENSORSPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK | CREATEMASK)
#define UNITTESTPERMISSIONS             (TEMINATEMASK | SUSPEND_RESUMEMASK)
#define DOTWEARPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK | CREATEMASK)

struct manager_msg
{
    ManagerCommand cmd;
    ManagedThreads task; 
};
typedef struct manager_msg ManagerMsg;

/* public functions */
void systemStartup( void );
void startUnitTest( void );
void terminateThread( F_Threads thread_ );
void initializeThreadStatistics( void );
DEVICECLASS_t getDeviceClass( void );
DEVICESUBCLASS_t getDeviceSubClass( void );
PeripheralModel_t getMyModel( void );
BaseType_t createThreadManagerTask( void );
void assignThreadManagerMsgQueue( QueueHandle_t pQHandle );
void updateTaskHandle(ManagedThreads thread);
void delay_uS ( unsigned int time );

/* private functions */
static void startSystemThreads( void );
static void initializeModelIO( void );
static PeripheralModel_t getPeripheralModel( DEVICE_PROPERTIES_t *pDProperties );

static void threadManager( void *pvParameters );
static void handleManagerMsg(ManagerMsg * msg);
static void tm_createSensorTask();
static bool tm_createDotWearTask();

static void showSystemBanner( void );
#ifdef PRINT_STATS
void statsTimerCallback( TimerHandle_t xTimer );
#endif
#endif