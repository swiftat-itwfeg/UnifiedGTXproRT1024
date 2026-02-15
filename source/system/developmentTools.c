#include "developmentSettings.h"
#include "fsl_debug_console.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <translator.h>


static unsigned short motorStallCounter = 0;
static unsigned short failToPeelCounter = 0;
static unsigned long totalLabelsPrinted = 0;
static unsigned short recoveredFromStall = 0;

/* These vars record which counters were incremented during the last label. */
static bool motorStallCounterIncrLastLabel = false;
static bool failToPeelCounterIncrLastLabel = false;
static bool recoveredFromStallCounterIncrLastLabel = false;


/******************************************************************************/
/*!   \fn clearPeelLogLastLabelBools(
        
      \brief  These vars record which counters were incremented during the
              last label. This function resets them (called at the start of 
              a new label)
        
      \author
        Tom Fink
*******************************************************************************/
void clearPeelLogLastLabelBools(void)
{
   motorStallCounterIncrLastLabel = false;
   failToPeelCounterIncrLastLabel = false;
   recoveredFromStallCounterIncrLastLabel = false;
}


/******************************************************************************/
/*!   \fn void makeAndSendPrinterPeelLogMsg(void)
        
      \brief  create a prPeelLog message and send it to the application for logging
        
      \author
        Tom Fink
*******************************************************************************/
void makeAndSendPrinterPeelLogMsg(void)
{
   PrPeelLog prPeelLog;

   prPeelLog.msgType = PR_PEEL_LOG;
   prPeelLog.numMtrStalls = motorStallCounter;
   prPeelLog.numRecoverFromMtrStall = recoveredFromStall;
   prPeelLog.numFailToPeel = failToPeelCounter;
   prPeelLog.totalLabelsPrinted = totalLabelsPrinted;
 
   sendPrPeelLog(&prPeelLog);
}

void printQueueInfo(unsigned short pQItems, unsigned short wQItems, unsigned short setQItems)
{ 
  static unsigned short counter = 0;
  static bool recursiveCall = false;
  counter++;
  
  if(!recursiveCall) {
  recursiveCall = true;
  PRINTF("msg in PQueue: %d, Msg in WQueue %d, msg in SetQueue: %d\r\n", pQItems, wQItems, setQItems); 
  recursiveCall = false;
  }  
}


/* Private Functions */
void printSelectedString(PrintMessageList stringIdentifier);

  /* The following assertion will fail if a service routine (ISR) for
             * an interrupt that has been assigned a priority above
             * configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
             * function.  ISR safe FreeRTOS API functions must *only* be called
             * from interrupts that have been assigned a priority at or below
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
             *
             * Numerically low interrupt priority numbers represent logically high
             * interrupt priorities, therefore the priority of the interrupt must
             * be set to a value equal to or numerically *higher* than
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
*/

//#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 2
#define PFISR_QSIZE 30

typedef struct {
    PrintMessageList items[PFISR_QSIZE];
    int front;
    int rear;
} pfISRQueueStuct;

pfISRQueueStuct pfISRQueue;

// Initialize the queue
void initQueue(void) {
    pfISRQueue.front = -1;
    pfISRQueue.rear = -1;
}

// Check if the queue is full
int isFull(void) {
    return pfISRQueue.rear == PFISR_QSIZE - 1;
}

// Check if the queue is empty
int isEmpty(void) {
    return pfISRQueue.front == -1;
}


// Add an element to the queue
void enqueue(PrintMessageList value) {
    if (isFull()) {
        return;
    }
    if (isEmpty()) {
        pfISRQueue.front = 0;
    }
    pfISRQueue.rear++;
    pfISRQueue.items[pfISRQueue.rear] = value;

}

// Remove an element from the queue
PrintMessageList dequeue(void) {
    if (isEmpty()) {
        return NO_PRINTF_MESSAGE;
    }
    PrintMessageList item = pfISRQueue.items[pfISRQueue.front];
    pfISRQueue.front++;
    if (pfISRQueue.front > pfISRQueue.rear) {
        pfISRQueue.front = pfISRQueue.rear = -1;
    }
    return item;
}

/*************************** PUBLIC FUNCTION *****************/
void queuePrintStringFromISR(PrintMessageList stringIdentifier)
{
   #ifdef PRINTFROMINTERRUPT
   enqueue(stringIdentifier);
   #endif   
}

void printFromInterrupt(void)
{
   #ifdef PRINTFROMINTERRUPT
  
   unsigned char i;
   for(i = 0; i<5;i++)  //Print five messages at a time. 
   {
      PrintMessageList msg2Print = dequeue();
      if (msg2Print != NO_PRINTF_MESSAGE) {
         printSelectedString(msg2Print);
      } 
      else
         i = 5; 
   }
   #endif
}

short copyOfAdjustDesiredTorqueDueToHighTorque;
short copyOfAdjustDesiredTorqueForAccurracy; 
unsigned short copyOfImmediateAdustStaticTUSpeedDueToHighTorque;
unsigned short copyOfLongTermAjustStaticTUSpeedDueToHighTorque;
void sendTorqeAdjustmentsToPrintf(short adjDesTorqDueToHighTorq, short adjDesTorqForAccur, unsigned short immAdjStatTUSpdDueToHiTorq, unsigned short LTAdjStatTUSpdDueToHiTorq)
{
   copyOfAdjustDesiredTorqueDueToHighTorque = adjDesTorqDueToHighTorq;
   copyOfAdjustDesiredTorqueForAccurracy = adjDesTorqForAccur;
   copyOfImmediateAdustStaticTUSpeedDueToHighTorque = immAdjStatTUSpdDueToHiTorq;
   copyOfLongTermAjustStaticTUSpeedDueToHighTorque = LTAdjStatTUSpdDueToHiTorq;
}



static unsigned short LLAverageTime = 0;
void sendLLAverageTimeToPrintf(unsigned short time)
{
   LLAverageTime = time;  
}

static tuControlMethodEnum previousControlState = INVALID;
static tuControlMethodEnum currentControlState = INVALID;
void sendControlStateToPrintf(unsigned short state)
{  
   currentControlState = (tuControlMethodEnum)state;  
   queueControlState();
}

void queueControlState(void)
{
   if(currentControlState != previousControlState) {
      previousControlState = currentControlState;
      if(currentControlState == HOLD_TIGHTEN_STEP_SPEED)
         enqueue(TU_CONTROL_HOLD_TIGHTEN_STEP_SPEED);
      else if(currentControlState == RAMP_TO_STATIC_STEP)
         enqueue(TU_CONTROL_RAMP_TO_STATIC_STEP);
      else if(currentControlState == STATIC_STEP)
         enqueue(TU_CONTROL_STATIC_STEP);
      else if(currentControlState == PID_CONTROL)
         enqueue(TU_CONTROL_PID_CONTROL);
      else if(currentControlState == NORMAL_TU_MOTOR_INTR_EXIT )
         enqueue(TU_CONTROL_NORMAL_TU_MOTOR_INTR_EXIT);
      else if(currentControlState == ABNORMAL_TU_MOTOR_INTR_EXIT )
         enqueue(TU_CONTROL_ABNORMAL_TU_MOTOR_INTR_EXIT);
      else
         enqueue(TU_CONTROL_UNKNOWN_STATE);        
   }
   
}

void printLLAverageTime(void)
{
   if(LLAverageTime != 0)
      PRINTF("LastLblAvgTime: %d, AdjDesTorqDue2HiTorq: %d, AdjDesTorq4Accur: %d, ImmAdjStaticTUSpdDue2HiTorq, %d, LTAdjStaticSpdDue2HighTorq %d\r\n\r\n", LLAverageTime, copyOfAdjustDesiredTorqueDueToHighTorque,
             copyOfAdjustDesiredTorqueForAccurracy, copyOfImmediateAdustStaticTUSpeedDueToHighTorque,copyOfLongTermAjustStaticTUSpeedDueToHighTorque);
   
   LLAverageTime = 0;
}

static unsigned short  copyOfStepsTakenTPHIntr = 0;
void sendStepsTakenTPHIntrToPrintf(unsigned short stepsTaken)
{
   copyOfStepsTakenTPHIntr = stepsTaken;
}

void printStepsTakenTPHIntr(void)
{
   if(copyOfStepsTakenTPHIntr != 0)
      PRINTF("StepsTaken: %d ,", copyOfStepsTakenTPHIntr);
   
   copyOfStepsTakenTPHIntr = 0;
}




void printSelectedString(PrintMessageList stringIdentifier)
{
   switch (stringIdentifier) {
    case    NO_PRINTF_MESSAGE:
      break;
    case    VSCOPE_INITIALIZED_MSG:
     // PRINTF("initializeVScope():INITIALIZED\r\n");
      break;
    case  VSCOPE_UNINITIALIZED_MSG:
     // PRINTF("batchPrintVScopeData():UNINITIALIZED\r\n");
      break;
    case VSCOPE_FULL_MSG:
      //PRINTF("vScopeRecordTakeUp():FULL\r\n");
      break;
    case VSCOPE_RECORDING_MSG:
     // PRINTF("enableVScope():RECORDING\r\n");
      break;
    case VSCOPE_STARTPRINTING_MSG:
     // PRINTF("startPrintVScope():START_PRINTING\r\n");
      break;
    case STEPTUMOTORINTR_TORQUE_OVERSHOOT:
      PRINTF("stepTUMtrIntr(): Early Torq Overshoot\r\n");
      break; 
    case STEPTUMOTORINTR_HIGHTORQ_SLOWSTEP:
      PRINTF("stepTUMtrIntr(): High Torq. Slow TU Step!\r\n");
      break;
    case STEPTUMOTORINTR_BAD_LAST_LABLEL_TIME:
      PRINTF("stepTUMtrIntr(): LastLabelTUStepTime Off\r\n");
      break; 
    case SWITCHED_TO_PID_DUE_TO_LOW_TORQUE:
      PRINTF("stepTUMtrIntr(): SWITCHED_TO_PID_DUE_TO_LOW_TORQUE\r\n");
      break; 
    case DRAMA_FREE_PRINT:
      /* successfully peeled the label */
      totalLabelsPrinted++;
      //makeAndSendPrinterPeelLogMsg(); //Athulya said we'll overwelm log if we send message every label
      PRINTF("\r\nMotorStall# %d, RecovFromStall %d, Fail2Peel# %d, TtlLables# %d\r\n\r\n", motorStallCounter, recoveredFromStall, failToPeelCounter, totalLabelsPrinted);
      break;
    case MOTOR_STALL:
      motorStallCounter++;
      motorStallCounterIncrLastLabel = true;
      totalLabelsPrinted++;
      makeAndSendPrinterPeelLogMsg();
      PRINTF("\r\nMotorStall# %d, RecovFromStall %d, Fail2Peel# %d, TtlLables# %d\r\n\r\n", motorStallCounter, recoveredFromStall, failToPeelCounter, totalLabelsPrinted);
      break;
    case FAIL_TO_PEEL:
      failToPeelCounter++;
      failToPeelCounterIncrLastLabel = true;
      totalLabelsPrinted++;
      makeAndSendPrinterPeelLogMsg();
      PRINTF("\r\nMotorStall# %d, RecovFromStall %d, Fail2Peel# %d, TtlLables# %d\r\n\r\n", motorStallCounter, recoveredFromStall, failToPeelCounter, totalLabelsPrinted);
      break;
    case RECOVERED_FROM_STALL:
      recoveredFromStall++;
      recoveredFromStallCounterIncrLastLabel = true;
      makeAndSendPrinterPeelLogMsg();
      PRINTF("\r\nMotorStall# %d, RecovFromStall %d, Fail2Peel# %d, TtlLables# %d\r\n\r\n", motorStallCounter, recoveredFromStall, failToPeelCounter, totalLabelsPrinted);
      break;
    case TU_CONTROL_HOLD_TIGHTEN_STEP_SPEED:
      PRINTF("HOLD_TIGHTEN_STEP_SPEED\r\n");
      break;
    case  TU_CONTROL_RAMP_TO_STATIC_STEP:
      PRINTF("RAMP_TO_STATIC_STEP\r\n");
      break;
    case TU_CONTROL_STATIC_STEP:
      PRINTF("STATIC_STEP\r\n");
      break;
    case TU_CONTROL_PID_CONTROL:
      PRINTF("PID_CONTROL\r\n");
      break;
    case TU_CONTROL_NORMAL_TU_MOTOR_INTR_EXIT:
      PRINTF("NORMAL_TU_MOTOR_INTR_EXIT\r\n");
      break;
    case TU_CONTROL_ABNORMAL_TU_MOTOR_INTR_EXIT:
      PRINTF("ABNORMAL_TU_MOTOR_INTR_EXIT\r\n");
      break;
    case TU_CONTROL_UNKNOWN_STATE :
      PRINTF("UNKNOWN_STATE\r\n");
      break;
    case USB_SEND_MSG_QUEUE_ALMOST_FULL:
       PRINTFThrottle(5,"USB Send Msg Queue almost full\r\n");
    case NON_GT_PRINTHEAD_DETECTED:
       PRINTFThrottle(500,"Non GT printhead detected\r\n");   
    break;
    default:
      PRINTF("printFromInterrupt(): unknown stringID = %d\r\n",stringIdentifier);
   }   
}

/******************************************************************************/
/*!   \fn void rollBackPeelLogCounters(void)
        
      \brief This function "rolls back" the peel log counters if they were 
             incremented on the last label. The purpose is to not count "fail
             to peel" or "motor stalls" that occurred because the last label
             stuck to the label roll core 
        
      \author
        Tom Fink
*******************************************************************************/
void rollBackPeelLogCounters(void)
{
   if(motorStallCounterIncrLastLabel == true) {
      if(motorStallCounter > 0)
         motorStallCounter--;
      
      PRINTF("\r\n\r\n**********************************\r\n");
      PRINTF("Rolled back motor stall counter due to End of Roll\r\n");
      PRINTF("\r\n**********************************\r\n\r\n");
   }
   
   if(recoveredFromStallCounterIncrLastLabel == true) {
      if(recoveredFromStall > 0)
        recoveredFromStall--;
      
      PRINTF("\r\n\r\n**********************************\r\n");
      PRINTF("Rolled back recovered from motor stall counter due to End of Roll\r\n");
      PRINTF("\r\n**********************************\r\n\r\n");
   }
   
   if(failToPeelCounterIncrLastLabel == true) {
      if(failToPeelCounter > 0)
        failToPeelCounter--;  
      
      PRINTF("\r\n\r\n**********************************\r\n");
      PRINTF("Rolled back Failed to Peel counter due to End of Roll\r\n");
      PRINTF("\r\n**********************************\r\n\r\n");
   }
}

void queueFullDebugMessage(UBaseType_t queueLength)
{ 
   PRINTF("\r\n\r\n**********\r\nprvNotifyQueueSetContainer() Queue Full! Length: %d\r\n\r\n**********\r\n",queueLength);
}

void printConfigAssertDebugMessage(void)
{  
  PRINTF("\r\n\r\n*********\r\n configAssert() called!\r\n\r\n**********\r\n");  
}


#if 0
// Define the queue handle
QueueHandle_t xQueue;


void createPrintFromInterruptQueue(void)
{
  #ifdef PRINTFROMINTERRUPT
  xQueue = xQueueCreate(20, sizeof(PrintMessageList));
  PRINTF("Print from Interrupt Queue created\r\n");
  #endif 
}


void queuePrintStringFromISR(PrintMessageList stringIdentifier)
{
   #ifdef PRINTFROMINTERRUPT
   BaseType_t xHigherPriorityTaskWasWoken;
   xQueueSendFromISR(xQueue, &stringIdentifier, &xHigherPriorityTaskWasWoken);
   #endif   
}

void printFromInterrupt(void)
{
   #ifdef PRINTFROMINTERRUPT
   PrintMessageList receivedStringID = NO_PRINTF_MESSAGE;
   
   if(xQueue == NULL)
      return;
   
   unsigned char i;
   for(i = 0; i<5;i++)  //Print five messages at a time. 
   {
      if (xQueueReceive(xQueue, &receivedStringID, 0) == pdPASS) {
         printSelectedString(receivedStringID);
      } 
      else
         i = 5; 
   }
   #endif
}

#endif