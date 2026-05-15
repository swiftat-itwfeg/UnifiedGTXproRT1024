#ifndef DRV8818_H
#define DRV8818_H
#include "deviceProperties.h"
#include "fsl_common.h"
#include "MIMXRT1024.h"
#include <stdint.h>
/****************************** common defines ********************************/
/******************************************************************************/
typedef enum
{
   _MAIN_STEPPER,
   _TAKEUP_STEPPER    
}DVR8818Type;

typedef enum
{
    FORWARDM_,
    BACKWARDM_
}StepDirM;
/******************************************************************************/
/******************************************************************************/


/****************************** avery defines *********************************/
/******************************************************************************/
#define BRAKING_DISTANCE			10	
#define ACCELERATION_CONSTANT		        30      	/* bigger numbers means slower acceleration*/
#define MIN_SPEED				20		/* steps/sec */
#define DEFAULT_SPEED_MM			150		/* mm/sec */
#define DEFAULT_SPEED_STEPS 			( 8 * 150 )     /* steps/sec */
#define NOMINAL_MVOLTS				24000
/******************************************************************************/
/******************************************************************************/


/************************** common public functions ***************************/
/******************************************************************************/
void initializeMotors( DEVICESUBCLASS_t sClass, StepDirM direction );
static void initIODRV8818( DVR8818Type type );
AT_QUICKACCESS_SECTION_CODE(static void setStepDirection( DVR8818Type type, StepDirM dir )); 
AT_QUICKACCESS_SECTION_CODE(void setMainMotorDirection( StepDirM direction ));
AT_QUICKACCESS_SECTION_CODE(void setTakeUpMotorDirection( StepDirM direction ));
AT_QUICKACCESS_SECTION_CODE(void powerOnMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void powerOnMotorsDuringCal( void ));
AT_QUICKACCESS_SECTION_CODE(void powerOffMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void stepMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void rampStepMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void stepMainMotor( void ));
AT_QUICKACCESS_SECTION_CODE(void stepTakeUpMotor( void ));
AT_QUICKACCESS_SECTION_CODE(void setFullStepMode( DVR8818Type type ));
AT_QUICKACCESS_SECTION_CODE(void setHalfStepMode( DVR8818Type type ));
AT_QUICKACCESS_SECTION_CODE(void setQuarterStepMode( DVR8818Type type ));
AT_QUICKACCESS_SECTION_CODE(void setEighthStepMode( DVR8818Type type ));
void sleepMotor( DVR8818Type type );
void wakeupMotor( DVR8818Type type );
void releaseFromReset( DVR8818Type type );
void resetMotor(  DVR8818Type type );
bool getMotorPowerStatus( void );
/******************************************************************************/
/******************************************************************************/


/*************************** avery's public functions *************************/
/******************************************************************************/
void stepMainMotorAvery( bool toggle, bool dirn );
void stepTakeUpMotorAvery( bool toggle );
uint32_t getNextSpeed( uint32_t targetSpeed, uint32_t currentSpeed, bool dirn, uint32_t stepsToGo ); 
uint32_t getStepTime( uint32_t speed ); 
/******************************************************************************/
/******************************************************************************/
#endif