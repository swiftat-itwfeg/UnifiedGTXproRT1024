#include "dvr8818.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "fsl_debug_console.h"
#include "sensors.h"
#include "averyUtils.h"

/******************************* common variables *****************************/
/******************************************************************************/
extern void delay_uS( unsigned int time );
static bool powerStatus = false;
static bool motorTimer_ = false;
static DEVICESUBCLASS_t sClass_ = UNKNOWN_SUBCLASS;
/******************************************************************************/
/******************************************************************************/

/******************************* hobart variables *****************************/
/******************************************************************************/
extern PrStatusInfo currentStatus;
extern bool printing_;
/******************************************************************************/
/******************************************************************************/


/******************************* avery variables ******************************/
/******************************************************************************/

extern unsigned long gMVolts24;
extern unsigned long gSetSpeed;
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*!   \fn void initializeMotors( StepDirM direction )

      \brief
        This function initializes main / takeup stepper motor dirver ic and
        sets up timer to pulse motor enable signal when motors are on.
   
      \author
          Aaron Swift
*******************************************************************************/
void initializeMotors( DEVICESUBCLASS_t sClass, StepDirM direction )
{
    sClass_ = sClass;
    if( ( sClass_ == AV_XPRO ) || ( sClass_ == HB_GT) ) { 
        /* keep both motors in reset */
        initIODRV8818( _MAIN_STEPPER );
        initIODRV8818( _TAKEUP_STEPPER );

        /* wakeup both drivers but keep output disabled */
        wakeupMotor( _MAIN_STEPPER );
        wakeupMotor( _TAKEUP_STEPPER );
        
        /* set both motor directions */
        setStepDirection( _MAIN_STEPPER, direction );
        
        /* always drive the takeup opposite direction of the main motor */
        if( direction == FORWARDM_ )
            setStepDirection( _TAKEUP_STEPPER, BACKWARDM_ );
        else
            setStepDirection( _TAKEUP_STEPPER, FORWARDM_ );
        
        /* set both motors step mode to quater step */
        setQuarterStepMode( _MAIN_STEPPER ); 
        setQuarterStepMode( _TAKEUP_STEPPER ); 
        
        /* sets up an overall period of 80mS for motor enable signal to keep alive circuit.*/ 
        PIT_SetTimerPeriod( PIT, kPIT_Chnl_2, USEC_TO_COUNT( 100000U, CLOCK_GetFreq( kCLOCK_OscClk ) ) );        

        PIT_EnableInterrupts( PIT, kPIT_Chnl_2, kPIT_TimerInterruptEnable );   

        /* keep power off but enable h-bridge output to motors */
        powerOffMotors();    
        
        releaseFromReset( _MAIN_STEPPER );
        releaseFromReset( _TAKEUP_STEPPER );
    } else if( ( sClass_ == AV_XONE ) || ( sClass_ == HB_DT ) ) {
        initIODRV8818( _MAIN_STEPPER );
        /* wakeup driver but keep output disabled */
        wakeupMotor( _MAIN_STEPPER );
        /* set motor directions */
        setStepDirection( _MAIN_STEPPER, direction );
        /* set motor step mode to quater step */
        setQuarterStepMode( _MAIN_STEPPER ); 
        /* sets up an overall period of 80mS for motor enable signal to keep alive circuit.*/ 
        PIT_SetTimerPeriod( PIT, kPIT_Chnl_2, USEC_TO_COUNT( 100000U, CLOCK_GetFreq( kCLOCK_OscClk ) ) );        

        PIT_EnableInterrupts( PIT, kPIT_Chnl_2, kPIT_TimerInterruptEnable );   

        /* keep power off but enable h-bridge output to motor */
        powerOffMotors();    
        releaseFromReset( _MAIN_STEPPER );
    } 
}

/******************************************************************************/
/*!   \fn void setMainMotorDirection( StepDirM direction )

      \brief
        This function sets the direction pin on the main stepper motor dirver ic.    
   
      \author
          Aaron Swift
*******************************************************************************/
void setMainMotorDirection( StepDirM direction )
{
    setStepDirection( _MAIN_STEPPER, direction );    
}

/******************************************************************************/
/*!   \fn void setTakeUpMotorDirection( StepDirM direction )

      \brief
        This function sets the direction pin on the takeup stepper motor dirver ic.        
   
      \author
          Aaron Swift
*******************************************************************************/
void setTakeUpMotorDirection( StepDirM direction )
{
    setStepDirection( _TAKEUP_STEPPER, direction );    
}
   
/******************************************************************************/
/*!   \fn void powerOnMotors( void )

      \brief
        This function enables output of both main / takeup stepper motor dirver 
        ics.    
   
      \author
          Aaron Swift
*******************************************************************************/
void powerOnMotors( void )
{    
    if( ( sClass_ == AV_XPRO ) || ( sClass_ == HB_GT) ) {   
        if( ( ( ( currentStatus.error & HEAD_UP ) != HEAD_UP ) && 
            ( ( currentStatus.sensor & OUT_OF_MEDIA ) != OUT_OF_MEDIA ) && 
              ( ( currentStatus.sensor2 & JAMMED_LABEL ) != JAMMED_LABEL ) ) /*|| 
                 getGapCalStatus() == true */ ) {
            powerStatus = true;

            GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, false );
            delay_uS(1);
            GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true );
            
            /* if we have not started the motor enable timer then do so. */
            if( !motorTimer_ ) {
                motorTimer_ = true;
                PIT_StartTimer( PIT, kPIT_Chnl_2 );          
            }
        } else {          
            powerOffMotors();
        }  
    } else if( ( sClass_ == AV_XONE ) || ( sClass_ == HB_DT) ) {
        /* set power status flag */
        powerStatus = true;
      
        GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, false );
        delay_uS(1);
        GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true );
        
        /* if we have not started the motor enable timer then do so. */
        if( !motorTimer_ ) {
            motorTimer_ = true;
            PIT_StartTimer( PIT, kPIT_Chnl_2 );          
        }    
    }
}
/******************************************************************************/
/*!   \fn void powerOffMotors( void )

      \brief
        This function disables output of both main / takeup stepper motor dirver 
        ics.    
   
      \author
          Aaron Swift
*******************************************************************************/
void powerOffMotors( void )
{
    powerStatus = false;
    /* if we have started the motor enable timer then stop it. */   
    if( motorTimer_ ) {
        PIT_StopTimer( PIT, kPIT_Chnl_2 );
        motorTimer_ = false;
    }
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true );
    GPIO_WritePinOutput( TAKEUP_MOTOR_EN_GPIO, TAKEUP_MOTOR_EN_PIN, true );
}

/******************************************************************************/
/*!   \fn static void initIODRV8818( DVR8818Type type )

      \brief
        This function initializes the gpio associated with the type argument.            
        Motor drivers will be disabled after this function call.
      \author
          Aaron Swift
*******************************************************************************/
static void initIODRV8818( DVR8818Type type )
{
    gpio_pin_config_t dirConfig  = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    gpio_pin_config_t stepConfig  = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode };
    gpio_pin_config_t enableConfig = { kGPIO_DigitalOutput, 1, kGPIO_NoIntmode };
    gpio_pin_config_t resetConfig = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode  };    
    gpio_pin_config_t sleepConfig = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode  }; 
    gpio_pin_config_t msConfig = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode}; 
      
    
    if( type == _MAIN_STEPPER ) {
        /* intialize the gpio to the main motor driver ic */
        GPIO_PinInit( MAIN_MOTOR_DIR_GPIO, MAIN_MOTOR_DIR_PIN, &dirConfig );
        GPIO_PinInit( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, &stepConfig );

        GPIO_PinInit( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, &resetConfig );
        GPIO_PinInit( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, &sleepConfig );    
        GPIO_PinInit( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, &msConfig );    
        GPIO_PinInit( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, &msConfig );      
    } else {
        /* intialize the gpio to the takeup motor driver ic */
        GPIO_PinInit( TAKEUP_MOTOR_DIR_GPIO, TAKEUP_MOTOR_DIR_PIN, &dirConfig );
        GPIO_PinInit( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, &stepConfig );
        GPIO_PinInit( TAKEUP_MOTOR_EN_GPIO, TAKEUP_MOTOR_EN_PIN, &enableConfig );
        GPIO_PinInit( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, &resetConfig );
        GPIO_PinInit( TAKEUP_MOTOR_SLEEP_GPIO, TAKEUP_MOTOR_SLEEP_PIN, &sleepConfig ); 
        GPIO_PinInit( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, &msConfig ); 
        GPIO_PinInit( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, &msConfig ); 
        
        /*Should always be true for production boards. False is for Avery model */
        /* Set current limit - true = high, false = low */
        GPIO_WritePinOutput( TAKEUP_MOTOR_HIGH_CUR_LIMIT_GPIO, TAKEUP_MOTOR_HIGH_CUR_LIMIT_PIN, true );
    }    
}

/******************************************************************************/
/*!   \fn void stepMotors( void )

      \brief
        This function steps both the main and takeup motors.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepMotors( void )
{
    powerOnMotors();
  
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );

    delay_uS(5); 
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
}

/******************************************************************************/
/*!   \fn void stepMainMotor( void )

      \brief
        This function steps both the main and takeup motors. Used in rampMotors().
        
      \author
          Chris King
*******************************************************************************/
void stepMainMotor( void )
{    
    powerOnMotors();
    
    HeadType_t head = getPrintHeadType();
    
    if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {
        GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
        
        delay_uS(5);
        
        GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false ); 
    } else {
        GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );

        // step pin minimum high/low time is 1uS 
        delay_uS(5); 
            
        GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false ); 
    }
}

/******************************************************************************/
/*!   \fn void stepMotors( void )

      \brief
        This function steps only the takeup motor.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepTakeUpMotor( void )
{
    powerOnMotors();
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
    
    delay_uS(5);
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false ); 
}

/******************************************************************************/
/*!   \fn static void setStepDirection( DVR8818Type type, StepDirM dir )

      \brief
        This function sets the step direction of the main and take up motors.
        
      \author
          Aaron Swift
*******************************************************************************/
static void setStepDirection( DVR8818Type type, StepDirM dir )
{
    
    HeadType_t head = getPrintHeadType();
  
    if( head == KYOCERA753_OHM || head == KYOCERA800_OHM || head == KYOCERA849_OHM ) {   
        dir = 1;
    }
  
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_DIR_GPIO, MAIN_MOTOR_DIR_PIN, dir );      
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_DIR_GPIO, TAKEUP_MOTOR_DIR_PIN, dir );      
    }   
}

/******************************************************************************/
/*!   \fn static void setFullStepMode( DVR8818Type type )

      \brief
        This function sets the step mode of the motor to full.
        
      \author
          Aaron Swift
*******************************************************************************/
void setFullStepMode( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, false );  
        GPIO_WritePinOutput( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, false );  
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, false );  
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, false );          
    }    
}

/******************************************************************************/
/*!   \fn static void setHalfStepMode( DVR8818Type type )

      \brief
        This function sets the step mode of the motor to half.
        
      \author
          Aaron Swift
*******************************************************************************/
void setHalfStepMode( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, true );  
        GPIO_WritePinOutput( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, false );  
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, true );  
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, false );          
    }    
}

/******************************************************************************/
/*!   \fn static void setQuarterStepMode( DVR8818Type type )

      \brief
        This function sets the step mode of the motor to quarter.
        
      \author
          Aaron Swift
*******************************************************************************/
void setQuarterStepMode( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, false );  
        GPIO_WritePinOutput( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, true );  
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, false );  
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, true );          
    }         
}

/******************************************************************************/
/*!   \fn static void setEightStepMode( DVR8818Type type )

      \brief
        This function sets the step mode of the motor to eigth.
        
      \author
          Aaron Swift
*******************************************************************************/
void setEighthStepMode( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, true );  
        GPIO_WritePinOutput( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, true );  
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, true );  
        GPIO_WritePinOutput( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, true );          
    }        
}

/******************************************************************************/
/*!   \fn static void setEightStepMode( DVR8818Type type )

      \brief
        This function sets the motor into sleep mode.
        
      \author
          Aaron Swift
*******************************************************************************/
void sleepMotor( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, false );      
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_SLEEP_GPIO, TAKEUP_MOTOR_SLEEP_PIN, false );      
    }    
}

/******************************************************************************/
/*!   \fn static void setEightStepMode( DVR8818Type type )

      \brief
        This function wakes the motor from sleep mode.
        
      \author
          Aaron Swift
*******************************************************************************/
void wakeupMotor( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, true );      
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_SLEEP_GPIO, TAKEUP_MOTOR_SLEEP_PIN, true );      
    }
}

/******************************************************************************/
/*!   \fn void releaseFromReset( DVR8818Type type )

      \brief
        This function releases the motor from reset.
        
      \author
          Aaron Swift
*******************************************************************************/
void releaseFromReset( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, true );      
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, true );      
    }    
}

#pragma diag_suppress=Pe177
/******************************************************************************/
/*!   \fn resetMotor( DVR8818Type type )

      \brief
        This function resets the motor.
        
      \author
          Aaron Swift
*******************************************************************************/
void resetMotor( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
         GPIO_WritePinOutput( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, false );           
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, false );        
    }    
}
#pragma diag_default=Pe177

/******************************************************************************/
/*!   \fn void unitTestMotorsIO( void )

      \brief
        This function toggles each gpio pin to allow validation with scope.
        
      \author
          Aaron Swift
*******************************************************************************/
void unitTestMotorsIO( void )
{
    /* reset gpio */
    GPIO_WritePinOutput( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, true );     
    GPIO_WritePinOutput( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, false );     

    GPIO_WritePinOutput( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, true );        
    GPIO_WritePinOutput( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, false );        
    /* sleep gpio */
    GPIO_WritePinOutput( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, true );      
    GPIO_WritePinOutput( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, false );      

    GPIO_WritePinOutput( TAKEUP_MOTOR_SLEEP_GPIO, TAKEUP_MOTOR_SLEEP_PIN, true );      
    GPIO_WritePinOutput( TAKEUP_MOTOR_SLEEP_GPIO, TAKEUP_MOTOR_SLEEP_PIN, false );      
    /* ms0 gpio */
    GPIO_WritePinOutput( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, true );  
    GPIO_WritePinOutput( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, false );  

    GPIO_WritePinOutput( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, true );  
    GPIO_WritePinOutput( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, false );  
    /* ms1 gpio */    
    GPIO_WritePinOutput( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, true );  
    GPIO_WritePinOutput( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, false );  

    GPIO_WritePinOutput( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, true );          
    GPIO_WritePinOutput( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, false );          
    /* direction gpio */
    GPIO_WritePinOutput( MAIN_MOTOR_DIR_GPIO, MAIN_MOTOR_DIR_PIN, FORWARDM_ );      
    GPIO_WritePinOutput( MAIN_MOTOR_DIR_GPIO, MAIN_MOTOR_DIR_PIN, BACKWARDM_ );      

    GPIO_WritePinOutput( TAKEUP_MOTOR_DIR_GPIO, TAKEUP_MOTOR_DIR_PIN, FORWARDM_ );      
    GPIO_WritePinOutput( TAKEUP_MOTOR_DIR_GPIO, TAKEUP_MOTOR_DIR_PIN, BACKWARDM_ );      
    
    /* enable gpio */
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true ); 
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, false ); 
    
    
    /* low to high transation causes step */
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
    
    /* step pin minimum high/low time is 1uS */ 
    delay_uS(6); 
   
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );  
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );        
}

bool getMotorPowerStatus( void )
{
    return powerStatus;
}

/******************************************************************************/
/*!   \fn uint32_t getNextSpeed( uint32_t targetSpeed, 
                                      uint32_t currentSpeed, 
                                      bool dirn, uint32_t stepsToGo )

      \brief
        This function computes the next speed dependent on various factors 

      \param
        unsigned long targetSpeed - the nominal speed 
        unsigned long currentSpeed - speed at previous step
        unsigned long stepsToGo - how far before we may have to stop

      \author
          Nick Barnes
*******************************************************************************/
uint32_t getNextSpeed( uint32_t targetSpeed, 
                            uint32_t currentSpeed, 
                            bool dirn, uint32_t stepsToGo )
{
	unsigned long speed;
	long previousMV24 = gMVolts24;

	//if we need to stop
	if(stepsToGo < BRAKING_DISTANCE)
	{
		//put on the brakes.  Choose
		//a speed proportional to how many steps are left
		speed = targetSpeed * stepsToGo / BRAKING_DISTANCE;

		//just to ensure that if we hadn't attained target speed, we don't 
		//abruptly accelerate, when our intention is to slow down
		if(speed > currentSpeed)
		{
			speed = currentSpeed;
		}
	}
	else
	{
		long dif;
		long acceleration;

		#define MIN_VOLTS 5000
		#define SLIGHT_MVOLT_DROP 2000
		#define MAX_MVOLT_DROP 6000
		long mV24 = gMVolts24 = convertTo24vMVolts( readPrinterAdc( ADC_24V ) );
		long myTargetSpeed = targetSpeed;
		long boost = 4 - (4 * currentSpeed) /
		//              --------------------
		                (gSetSpeed + 1);	//Acceleration can be boosted at lower speeds
		long accelerationConstant = ACCELERATION_CONSTANT / boost;

		if(mV24 > MIN_VOLTS)
		{
			long mV24ExcessDrop = NOMINAL_MVOLTS - mV24 - SLIGHT_MVOLT_DROP;
			long mV24Change = mV24 - previousMV24;

			//if volts drops too much
			if(mV24ExcessDrop > 0)
			{
				//Reduce speed to allow power supply to recover
				myTargetSpeed = myTargetSpeed * (MAX_MVOLT_DROP - mV24ExcessDrop)/
				//               ------------------------------------------------
				                                 MAX_MVOLT_DROP;
	
				//2nd order term, to stabilise the control system
				myTargetSpeed = myTargetSpeed * (MAX_MVOLT_DROP + 2 * mV24Change)/
				//               ------------------------------------------------
				                                 MAX_MVOLT_DROP;
			}						  
		}

		dif = myTargetSpeed - currentSpeed;

		//if going backwards...
		if( dirn == BACKWARDM_ )
			//...accelerate more gently
			accelerationConstant *= 1;
			
		//need to accelerate toward target speed.  The ideal acceleration 
		//profile gives high acceleration at low speeds and more gentle
		//acceleration as the target speed is neared.  NB this will always
		//give value > 0 unless targetSpeed == currentSpeed
		acceleration = (ABS(dif) + accelerationConstant)/
		//             ----------------------------------
		                   (accelerationConstant + 1);

		//if desired speed is less than current...
		if(dif < 0)
		{
			//change acceleration to deceleration - and double it, because
			//it's easier to slow down that to speed up
			acceleration = -acceleration * 2;
		}

		speed = currentSpeed + acceleration;
	}

	if(speed < MIN_SPEED || speed > gSetSpeed)
	{
		speed = MIN_SPEED; //safety
	}

	return speed;
}


/****************************************************************************/
/*                                                                          */
/* Function:                                                                */
/*                                                                          */
/* Description:                                                             */
/*         convert from speed (in steps per second) to step time (in uS)    */
/*                                                                          */
/* Author: Nick Barnes                                       31/10/2008     */
/* Task:                                                                    */
/*                                                                          */
/* Inputs:                                                                  */
/*         unsigned long speed - speed (steps/s)                            */
/*                                                                          */
/* Outputs:                                                                 */
/*         unsigned long steps time (uS)                                    */
/*                                                                          */
/****************************************************************************/
/******************************************************************************/
/*!   \fn uint32_t getStepTime( uint32_t speed )

      \brief
        This function convert from speed (in steps per second) to step time (in uS) 

      \param
        uint32_t speed - speed (steps/s) 

      \return uint32_t steps time (uS) 
      
      \author
          Nick Barnes
*******************************************************************************/
uint32_t getStepTime( uint32_t speed )
{
    return 1000000/speed;       /* uSec */
}

