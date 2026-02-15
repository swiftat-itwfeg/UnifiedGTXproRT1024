#include "dvr8818.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "sensors.h"
#include "printEngine.h"
#include "lp5521.h"
#include "virtualScope.h"
#include "fsl_pit.h"
#include "takeupMotor.h"
//TFinkToDo! #include "threadManager.h"

extern void delay_uS( unsigned int time );
extern PrStatusInfo currentStatus;
static bool powerStatus = false;
/******************************************************************************/
/*!   \fn void initializeMotors( StepDirM direction )

      \brief
        This function initializes main / takeup stepper motor dirver ic.    
   
      \author
          Aaron Swift
*******************************************************************************/
void initializeMotors( StepDirM direction )
{
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
    
    /* set both motors step mode to half step */
    //setHalfStepMode( _MAIN_STEPPER );
    //setHalfStepMode( _TAKEUP_STEPPER ); 
    
      
    setQuarterStepMode( _MAIN_STEPPER ); 
    setQuarterStepMode( _TAKEUP_STEPPER ); 
    
    /* keep power off but enable h-bridge output to motors */
    powerOffMotors();    
    
    releaseFromReset( _MAIN_STEPPER );
    releaseFromReset( _TAKEUP_STEPPER );
}

void setMainMotorDirection( StepDirM direction )
{
    setStepDirection( _MAIN_STEPPER, direction );    
}

void setTakeUpMotorDirection( StepDirM direction )
{
    setStepDirection( _TAKEUP_STEPPER, direction );    
}

/******************************************************************************/
/*!   \fn void initializeMainMotor( void )

      \brief
        This function enables output of both main / takeup stepper motor dirver 
        ics.    
   
      \author
          Aaron Swift
*******************************************************************************/
void powerOnMotorsDuringCal( void )
{
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, false );
    delay_uS(1);
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true );
}

void powerOnMotors( void )
{    
    if((((currentStatus.error & HEAD_UP ) != HEAD_UP ) && ((currentStatus.sensor & OUT_OF_MEDIA) != OUT_OF_MEDIA) && ((currentStatus.sensor2 & JAMMED_LABEL) != JAMMED_LABEL)) || (getGapCalStatus() == true || getTUCalStatus() == true)) 
    {
        powerStatus = true;
      
        GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, false );
        delay_uS(1);
        GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true );
        
        //PRINTF("!\r\n");
        
        vScopeRecordMtrEnblTime();
    } 
    else 
    {   
        //PRINTF("PRINTER FEED ERROR\r\n");
        
        if(((currentStatus.error & HEAD_UP ) == HEAD_UP ))
        {
            //PRINTF("HEAD UP\r\n");
        }
        
        if(((currentStatus.sensor & OUT_OF_MEDIA) == OUT_OF_MEDIA))
        {
            //PRINTF("OUT OF MEDIA\r\n");
        }
        
        if(((currentStatus.sensor2 & JAMMED_LABEL) == JAMMED_LABEL))
        {
            //PRINTF("JAMMED LABEL\r\n");
        }
    
        stopTakeupIntr();
        stopTPHIntr();
        //setSizingState(0);
        clearSizingVariables();
        
        powerOffMotors();
    }
    
    /*
    if(((currentStatus.error & HEAD_UP ) == HEAD_UP ))
    {
        PRINTF("HEAD UP\r\n");
    }
    
    if(((currentStatus.sensor & OUT_OF_MEDIA) == OUT_OF_MEDIA))
    {
        PRINTF("OUT OF MEDIA\r\n");
    }
    
    if(((currentStatus.sensor2 & JAMMED_LABEL) == JAMMED_LABEL))
    {
        PRINTF("JAMMED LABEL\r\n");
    }
    */
    //PRINTF("!\r\n");
    /*
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, false );
    GPIO_WritePinOutput( TAKEUP_MOTOR_EN_GPIO, TAKEUP_MOTOR_EN_PIN, false );
    delay_uS(1);
    GPIO_WritePinOutput( MOTOR_EN_GPIO, MOTOR_EN_PIN, true );
    GPIO_WritePinOutput( TAKEUP_MOTOR_EN_GPIO, TAKEUP_MOTOR_EN_PIN, true );
    */
}
/******************************************************************************/
/*!   \fn void initializeMainMotor( void )

      \brief
        This function disables output of both main / takeup stepper motor dirver 
        ics.    
   
      \author
          Aaron Swift
*******************************************************************************/
void powerOffMotors( void )
{
    powerStatus = false;
  
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
        //GPIO_PinInit( MOTOR_EN_GPIO, MOTOR_EN_PIN, &enableConfig );
        GPIO_PinInit( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, &resetConfig );
        GPIO_PinInit( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, &sleepConfig );    
        GPIO_PinInit( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, &msConfig );    
        GPIO_PinInit( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, &msConfig );
        
        #if 0 //TFinkToDo!
        /* Default to high current for for rev 0 boards, low for rev 1 boards - because of 
           resistor changes, both will be ~800mA */
        if(getpcbaRevision() == 0)
           GPIO_WritePinOutput( MAIN_MOTOR_HIGH_CUR_LIMIT_GPIO, MAIN_MOTOR_HIGH_CUR_LIMIT_PIN, true);
        else if(getpcbaRevision() == 1)
           GPIO_WritePinOutput( MAIN_MOTOR_HIGH_CUR_LIMIT_GPIO, MAIN_MOTOR_HIGH_CUR_LIMIT_PIN, false);
       #endif
        
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
    /*
    powerOnMotors();
  
    if( getTakeUpTorque() < 2400 ) {
        GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
        GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );

        delay_uS(10); 
      
        GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
        GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
    } else {        
        GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
        
        delay_uS(6); 
        
        GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
        //PRINTF(">");
    }
    
    powerOffMotors();
*/
}

/******************************************************************************/
/*!   \fn void rampStepMotors( void )

      \brief
        This function steps both the main and takeup motors. Used in rampMotors().
        
      \author
          Chris King
*******************************************************************************/
void rampStepMotors( void )
{
    powerOnMotors();
  
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );

    // step pin minimum high/low time is 1uS 
    delay_uS(10); 
        
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
    
     vScopeRecordPHStep();
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    GPIO_WritePinOutput( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN, true );

    // step pin minimum high/low time is 1uS 
    delay_uS(5); 
        
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false ); 
    GPIO_WritePinOutput( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN, false );
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
    GPIO_WritePinOutput( ACCEL_SPI_MOSI_GPIO, ACCEL_SPI_MOSI_PIN, true );
    
    delay_uS(10);
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false ); 
    GPIO_WritePinOutput( ACCEL_SPI_MOSI_GPIO, ACCEL_SPI_MOSI_PIN, false );
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

void releaseFromReset( DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
        GPIO_WritePinOutput( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, true );      
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, true );      
    }    
}

/******************************************************************************/
/*!   \fn static void setEightStepMode( DVR8818Type type )

      \brief
        This function resets the motor.
        
      \author
          Aaron Swift
*******************************************************************************/
void resetMotor(  DVR8818Type type )
{
    if( type == _MAIN_STEPPER ) { 
         GPIO_WritePinOutput( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, false );           
    } else {
        GPIO_WritePinOutput( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, false );        
    }    
}

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

void unitTestSpinMotors( void )
{
        initializeMotors( FORWARDM_ );            
        powerOnMotors(); 
        stepMotors();
        stepMotors();
        stepMotors();
        stepMotors();
        stepMotors();
        stepMotors();
}

bool getMotorPowerStatus( void )
{
    return powerStatus;
}

