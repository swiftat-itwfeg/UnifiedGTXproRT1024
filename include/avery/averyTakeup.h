#ifndef AVERYTAKEUP_H
#define AVERYTAKEUP_H

#define AIM_PAPER_TENSION			14000					// 10000 //20000  // in milliNewtons
#define MEDIUM_PAPER_TENSION		        (AIM_PAPER_TENSION*3/4) // used for pre-tensioning before print
#define LIGHT_PAPER_TENSION			(AIM_PAPER_TENSION/2)   // enough to take up the slack and get a reading
#define VERY_LIGHT_PAPER_TENSION	        (AIM_PAPER_TENSION/2)   // just enough to take up the slack and get a reading

#define TENSIONER_RATIOx1000		        1704					// gear ration between take up spool and tensioner arm
#define TORQUE_CONSTANT				(25000*1000/TENSIONER_RATIOx1000) // (milliNmm/degree) this is how much torque required to twist the takeup spool by one degree
#define LIGHT_TORQUE				(4*TORQUE_CONSTANT)		// milliNmm - low, but measurable torque
#define GEAR_RATIOx1000				7890					// This is the take-up gear ratio multiplied by 1000
#define DEGREES_PER_STEPx100		        375 					// ie 3.75 degrees per motor step (half step really)
#define MAX_TWIST				(30000*TENSIONER_RATIOx1000/1000)	// Max angular deflection (in 0.001 degree increments) that takeup spool can move, where its motor is stationary
#define TU_MIN_DIAMETER				30						// diameter of take up spool in mm
#define TU_MAX_DIAMETER				60						// should never exceed this diameter


#endif