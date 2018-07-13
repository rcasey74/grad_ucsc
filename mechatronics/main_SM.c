// ***********************************************************************
// 
// Filename:      main_SM.c
// Purpose:       Entry point to Bull/Toreador robot application.
// Date Created:  02/28/2011  
//				  Adapted from Lab0 code, SES code.
// Author:        Robert T. Casey
//
//
// ***********************************************************************

// ***********************************************************************
//            H E A D E R    F I L E S
// ***********************************************************************

#include <stdio.h>
#include <stdlib.h> /* needed for rand() */
#include <ses.h>
#include <timers12.h>
#include <pwms12.h> 

#include "StateMachine.h"
#include "AnalogInput.h"
#include "Actuators.h"
#include "Sensors.h"


// ***********************************************************************
//            C O N S T A N T S   /   M A C R O S
// ***********************************************************************

/*---------------------------- Module Variables ---------------------------*/

/*------------------------------ Module Code ------------------------------*/

// ***********************************************************************
//              M   A   I   N                          
// ***********************************************************************

void main(void)
{
	// Local variable declarations
	enum SensorInit   SensorInitStatus   = sns_PRE_INIT;
  enum ActuatorInit ActuatorInitStatus = act_PRE_INIT;
    
    // Initializations
	EnableInterrupts;
	TERMIO_Init();
  SensorInitStatus   = InitSensors();
  
  printf("Calibrated Thresholds: \r\n" );
  printf("FC %d,", GetThreshFC() );
  printf("FL %d,", GetThreshFL() );
  printf("FR %d,", GetThreshFR() );
  printf("RC %d,", GetThreshRC() );
  printf("RL %d,", GetThreshRL() );
  printf("RR %d,", GetThreshRR() );

  //while(1); // chill
  
  ActuatorInitStatus = InitActuators();
    
 /* TESTING: SUCCESS!
 printf( "BotMode = " );  
  if( GetBotMode() == BULL_MODE )
  {
     printf( "BULL" );
  }
  
   else if( GetBotMode() == TOREADOR_MODE )
   {
      printf( "TOREADOR" );    
   }
   */

    
	SES_Init( SES_ROUND_ROBIN, SES_NO_UPDATE );
    TMRS12_Init( TMRS12_RATE_4MS );
    	
    // Register event/service routines with SES
	SES_Register( TestForTimerZero,			  RespForTimerZero );
	SES_Register( TestForTimerOne,			  RespForTimerOne );
	SES_Register( TestForTimerTwo,			  RespForTimerTwo );

	SES_Register( TestForBumperFront,		  RespForBumperFront );
//	SES_Register( TestForBumperLeft,		  RespForBumperLeft );
//	SES_Register( TestForBumperRight,		  RespForBumperRight );
//	SES_Register( TestForBumperRear,		  RespForBumperRear );

	//SES_Register( TestForTapeFrontLeft,		RespForTapeFrontLeft );
	//SES_Register( TestForTapeFrontRight,	RespForTapeFrontRight );
	//SES_Register( TestForTapeFrontCenter,	RespForTapeFrontCenter );
//	SES_Register( TestForTapeRearLeft,		RespForTapeRearLeft );
//	SES_Register( TestForTapeRearRight,		RespForTapeRearRight );
//	SES_Register( TestForTapeRearCenter,	RespForTapeRearCenter );

	SES_Register( TestFor_RF_LessThan_5FT,  RespFor_RF_LessThan_5FT );
	SES_Register( TestFor_RF_LessThan_3FT,  RespFor_RF_LessThan_3FT );	
	
	SES_Register( TestForLostTracking,			RespForLostTracking );
	
	SES_Register( TestForMaxGores,				RespForMaxGores );

	/*    SES_Register(TimeOut, TimeOutResp);     */
	
	TMRS12_InitTimer(TIMER_0, TIME_INTERVAL);
	
	StateMachine( eDummy );   /* "initial" event */
	
	// Main program loop
    while(1) 
	{  
	    SES_HandleEvents();
    }
}	// end main

