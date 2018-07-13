// ***********************************************************************
// 
// Filename:      StateMachineMinSpec0.c
// Purpose:       Implementations/function definitions for state machine 
//                  module.
// Date Created:  03/12/2011  
//				        
// Author:        Robert T. Casey, Russell Petersen.
//
//
// ***********************************************************************

// HEADER FILES / INCLUDES -----------------------------------------------

#include <stdio.h>
#include <stdlib.h> /* needed for rand() */
#include <roachlib.h>  // even needed?
#include <ses.h>
#include <timers12.h>
#include <pwms12.h> 

#include "StateMachine.h"
#include "AnalogInput.h"
#include "Actuators.h"
#include "Sensors.h"


// MACROS AND CONSTANTS --------------------------------------------------
//#define DBG_STATE_MODE
#define IR_BUFFER_SIZE   16
#define TAPE_BUFFER_SIZE 16

// MODULE LEVEL GLOBALS -------------------------------------------------
static int NumGores;

// IR Rangefinder
int CurrFrontRange[ IR_BUFFER_SIZE ];
int CurrFrontRangePtr;
int LastFrontRange;

// Tape Sensors
int tFL[ TAPE_BUFFER_SIZE ];
int tFC[ TAPE_BUFFER_SIZE ];
int tFR[ TAPE_BUFFER_SIZE ];
int CurrTapePtrFC;
int CurrTapePtrFL;
int CurrTapePtrFR;



/* Function
      stateMachine
 
	All of the event servicers call this function, which implements the state machine. Note
 use of the static to keep the state between function calls. 
  
 */

void StateMachine(event_t event)
{
	static state_t state = Initial;
	
	switch (state)
	{
		case Initial:			// ---------------------------------------
				#ifdef DBG_STATE_MODE
					printf("Initial \r\n" );
.					printf("IR: %d \r\n", GetCurrRangeFinder() );
				#endif
			CurrFrontRangePtr = 0;
			CurrTapePtrFC = 0;
			CurrTapePtrFL = 0;
			CurrTapePtrFR = 0;

			state = SpinSearching;
			startSpinSearching();
		break;	// case Initial

		case SpinSearching:		// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("SpinSearching \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() );
			  #endif
			switch( event )
			{
				case eTimerOne:
					state = FwdSearching;
					startFwdSearching();
				break; // end eTimerOne
        
			  case eTimerTwo:	
			     state = PollingIR;
			     startPollingIR();
			  break;
/*				case eRangeFinderLessThan3Feet:
  					state = FwdSearching;
  					startFwdSearching();
				break; // end eRangeFinderLessThan3Feet
*/				
			} // end switch (event )
	  
 		  break;  // SpinSearching

	  case PollingIR:
	    switch( event )
	    {
        case eRangeFinderLessThan3Feet:	      
            state = FwdSearching;
            startFwdSearching();
        break;
        
        default:
    			state = SpinSearching;
    			startSpinSearching();
  			break;
	    }
	  break;

		case FwdSearching:		// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("FwdSearching \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() );
			  #endif
			switch( event )
			{
				case eTimerTwo:
					state = SpinSearching;
					startSpinSearching();
				break;  // eTimerTwo

				case eTapeFrontLeft:
					state = EvadingRightClearTape;
					startEvadingRightClearTape();
				break; // eTapeFrontLeft

				case eTapeFrontRight:
					state = EvadingLeftClearTape;
					startEvadingLeftClearTape();
				break; // eTapeFrontRight

				case eTapeFrontCenter:
					state = FwdStopping;
					startFwdStopping();
				break; // eTapeFrontCenter

				case eBumperFront:  // Best case
					state = FindingBot;
					startFindingBot();
				break;
			} // end switch( event )

		break;  // FwdSearching

		case EvadingRightClearTape:			// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("EvadingRightClearTape \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() );
			  #endif
			switch( event )
			{
				case eTimerTwo:			
					state = EvadingRightSpin;
					startEvadingRightSpin();
				break;  // eTimerTwo
			} // end switch( event )

		break;  // EvadingRight

		case EvadingRightSpin:			// ---------------------------------------
				#ifdef DBG_STATE_MODE
					printf("EvadingRightSpin \r\n" );
					printf("IR: %d \r\n", GetCurrRangeFinder() );
				#endif
			switch( event )
			{
				case eTimerTwo:			
					state = FwdSearching;
					startFwdSearching();
				break;  // eTimerTwo
			} // end switch( event )
		break;  // EvadingRight

		case EvadingLeftClearTape:			// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("EvadingLeftClearTape \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() ) ;
			  #endif
			switch( event )
			{
				case eTimerTwo:			
					state = EvadingLeftSpin;
					startEvadingLeftSpin();
				break;  // eTimerTwo
			} // end switch( event )
		break;  // EvadingLeft

		case EvadingLeftSpin:			// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("EvadingLeftSpin \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() ) ;
			  #endif
			switch( event )
			{
				case eTimerTwo:			
					state = FwdSearching;
					startFwdSearching();
				break;  // eTimerTwo
			} // end switch( event )
		break;  // EvadingLeft

		case FwdStopping:			// ---------------------------------------
		  #ifdef DBG_STATE_MODE
			  printf("FwdStopping \r\n" );
		  #endif
			switch( event )
			{
				case eTimerTwo:			
					state = EvadingRightSpin;
					startEvadingRightSpin();
				break;  // eTimerTwo
			}  // end switch( event )
		break; // FwdStopping

		case FindingBot:			// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("FindingBot \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() );
			  #endif
			switch( event )
			{
				case eTimerTwo:
					if( GetBotMode() == TOREADOR_MODE )
					{
						state = FiringLocation;
						startFiringLocation();
					} 
					else // BULL_MODE
					{
						state = GoreTime;
						startGoreTime();
					} // end if-else
				break; // eTimerTwo
			}  // end switch (event)
		break;  // FindingBot
    
		case FiringLocation:		// ---------------------------------------
				#ifdef DBG_STATE_MODE
					printf("FiringLocation \r\n" );
					printf("IR: %d \r\n", GetCurrRangeFinder() );
				#endif
			switch( event )
			{ 
				case eTimerOne:
					state = FiringCannon;
					startFiringCannon();
				break; // eTimerOne
			} // switch(event)
		break;  // Firing Location
	
		case FiringCannon:			// ---------------------------------------
			  #ifdef DBG_STATE_MODE
				  printf("FiringCannon \r\n" );
				  printf("IR: %d \r\n", GetCurrRangeFinder() );
			  #endif
			switch( event )
			{
				case eTimerOne:
					state = TogglingFeederArm;
					startTogglingFeederArm();
				break; // eTimerOne
			} // switch(event)
		break;  // FiringCannon
	
		case TogglingFeederArm:		// ---------------------------------------
				#ifdef DBG_STATE_MODE
					printf("TogglingFeederArm \r\n" );
					printf("IR: %d \r\n", GetCurrRangeFinder() ) ;
				#endif
			switch( event )
			{
				case eTimerOne:
					// END STATE FOR TOREADOR
					state = TogglingFeederArm;
					startTogglingFeederArm();
				break; // eTimerOne
			} // switch(event)
		break; // TogglingFeederArm
		   
		case GoreTime:				// ---------------------------------------
				#ifdef DBG_STATE_MODE
					printf("GoreTime \r\n" );
					printf("IR: %d \r\n", GetCurrRangeFinder() ) ;
				#endif
			switch( event )
			{
				case eBumperFront: // best case
					state = FindingBot;
					startFindingBot();
				break;

				case eTimerZero:
					// We lost the toreador in a mad rush...find him!
					state = SpinSearching;
					startSpinSearching();
				break;

				// Charging, we have hit the boundary of the ring
				case eTapeFrontCenter:
					state = FwdStopping;
					startFwdStopping();
				break; // eTapeFrontCenter

				case eTapeFrontLeft:
					state = EvadingRightClearTape;
					startEvadingRightClearTape();
				break; // eTapeFrontLeft

				case eTapeFrontRight:
					state = EvadingLeftClearTape;
					startEvadingLeftClearTape();
				break; // eTapeFrontRight

				case eMaxGoresReached:
					state = Stopping;
					startStopping();
				break;

			} // switch event
		break;  // GoreTime
	
		case Stopping:				// ---------------------------------------
			// END STATE FOR BULL
			#ifdef DBG_STATE_MODE
				printf("Stopping \r\n" );
				printf("IR: %d \r\n", GetCurrRangeFinder() ) ;
			#endif
		break; // Stopping

	}	// end switch(state)
}	// end StateMachine


/***********************************************************************
 
 STATE TRANSITION FUNCTIONS. 

 These functions are called once
 when the state is entered. They are called by the state machine.
 The program should not stay in these functions; they should perform
 the actions to transition state and return immediately.
 
 There should be one and only one function for each state.
 
 If an action is to be performed for an extended amount of time, create
 a new state and use a timer to wait for the end of the state. That way
 the main loop is running at all times and it is easy to add more states
 and transitions.
 
 */
// ***********************************************************************


// ***********************************************************************

void startSpinSearching()
{

	DriveStop();  // in case we were previously moving fast
	//Wait();
	Spin( RIGHT, DRIVE_SLOW );
  TMRS12_InitTimer( TIMER_1, SPIN_SEARCH_TIME );
  TMRS12_InitTimer( TIMER_2, IR_POLLING_TIME );
}

// ***********************************************************************

void startPollingIR()
{
  LastFrontRange = CurrFrontRange[ CurrFrontRangePtr ];
  
  if( CurrFrontRangePtr + 1 >= IR_BUFFER_SIZE )
  {
    CurrFrontRangePtr = 0;
  }
  else
  {
    ++CurrFrontRangePtr;  
  }
  
  CurrFrontRange[ CurrFrontRangePtr ] = ReadIRRangeFinder( rf_FRONT );
  
  
}

// ***********************************************************************

void startFwdSearching()
{
    DriveForward( DRIVE_SLOW );
  	TMRS12_InitTimer( TIMER_2, DRIVE_FWD_TIME );
}

// ***********************************************************************

void startTracking()
{
	// Action already handled by IRQ routine
	// Empty/placeholder state
	TMRS12_InitTimer( TIMER_1, TRACKING_TIME );
}

// ***********************************************************************

void startHoneTracking()
{
	Spin( LEFT, DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_1, HONE_TRACKING_TIME ) ;
}

// ***********************************************************************

void startEvadingRightClearTape()
{
	DriveStop();
	//Wait();
	DriveReverse( DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_2, EVADE_BACKUP_TIME ) ;
}

void startEvadingRightSpin()
{
	Spin( RIGHT, DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_2, EVADE_RIGHT_TIME ) ;
}

// ***********************************************************************

void startEvadingLeftClearTape()
{
	DriveStop();
	//Wait();
	DriveReverse( DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_2, EVADE_BACKUP_TIME ) ;
}

void startEvadingLeftSpin()
{
	Spin( LEFT, DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_2, EVADE_LEFT_TIME ) ;
}

// ***********************************************************************

void startFwdStopping()
{
	DriveStop();
	//Wait();
	DriveReverse( DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_2, EVADE_BACKUP_TIME ) ;
}

// ***********************************************************************

void startFindingBot()
{
	DriveReverse( DRIVE_SLOW );
	if( GetBotMode() == TOREADOR_MODE )
	{
		TMRS12_InitTimer( TIMER_2, FINDING_BOT_BACKUP_TOR ) ;
	}
	else // BULL_MODE
	{
		TMRS12_InitTimer( TIMER_2, FINDING_BOT_BACKUP_BULL ) ;
	}
}

// ***********************************************************************

void startFiringLocation()
{
	DriveStop();
//	Wait();
	Spin( LEFT, DRIVE_SLOW );
	TMRS12_InitTimer( TIMER_1, NINETY_DEGREE_TURN ) ;
}

// ******************************22*****************************************

void startFiringCannon()
{
	DriveStop();
	DriveMotor( FAN, NONE, mdNONE, CANNON_FIRE_CLOSE_RANGE );
	DriveMotor( SERVO, NONE, mdNONE, FEEDER_SERVO_OPEN );  
	TMRS12_InitTimer( TIMER_1, FEEDER_ARM_SLEW_TIME ) ;
}

// ***********************************************************************

void startTogglingFeederArm()
{
	static int FeederState = 0;

	if( FeederState == 0 ) // initial state here is open feeder
	{
		// Seal the tube
		DriveMotor( SERVO, NONE, mdNONE, FEEDER_SERVO_CLOSED ); 
		FeederState = 1;
	}
	else // FeederState == 1
	{
		// Feed more balls
		DriveMotor( SERVO, NONE, mdNONE, FEEDER_SERVO_OPEN ); 
		FeederState =  0;
	}

	TMRS12_InitTimer( TIMER_1, FEEDER_ARM_SLEW_TIME ) ;
}

// ***********************************************************************

void startGoreTime()
{
	NumGores++;
	DriveForward( DRIVE_FAST ); 
	TMRS12_InitTimer( TIMER_0, BULL_RUSH_TIME ) ;
}

// ***********************************************************************

void startStopping()
{
	DriveStop();
}

//
// -----------------------------------------------------------------------
//				EVENT-CHECKER, SERVICE ROUTINES
// -----------------------------------------------------------------------
//
// ***********************************************************************
//					TIMER TESTING
// ***********************************************************************

uchar TestForTimerZero( EVENT_PARAM )
{
	return( TMRS12_IsTimerExpired( TIMER_0 ) );
}

void RespForTimerZero( SERVICE_PARAM )
{
	StateMachine( eTimerZero );
}

// --------------------------------------------------------

uchar TestForTimerOne( EVENT_PARAM )
{
	return( TMRS12_IsTimerExpired( TIMER_1 ) );
}

void RespForTimerOne( SERVICE_PARAM )
{
	StateMachine( eTimerOne );
}

// --------------------------------------------------------

uchar TestForTimerTwo( EVENT_PARAM )
{
	return( TMRS12_IsTimerExpired( TIMER_2 ) );
}

void RespForTimerTwo( SERVICE_PARAM )
{
	StateMachine( eTimerTwo );
}

// ***********************************************************************

uchar TestForMaxGores( EVENT_PARAM )
{
   static char WasAtMaxGores = 0;
   // Test for event
   char EventOccurred = ( NumGores >= MAX_GORES_B4_QUIT ) && !WasAtMaxGores;

   // Update the state being tested
   WasAtMaxGores = ( NumGores >= MAX_GORES_B4_QUIT );
   return EventOccurred;
}

void RespForMaxGores( SERVICE_PARAM )
{
	StateMachine( eMaxGoresReached );
}

// ***********************************************************************

uchar TestFor_RF_LessThan_5FT( EVENT_PARAM )
{
   static char WasRangeLessThan5Ft = 0;

   // Test for event
   char EventOccurred = ( ( CurrFrontRange[ CurrFrontRangePtr ] > IRRF_FRONT_5FT_THRESH ) &&
	   !WasRangeLessThan5Ft );

   // Update the state being tested
   WasRangeLessThan5Ft = ( CurrFrontRange[ CurrFrontRangePtr ] > IRRF_FRONT_5FT_THRESH );

   return EventOccurred;   
}

void RespFor_RF_LessThan_5FT( SERVICE_PARAM )
{
	StateMachine( eRangeFinderLessThan5Feet );  
}

// ***********************************************************************

uchar TestFor_RF_LessThan_3FT( EVENT_PARAM )
{
   static char WasRangeLessThan3Ft = 0;

   // Test for event
   char EventOccurred = ( ( CurrFrontRange[ CurrFrontRangePtr ] > IRRF_FRONT_3FT_THRESH ) &&
	   !WasRangeLessThan3Ft );

   // Update the state being tested
   WasRangeLessThan3Ft = ( CurrFrontRange[ CurrFrontRangePtr ] > IRRF_FRONT_3FT_THRESH );

   return EventOccurred;   
}

void RespFor_RF_LessThan_3FT( SERVICE_PARAM )
{
	StateMachine( eRangeFinderLessThan3Feet );  
}

// ***********************************************************************

uchar TestForLostTracking( EVENT_PARAM )
{
   static char HadLostTracking = 0;
   char EventOccurred = ( ( GetCurrRangeFinder() > GetLastRangeFinder() ) &&
	   !HadLostTracking);

   HadLostTracking = ( GetCurrRangeFinder() > GetLastRangeFinder() );

   return EventOccurred;
}

void RespForLostTracking( SERVICE_PARAM )
{
  StateMachine( eLostTracking );
}

// ***********************************************************************
//					BUMPER TESTING
// ***********************************************************************

uchar TestForBumperLeft(EVENT_PARAM)
{
    static char LeftWasBumped = 0;
	
    char EventOccured = ( ReadSwitch( sw_LEFT_BUMPER ) && !LeftWasBumped );
    
    LeftWasBumped = ReadSwitch( sw_LEFT_BUMPER );
    
    return (EventOccured);  
}

void RespForBumperLeft(SERVICE_PARAM)
{
	StateMachine( eBumperLeft );
}

// ***********************************************************************

uchar TestForBumperRight(EVENT_PARAM)
{
    static char RightWasBumped = 0;
	
    char EventOccured = ( ReadSwitch( sw_RIGHT_BUMPER ) && !RightWasBumped );
    
    RightWasBumped = ReadSwitch( sw_RIGHT_BUMPER );
    
    return (EventOccured);  
}

void RespForBumperRight(SERVICE_PARAM)
{
	StateMachine( eBumperRight );
}

// ***********************************************************************

uchar TestForBumperFront(EVENT_PARAM)
{
    static char FrontWasBumped = 0;
	
    char EventOccured = ( ReadSwitch( sw_FRONT_BUMPER ) && !FrontWasBumped );
    
    FrontWasBumped = ReadSwitch( sw_FRONT_BUMPER );
    
    return (EventOccured);  	
}

void RespForBumperFront(SERVICE_PARAM)
{
	StateMachine( eBumperFront );
}
	
// ***********************************************************************

uchar TestForBumperRear(EVENT_PARAM)
{
    static char RearWasBumped = 0;
	
    char EventOccured = ( ReadSwitch( sw_REAR_BUMPER ) && !RearWasBumped );
    
    RearWasBumped = ReadSwitch( sw_REAR_BUMPER );
    
    return (EventOccured);  	
}

void RespForBumperRear(SERVICE_PARAM)
{
	StateMachine( eBumperRear );
}

// ***********************************************************************
//					TAPE TESTING
// ***********************************************************************

uchar TestForTapeFrontLeft(EVENT_PARAM)
{
    static char TapeWasFound = 0;
	
    char EventOccured = ( ( ReadTapeSensor( ts_FRONT_LEFT, tsrBINARY ) == ts_TAPE ) 
		&& !TapeWasFound );
    
    TapeWasFound = ( ReadTapeSensor( ts_FRONT_LEFT, tsrBINARY ) == ts_TAPE );
    
    return (EventOccured);  	
}

void RespForTapeFrontLeft(SERVICE_PARAM)
{
	StateMachine( eTapeFrontLeft );
}

// ***********************************************************************

uchar TestForTapeFrontRight(EVENT_PARAM)
{
    static char TapeWasFound = 0;
	
    char EventOccured = ( ( ReadTapeSensor( ts_FRONT_RIGHT, tsrBINARY ) == ts_TAPE )
		&& !TapeWasFound );
    
    TapeWasFound = ( ReadTapeSensor( ts_FRONT_RIGHT, tsrBINARY ) == ts_TAPE );
    
    return (EventOccured);  	
}

void RespForTapeFrontRight(SERVICE_PARAM)
{
	StateMachine( eTapeFrontRight );
}

// ***********************************************************************

uchar TestForTapeFrontCenter(EVENT_PARAM)
{
    static char TapeWasFound = 0;
	
    char EventOccured = ( (ReadTapeSensor( ts_FRONT_CENTER, tsrBINARY ) == ts_TAPE) 
		&& !TapeWasFound );
    
    TapeWasFound = ( ReadTapeSensor( ts_FRONT_CENTER, tsrBINARY ) == ts_TAPE );
    
    return (EventOccured);  	
}

void RespForTapeFrontCenter(SERVICE_PARAM)
{
	StateMachine( eTapeFrontCenter );
}

// ***********************************************************************

uchar TestForTapeRearLeft(EVENT_PARAM)
{
    static char TapeWasFound = 0;
	
    char EventOccured = ( ( ReadTapeSensor( ts_REAR_LEFT, tsrBINARY ) == ts_TAPE )
		&& !TapeWasFound );
    
    TapeWasFound = ( ReadTapeSensor( ts_REAR_LEFT, tsrBINARY ) == ts_TAPE );
    
    return (EventOccured);  	
}

void RespForTapeRearLeft(SERVICE_PARAM)
{
	StateMachine( eTapeRearLeft );
}

// ***********************************************************************

uchar TestForTapeRearRight(EVENT_PARAM)
{
    static char TapeWasFound = 0;
	
    char EventOccured = ( ( ReadTapeSensor( ts_REAR_RIGHT, tsrBINARY ) == ts_TAPE ) 
		&& !TapeWasFound );
    
    TapeWasFound = ( ReadTapeSensor( ts_REAR_RIGHT, tsrBINARY ) == ts_TAPE );
    
    return (EventOccured);  	
}

void RespForTapeRearRight(SERVICE_PARAM)
{
	StateMachine( eTapeRearRight );
}

// ***********************************************************************

uchar TestForTapeRearCenter(EVENT_PARAM)
{
    static char TapeWasFound = 0;
	
    char EventOccured = ( ( ReadTapeSensor( ts_REAR_CENTER, tsrBINARY ) == ts_TAPE ) 
		&& !TapeWasFound );
    
    TapeWasFound = ( ReadTapeSensor( ts_REAR_CENTER, tsrBINARY ) == ts_TAPE );
                                                 
    return (EventOccured);  	
}

void RespForTapeRearCenter(SERVICE_PARAM)
{
	StateMachine( eTapeRearCenter ); 
}

// ***********************************************************************

void Wait( void )
{
	int i;
	for( i = 0; i < 3000; ++i )
	{
		// sit & spin (for 1ms)!
		// yes, it's blocking code, get over it.
	}
}