// ***********************************************************************
// 
// Filename:      SM_SpinSearch.c
// Purpose:       Implementations/function definitions for state machine 
//                  module doing spin searching.
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

#include "SM_SpinSearch.h"
#include "AnalogInput.h"
#include "Actuators.h"
#include "Sensors.h"


// MACROS AND CONSTANTS --------------------------------------------------
//#define DBG_STATE_MODE

// MODULE LEVEL GLOBALS -------------------------------------------------
static int NumGores;

/* Function
      stateMachine
 
	All of the event servicers call this function, which implements the state machine. Note
 use of the static to keep the state between function calls. 
  
 */

void StateMachine(event_t event)
{
	static state_t state = Initial;
	
	switch( state )
	{
	  case Initial:	
	    printf( "Initi. State\r\n");
	    printf( "IR: %d\r\n", GetCurrRangeFinder() );		
			state = SpinSearching;
			startSpinSearching();
		break;	// case Initial

		case SpinSearching:		
	    printf( "SpinSearch State\r\n");
	    printf( "IR: %d\r\n", GetCurrRangeFinder() );			
			switch( event )
			{
				case eRangeFinderLessThan3Feet:
				    printf( "eRangeFinderLTFeet event\r\n");
  					state = Stopping;
  					startStopping();
				break; // end eRangeFinderLessThan3Feet
			} // end switch (event )
	  
 		  break;  // SpinSearching
 		  
		case Stopping:				// ---------------------------------------
		  printf( "Stopping State\r\n");
		  printf( "IR: %d\r\n", GetCurrRangeFinder() );			
		break; // Stopping

	}// end switch(state)
}// end StateMachine


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
	Spin( RIGHT, DRIVE_SLOW );
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

uchar TestFor_RF_LessThan_3FT( EVENT_PARAM )
{
   static char WasRangeLessThan3Ft = 0;

   // Test for event
   char EventOccurred = ( ( GetCurrRangeFinder() > IRRF_FRONT_3FT_THRESH ) &&
	   !WasRangeLessThan3Ft );

   // Update the state being tested
   WasRangeLessThan3Ft = ( GetCurrRangeFinder() > IRRF_FRONT_3FT_THRESH );

   return EventOccurred;   
}

void RespFor_RF_LessThan_3FT( SERVICE_PARAM )
{
	StateMachine( eRangeFinderLessThan3Feet );  
}

// ***********************************************************************