// ***********************************************************************
// 
// Filename:      SM_SpinSearch.h
// Purpose:       Prototypes/function declarations for state machine 
//                  module.
// Date Created:  02/28/2011  
//				  Adapted from Lab0 code.
// Author:        Robert T. Casey
//
//
// ***********************************************************************

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// MACROS AND CONSTANTS --------------------------------------------------

// Timers (gone wild!)
#define TIMER_0             0
#define TIMER_1             1
#define TIMER_2             2

/* the timer runs at about 5msec per tick */
#define QUARTER_SEC           61
#define THIRD_SEC			        81
#define HALF_SEC              122
#define ONE_SEC               244  
#define NINETY_DEGREE_TURN    348
#define TWO_SEC               488
#define THREE_SEC             732
#define FOUR_SEC              976
#define FIVE_SEC              1220
#define FORTY_FIVE_SEC        10980

#define TIME_INTERVAL			      TWO_SEC
#define SPIN_SEARCH_TIME		    FIVE_SEC
#define TRACKING_TIME			      FIVE_SEC
#define HONE_TRACKING_TIME		  QUARTER_SEC
#define DRIVE_FWD_TIME			    FOUR_SEC
#define EVADE_LEFT_TIME			    ONE_SEC
#define EVADE_RIGHT_TIME		    ONE_SEC
#define EVADE_BACKUP_TIME		    ONE_SEC
#define FINDING_BOT_BACKUP_TOR  ONE_SEC
#define FINDING_BOT_BACKUP_BULL ONE_SEC
#define FEEDER_ARM_SLEW_TIME    HALF_SEC
#define BULL_RUSH_TIME		    	THREE_SEC


#define MAX_GORES_B4_QUIT		3

// ENUMERATIONS AND OTHER TYPES  -----------------------------------------
typedef enum 
{	Initial, 
	SpinSearching, 
	Stopping
	
} state_t ;

typedef enum 
{ 
	eDummy,
	eRangeFinderLessThan3Feet,
} event_t;

// FUNCTIONS  -----------------------------------------------------------------

void StateMachine(event_t);
void startSpinSearching();
void startStopping();

// -----------------------------------------------------------------------
// EVENT-CHECKER, SERVICE ROUTINES
// -----------------------------------------------------------------------

uchar TestFor_RF_LessThan_3FT( EVENT_PARAM ); 
void RespFor_RF_LessThan_3FT( SERVICE_PARAM );


#endif