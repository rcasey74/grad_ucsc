// ***********************************************************************
// 
// Filename:      StateMachine.h
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
#define IR_POLLING_TIME         QUARTER_SEC
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
	//Tracking, 
	//HoneTracking, 
	PollingIR,
	FwdSearching, 
	EvadingRightClearTape,
	EvadingRightSpin,
	EvadingLeftClearTape,
	EvadingLeftSpin,
	FwdStopping,
  FindingBot,
  FiringLocation,
	FiringCannon,
	TogglingFeederArm,
	GoreTime,
	Stopping
	
} state_t ;

typedef enum 
{ 
	eDummy,
	eBumperLeft, 
	eBumperRight, 
	eBumperFront, 
	eBumperRear, 
	eTapeFrontLeft,
	eTapeFrontRight,
	eTapeFrontCenter,
	eTapeRearLeft,
	eTapeRearRight,
	eTapeRearCenter,
  eTimerZero,
	eTimerOne,
	eTimerTwo,
	eRangeFinderLessThan5Feet,
	eRangeFinderLessThan3Feet,
	eLostTracking,
	eMaxGoresReached
} event_t;

// FUNCTIONS  -----------------------------------------------------------------

void StateMachine(event_t);
void startSpinSearching();
//void startTracking();
//void startHoneTracking();
void startPollingIR();
void startFwdSearching();
void startEvadingRightClearTape();
void startEvadingRightSpin();
void startEvadingLeftClearTape();
void startEvadingLeftSpin();
void startFwdStopping();
void startFindingBot();
void startFiringLocation();
void startFiringCannon();
void startTogglingFeederArm();
void startGoreTime();
void startStopping();

// -----------------------------------------------------------------------
// EVENT-CHECKER, SERVICE ROUTINES
// -----------------------------------------------------------------------

uchar TestFor_RF_LessThan_3FT( EVENT_PARAM );
void RespFor_RF_LessThan_3FT( SERVICE_PARAM );

uchar TestFor_RF_LessThan_5FT( EVENT_PARAM );
void RespFor_RF_LessThan_5FT( SERVICE_PARAM );

uchar TestForMaxGores( EVENT_PARAM );
void RespForMaxGores( SERVICE_PARAM );

uchar TestForLostTracking( EVENT_PARAM );
void RespForLostTracking( SERVICE_PARAM );

// --------------------------------------------------------


uchar TestForTimerZero( EVENT_PARAM );
void RespForTimerZero( SERVICE_PARAM );

uchar TestForTimerOne( EVENT_PARAM );
void RespForTimerOne( SERVICE_PARAM );

uchar TestForTimerTwo( EVENT_PARAM );
void RespForTimerTwo( SERVICE_PARAM );

// --------------------------------------------------------

uchar TestForBumperLeft(EVENT_PARAM);
void RespForBumperLeft(SERVICE_PARAM);

uchar TestForBumperRight(EVENT_PARAM);
void RespForBumperRight(SERVICE_PARAM);

uchar TestForBumperFront(EVENT_PARAM);
void RespForBumperFront(SERVICE_PARAM);
	
uchar TestForBumperRear(EVENT_PARAM);
void RespForBumperRear(SERVICE_PARAM);

// --------------------------------------------------------

uchar TestForTapeFrontLeft(EVENT_PARAM);
void RespForTapeFrontLeft(SERVICE_PARAM);

uchar TestForTapeFrontRight(EVENT_PARAM);
void RespForTapeFrontRight(SERVICE_PARAM);

uchar TestForTapeFrontCenter(EVENT_PARAM);
void RespForTapeFrontCenter(SERVICE_PARAM);
		
uchar TestForTapeRearLeft(EVENT_PARAM);
void RespForTapeRearLeft(SERVICE_PARAM);
	
uchar TestForTapeRearRight(EVENT_PARAM);
void RespForTapeRearRight(SERVICE_PARAM);

uchar TestForTapeRearCenter(EVENT_PARAM);
void RespForTapeRearCenter(SERVICE_PARAM);

// -----------------------------------------------------------------------
// UTILITY ROUTINES
// -----------------------------------------------------------------------

void Wait( void );

#endif