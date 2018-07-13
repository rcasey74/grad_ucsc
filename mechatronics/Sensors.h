// ***********************************************************************
// 
// Filename:      Sensors.h
// Purpose:       Prototypes/function declarations for all sensor modules.
// Date Created:  02/25/2011  
// Author:        Robert T. Casey
//
// ***********************************************************************

#ifndef SENSORS_H
#define SENSORS_H

// MACROS AND CONSTANTS --------------------------------------------------

#define NUM_TAPE_SENSORS		  6
#define NUM_BEACON_SENSORS		8
#define NUM_SWITCHES			    6
#define NUM_IR_RANGEFINDERS		2

#define SWITCH_HIGH				1
#define SWITCH_LOW				0

//#define BULL_MODE				1
//#define TOREADOR_MODE			0

#define IRRF_FRONT_5FT_THRESH	79
#define IRRF_FRONT_4FT_THRESH	109
#define IRRF_FRONT_3FT_THRESH	139 // ***
#define IRRF_FRONT_2FT_THRESH 198
#define IRRF_FRONT_1FT_THRESH	379 // ***

#define TAPE_SENSOR_THRESH_ON   125
#define TAPE_SENSOR_THRESH_OFF  115



// ENUMERATIONS AND OTHER TYPES  -----------------------------------------

enum eRobotMode
{
  TOREADOR_MODE = 0,  BULL_MODE 
};

enum SensorInit{ sns_PRE_INIT = -1, sns_INIT_SUCCESS, sns_INIT_FAILURE };

enum TapeSensorID
{
  ts_REAR_RIGHT   = 0, 
  ts_FRONT_LEFT   = 1, 
  ts_FRONT_CENTER = 2, 
  ts_FRONT_RIGHT  = 3,
  ts_REAR_LEFT    = 4,      
  ts_REAR_CENTER  = 5  
};
  
enum TapeSensorReturn
{
	ts_MDF, ts_TAPE
};

enum TapeSensorRetMode
{
  tsrBINARY,
  tsrRAW  
};

struct TapeThresholds
{
   int FrontCenter;
   int FrontLeft;
   int FrontRight;
   int RearCenter;
   int RearLeft;
   int RearRight;
};

enum BeaconSensorID
{
  bs_FRONT_LEFT = 0, bs_FRONT_CENTER, bs_FRONT_RIGHT,
  bs_MID_LEFT,       bs_MID_RIGHT,
  bs_REAR_LEFT,      bs_REAR_CENTER,  bs_REAR_RIGHT
};
  
enum SwitchID
{
  sw_RIGHT_BUMPER = 0,
  sw_LEFT_BUMPER  = 1,
  sw_REAR_BUMPER  = 2,
  sw_FRONT_BUMPER = 3, 
  sw_BULL_TOR_MODE= 4, 
  sw_GO           = 5
};

enum RangefinderID
{
  rf_FRONT = 0, 
  rf_CANNON 
};

// FUNCTIONS  -----------------------------------------------------

int InitSensors( void );
void CalibrateTapeSensors( void );

enum eRobotMode GetBotMode( void );

short ReadTapeSensor( enum TapeSensorID p_tsID, enum TapeSensorRetMode p_Mode );

short ReadBeaconSensor( enum BeaconSensorID );
enum BeaconSensorID GetMaxBeaconReturnID( void );
enum BeaconSensorID GetMinBeaconReturnID( void );

short ReadSwitch( enum SwitchID );

short ReadIRRangeFinder( enum RangefinderID );
short GetCurrRangeFinder( void );
short GetLastRangeFinder( void );
               
// *************************************************************************
// end file
#endif