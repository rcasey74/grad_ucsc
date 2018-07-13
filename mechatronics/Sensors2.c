// ***********************************************************************
// 
// Filename:      Sensors.c
// Purpose:       Implementations/function definitions for sensor 
//                  modules.
// Date Created:  02/25/2011  
// Author:        Robert T. Casey
//
// ***********************************************************************

// HEADER FILES / INCLUDES -----------------------------------------------

#include <hidef.h>      /* common defines and macros */
#include <mc9s12c32.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c32"
#include <pwms12.h>
#include <termio.h>
#include <ads12.h>
#include <stdio.h>

#include "AnalogInput.h"
#include "Sensors.h"


// MACROS AND CONSTANTS --------------------------------------------------

#define CLIP_SCALE        11
#define INPUT_MAX       1023
#define DUTY_CYCLE_MAX   100

#define TRUE               1
#define FALSE              0

#define INIT_SUCCESS       1
#define INIT_FAILURE       0

#define TAPE_SENSOR_MUX	   1
#define BEACON_SENSOR_MUX  0
#define IR_RANGEFINDER_MUX 1

#define NUM_TAPE_SAMPLES  30

#define POINT_ONE_MILLISECONDS 300

#define TAPE_FRONT_LEFT_MUX_LINE       2
#define TAPE_FRONT_CENTER_MUX_LINE     1
#define TAPE_FRONT_RIGHT_MUX_LINE      6

#define TAPE_REAR_LEFT_MUX_LINE        7
#define TAPE_REAR_CENTER_MUX_LINE      5
#define TAPE_REAR_RIGHT_MUX_LINE       4

                       
#define BEACON_FRONT_LEFT_MUX_LINE      3
#define BEACON_FRONT_CENTER_MUX_LINE    1
#define BEACON_FRONT_RIGHT_MUX_LINE     5

#define BEACON_MID_LEFT_MUX_LINE        2
#define BEACON_MID_RIGHT_MUX_LINE       6

#define BEACON_REAR_LEFT_MUX_LINE       7
#define BEACON_REAR_CENTER_MUX_LINE     0
#define BEACON_REAR_RIGHT_MUX_LINE      4

#define IR_RANGEFINDER_FRONT_MUX_LINE   3    // PIN 12
#define IR_RANGEFINDER_CANNON_MUX_LINE  0    // PIN 13

#define TAPE_MDF_DELTA                  20
#define TAPE_TAPE_DELTA                 40


// -----------------------------------------------------------------------
// MODULE LEVEL GLOBALS
// -----------------------------------------------------------------------
enum eRobotMode BotMode;
static int iInfraredRangefinderCycle;
static int CurrFrontIRRange, LastFrontIRRange;
struct TapeThresholds TapeThresh;

// ***********************************************************************

enum eRobotMode GetBotMode( void )
{
    return BotMode;
}

// ***********************************************************************

int InitSensors( void )
{
   // ------------------------------------------------
   // Configure interrupts for infrared rangefinders
   // ------------------------------------------------

	// Set the scalar for a overflow of 21ms 
	// Note: This will affect stepper motor period as it uses the same module.
	TSCR2_PR = 0b011; 
  
	// Enable OC6 as output/compare
	// TIOS == Timer Input Capture/Output Compare Select
	// Configures channel for either input capture or output compare
	TIOS |= TIOS_IOS6_MASK; 

	//OC7M|=TIOS_IOS5_MASK; //make sure it is an output

	// Controls IRQ-initiated pin toggling (1/2)
	// TCTL1 == Timer Control Register 1
	TCTL1_OL6 = 1;  

	// Controls IRQ-initiated pin toggling (1/2)
	TCTL1_OM6 = 0;  

	// Enable timers
	// TSCR1 == Timer System Control Register 1
	// TEN_MASK = Timer ENable (1 = enable, 0 = disable)
	TSCR1 |= TSCR1_TEN_MASK; 

	// Set an initial time for first interrupt, arbitrarily chosen
	TC6 = TCNT + 2 << 8;    // Timer Input Capture/Output Compare Register 6

	// Clear the interrupt flag
	TFLG1 = TFLG1_C6F_MASK;  

	// Enable the interrupt
	TIE |= TIE_C6I_MASK;   
  
	EnableInterrupts; 
  
	// Set the interrupt to fire every 1/4 second
    iInfraredRangefinderCycle = POINT_ONE_MILLISECONDS * 2500;

   // -------------------------------------
   // Port A Configuration
   // -----------------------------------
   // Configure Port A Direction Register
   // Outputs = 1 (active  = high)
   // Inputs  = 0 (passive = low)
   // Bits  7 6 5 4 3 2 1 0
   //       0 0 1 1 1 1 1 1
   // PA0 - PA5 = Digital output
   // PA6 - PA7 = Unused
   /*
      PA0	Digital output	Analog mux addressing
      PA1	Digital output	Analog mux addressing
      PA2	Digital output	Analog mux addressing
      PA3	Digital output	Analog mux addressing
      PA4	Digital output	Analog mux addressing
      PA5	Digital output	Analog mux addressing
      PA6	Unused by sensor module
      PA7	Unused by sensor module
    */
   
   // 0011 1111
   DDRA |= 0x3F; // First 6 pins on PORTA set to be outputs.  
   
   // -------------------------------------
   // Port AN/AD configuration
   // -------------------------------------
   /*
      AN0	Analog  input	Multiplexer[0] COM Analog Out
      AN1	Analog  input	Multiplexer[1] COM Analog Out
      AN2	Digital input	Bumper switches
      AN3	Digital input	Bumper switches
      AN4	Digital input	Bumper switches
      AN5	Digital input	Bumper switches
      AN6	Digital input Mode Switch
      AN7 Digital input Go button
   */
   
   
   if( ADS12_Init( "IIIIIIAA" ) == ADS12_OK )
   {
      BotMode = (enum eRobotMode) ReadSwitch( sw_BULL_TOR_MODE );
      CalibrateTapeSensors();
      return sns_INIT_SUCCESS;
   } 
   else 
   {
      return sns_INIT_FAILURE;
   }
} // end InitSensors

// ***********************************************************************
// GENERAL MUX NOTES:
// 3 address lines control which of the 8 input lines
// makes it through to the mux's 1 output line
// ADDC    ADDB      ADDA
// MSB     middle    LSB

static short ReadMux( int p_MuxID, int p_MuxInputLine ) 
{
  short sMuxOutput = 0;
  int   iWaitTimer = 1;  // Used for mux setup time
  
  switch( p_MuxID )
  {
    case 0:   // Mux 0
    // Port mapping:  3/5/11
    // ADDC = PA1
    // ADDB = PA3
    // ADDA = PA5
      switch( p_MuxInputLine )
      {
        case 0:
          PORTA &=  ~PORTA_BIT1_MASK;     //  0
          PORTA &=  ~PORTA_BIT3_MASK;     //  0
          PORTA &=  ~PORTA_BIT5_MASK;     //  0   
        break;
        
        case 1:
          PORTA &=  ~PORTA_BIT1_MASK;     //  0
          PORTA &=  ~PORTA_BIT3_MASK;     //  0
          PORTA |=   PORTA_BIT5_MASK;     //  1    
        break;
        
        case 2:
          PORTA &=  ~PORTA_BIT1_MASK;     //  0
          PORTA |=   PORTA_BIT3_MASK;     //  1
          PORTA &=  ~PORTA_BIT5_MASK;     //  0    
        break;
        
        case 3:
          PORTA &=  ~PORTA_BIT1_MASK;     //  0 
          PORTA |=   PORTA_BIT3_MASK;     //  1 
          PORTA |=   PORTA_BIT5_MASK;     //  1     
        break;
        
        case 4:
          PORTA |=   PORTA_BIT1_MASK;     //  1  
          PORTA &=  ~PORTA_BIT3_MASK;     //  0  
          PORTA &=  ~PORTA_BIT5_MASK;     //  0      
        break;
        
        case 5:
          PORTA |=   PORTA_BIT1_MASK;     //  1  
          PORTA &=  ~PORTA_BIT3_MASK;     //  0  
          PORTA |=   PORTA_BIT5_MASK;     //  1      
        break;
        
        case 6:
          PORTA |=   PORTA_BIT1_MASK;     //  1  
          PORTA |=   PORTA_BIT3_MASK;     //  1  
          PORTA &=  ~PORTA_BIT5_MASK;     //  0      
        break;
        
        case 7:
          PORTA |=   PORTA_BIT1_MASK;     //  1
          PORTA |=   PORTA_BIT3_MASK;     //  1
          PORTA |=   PORTA_BIT5_MASK;     //  1    
        break;
        
        default:
        break;
        
      }   // end switch( pMuxInputLine )

      // Allow about 300ns for setup   
      // Testing showed that executing 2 instructions should 
      // cause a 400ns delay, sufficient for our purposes here
      iWaitTimer++;
      iWaitTimer--;
      
      //sMuxOutput = ReadADValue( AN0 );
      sMuxOutput = ReadADValue( AN1 );   // 3/6/11 change
      
    break;  // case 0
    
    // -------------------------------------------------------------------
    
    case 1:   // Mux 1
      // ADDC = PA4
      // ADDB = PA2
      // ADDA = PA0
      switch( p_MuxInputLine )
      {
        case 0:
          PORTA &=  ~PORTA_BIT4_MASK;     //  0
          PORTA &=  ~PORTA_BIT2_MASK;     //  0
          PORTA &=  ~PORTA_BIT0_MASK;     //  0   
        break;
        
        case 1:
          PORTA &=  ~PORTA_BIT4_MASK;     //  0
          PORTA &=  ~PORTA_BIT2_MASK;     //  0
          PORTA |=   PORTA_BIT0_MASK;     //  1    
        break;
        
        case 2:
          PORTA &=  ~PORTA_BIT4_MASK;     //  0
          PORTA |=   PORTA_BIT2_MASK;     //  1
          PORTA &=  ~PORTA_BIT0_MASK;     //  0    
        break;
        
        case 3:
          PORTA &=  ~PORTA_BIT4_MASK;     //  0 
          PORTA |=   PORTA_BIT2_MASK;     //  1 
          PORTA |=   PORTA_BIT0_MASK;     //  1     
        break;
        
        case 4:
          PORTA |=   PORTA_BIT4_MASK;     //  1  
          PORTA &=  ~PORTA_BIT2_MASK;     //  0  
          PORTA &=  ~PORTA_BIT0_MASK;     //  0      
        break;
        
        case 5:
          PORTA |=   PORTA_BIT4_MASK;     //  1  
          PORTA &=  ~PORTA_BIT2_MASK;     //  0  
          PORTA |=   PORTA_BIT0_MASK;     //  1      
        break;
        
        case 6:
          PORTA |=   PORTA_BIT4_MASK;     //  1  
          PORTA |=   PORTA_BIT2_MASK;     //  1  
          PORTA &=  ~PORTA_BIT0_MASK;     //  0      
        break;
        
        case 7:
          PORTA |=   PORTA_BIT4_MASK;     //  1
          PORTA |=   PORTA_BIT2_MASK;     //  1
          PORTA |=   PORTA_BIT0_MASK;     //  1    
        break;
        
        default:
        break;        
        
      }   // end switch( pMuxInputLine )
      
      // Allow about 300ns for setup   
      // Testing showed that executing 2 instructions should 
      // cause a 400ns delay, sufficient for our purposes here
      iWaitTimer++;
      iWaitTimer--;

      //sMuxOutput = ReadADValue( AN1 );
      sMuxOutput = ReadADValue( AN0 );   // 3/6/11 change
    
    break;  // case 1
    
    default:
    break;
    
  } // end switch( pMuxID )
  
  return sMuxOutput;
  
} // end ReadMux
	            
// ***********************************************************************

short ReadTapeSensor( enum TapeSensorID p_tsID, enum TapeSensorRetMode p_Mode )
{
  // ANALOG INPUT
  // Read analog input off the appropriate mux pin
  static short sBinaryTapeEvaluation = 0; // on/off
  short sTapeSensorReading = 0;
  
  switch( p_tsID )
  {
    case ts_FRONT_LEFT:
      sTapeSensorReading = ReadMux( TAPE_SENSOR_MUX, TAPE_FRONT_LEFT_MUX_LINE );

      if( sTapeSensorReading > TapeThresh.FrontLeft  - TAPE_MDF_DELTA  )
      {
    	  sBinaryTapeEvaluation = 0;  // MDF, field, ok
      }
      else if( sTapeSensorReading < TapeThresh.FrontLeft - TAPE_TAPE_DELTA )
      {
    	  sBinaryTapeEvaluation = 1;  // TAPE, border, raise flag
      }
    break;    

    case ts_FRONT_CENTER:
      sTapeSensorReading = ReadMux( TAPE_SENSOR_MUX, TAPE_FRONT_CENTER_MUX_LINE );

      if( sTapeSensorReading > TapeThresh.FrontCenter  - TAPE_MDF_DELTA  )
      {
    	  sBinaryTapeEvaluation = 0;  // MDF, field, ok
      }
      else if( sTapeSensorReading < TapeThresh.FrontCenter - TAPE_TAPE_DELTA )
      {
    	  sBinaryTapeEvaluation = 1;  // TAPE, border, raise flag
      }      
    break;    
      
    case ts_FRONT_RIGHT:
      sTapeSensorReading = ReadMux( TAPE_SENSOR_MUX, TAPE_FRONT_RIGHT_MUX_LINE );

      if( sTapeSensorReading > TapeThresh.FrontRight  - TAPE_MDF_DELTA  )
      {
    	  sBinaryTapeEvaluation = 0;  // MDF, field, ok
      }
      else if( sTapeSensorReading < TapeThresh.FrontRight - TAPE_TAPE_DELTA )
      {
    	  sBinaryTapeEvaluation = 1;  // TAPE, border, raise flag
      }            
    break;    
      
    case ts_REAR_LEFT:
      sTapeSensorReading = ReadMux( TAPE_SENSOR_MUX, TAPE_REAR_LEFT_MUX_LINE );
      
      if( sTapeSensorReading > TapeThresh.RearLeft  - TAPE_MDF_DELTA  )
      {
    	  sBinaryTapeEvaluation = 0;  // MDF, field, ok
      }
      else if( sTapeSensorReading < TapeThresh.RearLeft - TAPE_TAPE_DELTA )
      {
    	  sBinaryTapeEvaluation = 1;  // TAPE, border, raise flag
      }                  
    break;    
      
    case ts_REAR_CENTER:
      sTapeSensorReading = ReadMux( TAPE_SENSOR_MUX, TAPE_REAR_CENTER_MUX_LINE );
      
      if( sTapeSensorReading > TapeThresh.RearCenter  - TAPE_MDF_DELTA  )
      {
    	  sBinaryTapeEvaluation = 0;  // MDF, field, ok
      }
      else if( sTapeSensorReading < TapeThresh.RearCenter - TAPE_TAPE_DELTA )
      {
    	  sBinaryTapeEvaluation = 1;  // TAPE, border, raise flag
      }                  
    break;    
      
    case ts_REAR_RIGHT:
      sTapeSensorReading = ReadMux( TAPE_SENSOR_MUX, TAPE_REAR_RIGHT_MUX_LINE );
      
      if( sTapeSensorReading > TapeThresh.RearRight  - TAPE_MDF_DELTA  )
      {
    	  sBinaryTapeEvaluation = 0;  // MDF, field, ok
      }
      else if( sTapeSensorReading < TapeThresh.RearRight - TAPE_TAPE_DELTA )
      {
    	  sBinaryTapeEvaluation = 1;  // TAPE, border, raise flag
      }                  
    break;                                
        
    default:
    break;        
  }
    
  if( p_Mode == tsrBINARY )
  {
    // High-level return - Bool 1/0 for tape on/off
     return sBinaryTapeEvaluation;     
  }
  else
  {
      // Low-level return - actual tape sensor reading
     return sTapeSensorReading;    
  }
} // end ReadTapeSensor

// ***********************************************************************

void CalibrateTapeSensors( void )
{
   int AccumFC = 0, AccumFL = 0, AccumFR = 0, 
       AccumRC = 0, AccumRL = 0, AccumRR = 0, idx;
       
   for( idx = 0; idx < NUM_TAPE_SAMPLES; ++idx )
   {
      AccumFC += ReadTapeSensor( ts_FRONT_CENTER, tsrRAW ); 
   }
   
   for( idx = 0; idx < NUM_TAPE_SAMPLES; ++idx )
   {
      AccumFL += ReadTapeSensor( ts_FRONT_LEFT, tsrRAW ); 
   }
   
   for( idx = 0; idx < NUM_TAPE_SAMPLES; ++idx )
   {
      AccumFR += ReadTapeSensor( ts_FRONT_RIGHT, tsrRAW ); 
   }
   
   for( idx = 0; idx < NUM_TAPE_SAMPLES; ++idx )
   {
      AccumRC += ReadTapeSensor( ts_REAR_CENTER, tsrRAW ); 
   }
   
   for( idx = 0; idx < NUM_TAPE_SAMPLES; ++idx )
   {
      AccumRL += ReadTapeSensor( ts_REAR_LEFT, tsrRAW ); 
   }
   
   for( idx = 0; idx < NUM_TAPE_SAMPLES; ++idx )
   {
      AccumRR += ReadTapeSensor( ts_REAR_RIGHT, tsrRAW ); 
   }
   
   AccumFC /= NUM_TAPE_SAMPLES;
   AccumFL /= NUM_TAPE_SAMPLES;
   AccumFR /= NUM_TAPE_SAMPLES;
   AccumRC /= NUM_TAPE_SAMPLES;
   AccumRL /= NUM_TAPE_SAMPLES;
   AccumRR /= NUM_TAPE_SAMPLES;
   
   TapeThresh.FrontCenter = AccumFC;
   TapeThresh.FrontLeft   = AccumFL;
   TapeThresh.FrontRight  = AccumFR;
   TapeThresh.RearCenter  = AccumRC;
   TapeThresh.RearLeft    = AccumRL;
   TapeThresh.RearRight   = AccumRR;   
 
}

// ***********************************************************************

int GetThreshFC( void )
{
   return TapeThresh.FrontCenter;  
}

int GetThreshFL( void )
{
   return TapeThresh.FrontLeft;  
}

int GetThreshFR( void )
{
   return TapeThresh.FrontRight;    
}

int GetThreshRC( void )
{
   return TapeThresh.RearRight;      
}

int GetThreshRL( void )
{
   return TapeThresh.RearLeft;      
}

int GetThreshRR( void )
{
   return TapeThresh.RearRight;      
}

// ***********************************************************************

short ReadBeaconSensor( enum BeaconSensorID p_bsID )
{
  // ANALOG INPUT
  // Read analog input off the appropriate op amp pin  
  short sBeaconSensorReading = 0;
  
  switch( p_bsID )
  {
    case bs_FRONT_LEFT:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_LEFT_MUX_LINE  );
      //static short        ReadMux( int p_MuxID, int p_MuxInputLine ) 
    break;    

    case bs_FRONT_CENTER:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_CENTER_MUX_LINE );
    break;    
      
    case bs_FRONT_RIGHT:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_RIGHT_MUX_LINE );
    break;    
      
    case bs_MID_LEFT:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_MID_LEFT_MUX_LINE );
    break;    
      
    case bs_MID_RIGHT:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_MID_RIGHT_MUX_LINE );
    break;    
      
    case bs_REAR_LEFT:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_LEFT_MUX_LINE );
    break;     

    case bs_REAR_CENTER:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_CENTER_MUX_LINE );
    break;    
      
    case bs_REAR_RIGHT:
      sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_RIGHT_MUX_LINE );
    break;   
    
    default:
    break;    
  }
  
  // Dummy return statement
  // Real one needs to return port-captured data
  return sBeaconSensorReading;  
} // end ReadBeaconSensor

// ***********************************************************************

enum BeaconSensorID GetMaxBeaconReturnID( void )
{
   // Init with a very small value so any return is larger
  short MaxVal = 0;
  enum BeaconSensorID MaxID = 0;
  short sBeaconSensorReading = 0;  
  
  // Sample all sensors
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_LEFT_MUX_LINE  );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_FRONT_LEFT; 
  }
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_CENTER_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_FRONT_CENTER; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_RIGHT_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_FRONT_RIGHT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_MID_LEFT_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_MID_LEFT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_MID_RIGHT_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_MID_RIGHT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_LEFT_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_REAR_LEFT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_CENTER_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_REAR_CENTER; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_RIGHT_MUX_LINE );
  if( sBeaconSensorReading > MaxVal )
  {
     MaxVal = sBeaconSensorReading;
     MaxID = bs_REAR_RIGHT; 
  }    
  return MaxID;
   
}

// ***********************************************************************

enum BeaconSensorID GetMinBeaconReturnID( void )
{
  // Init with a very large value so any return is smaller
  short MinVal = 60000;  
  enum BeaconSensorID MinID = 0;
  short sBeaconSensorReading = 0;
  
  // Sample all sensors
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_LEFT_MUX_LINE  );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_FRONT_LEFT; 
  }
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_CENTER_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_FRONT_CENTER; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_FRONT_RIGHT_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_FRONT_RIGHT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_MID_LEFT_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_MID_LEFT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_MID_RIGHT_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_MID_RIGHT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_LEFT_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_REAR_LEFT; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_CENTER_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_REAR_CENTER; 
  }  
  sBeaconSensorReading = ReadMux( BEACON_SENSOR_MUX, BEACON_REAR_RIGHT_MUX_LINE );
  if( sBeaconSensorReading < MinVal )
  {
     MinVal = sBeaconSensorReading;
     MinID = bs_REAR_RIGHT; 
  }    
  return MinID;

}

// ***********************************************************************

short ReadIRRangeFinder( enum RangefinderID p_rfID )
{
  // ANALOG INPUT  
  // Read analog input off the appropriate mux pin  
  short sIRRangefinderReading = 0;  
  
  switch( p_rfID )
  {
    case rf_FRONT:
      sIRRangefinderReading = ReadMux( IR_RANGEFINDER_MUX, IR_RANGEFINDER_FRONT_MUX_LINE );    
    break;

    case rf_CANNON:
      sIRRangefinderReading = ReadMux( IR_RANGEFINDER_MUX, IR_RANGEFINDER_CANNON_MUX_LINE );    
    break;    
    
    default:
    break;    
  }

  return sIRRangefinderReading;  
} // end ReadIRRangeFinder

// ***********************************************************************

short ReadSwitch( enum SwitchID p_swID )
{
  // DIGITAL INPUT 
  short sSwitchState = 0;
  switch( p_swID )
  {
    case sw_FRONT_BUMPER:
      //        AN
	  sSwitchState =  PTIAD_PTIAD2;
    break;
    
    case sw_LEFT_BUMPER:
       //       AN
	  sSwitchState =  PTIAD_PTIAD5;
    break;
    
    case sw_RIGHT_BUMPER:
       //       AN
	  sSwitchState =  PTIAD_PTIAD4;
    break;
    
    case sw_REAR_BUMPER:
        //      AN
	  sSwitchState =  PTIAD_PTIAD3;
    break;
    
    case sw_BULL_TOR_MODE:
        //      PA
	  sSwitchState =  PTIAD_PTIAD6;
    break;
    
    case sw_GO:
        //      PA
	  sSwitchState =  PTIAD_PTIAD7;
    break;
   
    default:
    break;    
  }
  
  return sSwitchState;  
} // end ReadSwitch


// ***********************************************************************               

short GetCurrRangeFinder( void )
{
  return CurrFrontIRRange;  
}

short GetLastRangeFinder( void )
{
  return LastFrontIRRange;  
}

// ***********************************************************************               

void interrupt 14 irq_PollRangeFinderResponse( void )
{
    int idx;
    int Accumulator = 0;
    TC6 += iInfraredRangefinderCycle;

	// Will be the same (0) upon startup 
    LastFrontIRRange = CurrFrontIRRange;
    
    // Poll range finders several times
    for( idx = 0; idx < 4; ++idx )
    {
        Accumulator += (int)ReadIRRangeFinder( rf_FRONT );      
    }

    // Average out the values to filter noise between samples
    Accumulator >>= 2;
    
    // Update the current reading
    CurrFrontIRRange = Accumulator;   
	
   	TFLG1 |= TFLG1_C6F_MASK;  //clear the flag for the interrupt
}	// end irq_BallFeederServoResponse


// ***********************************************************************                
// ***********************************************************************
// end file
