// ***********************************************************************
// 
// Filename:      main.c
// Purpose:       Entry point to Bull/Toreador robot application.
// Date Created:  03/14/2011  
//				        
// Author:        Robert T. Casey
//
//
// ***********************************************************************

#include <stdio.h>
#include <stdlib.h> /* needed for rand() */
#include <hidef.h>      /* common defines and macros */
#include <mc9s12c32.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c32"

#include "AnalogInput.h"
#include "Actuators.h"
#include "Sensors.h"

void main(void) 
{
  
  EnableInterrupts;
	TERMIO_Init();
  InitSensors();
  InitActuators();

  // SYSTEM LOOP
  for(;;)
  {
       DriveStop();
       Spin( LEFT, DRIVE_SLOW );
       Spin( RIGHT, DRIVE_SLOW );
       
       
  
  
  } /* wait forever */
  /* please make sure that you never leave this function */
}
