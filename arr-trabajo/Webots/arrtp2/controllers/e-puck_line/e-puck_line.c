/***************************************************************************
  
  e-puck_line -- Base code for a practical assignment on behaviour-based
  robotics. When completed, the behaviour-based controller should allow 
  the e-puck robot to follow the black line, avoid obstacles and
  recover its path afterwards.
  Copyright (C) 2006 Laboratory of Intelligent Systems (LIS), EPFL
  Authors: Jean-Christophe Zufferey
  Email: jean-christophe.zufferey@epfl.ch
  Web: http://lis.epfl.ch

  This program is free software; any publications presenting results
  obtained with this program must mention it and its origin. You 
  can redistribute it and/or modify it under the terms of the GNU 
  General Public License as published by the Free Software 
  Foundation; either version 2 of the License, or (at your option) 
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
  USA.

***************************************************************************/

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/led.h>
#include <stdio.h>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0            // for wb_robot_get_mode() function
#define REALITY 2               // for wb_robot_get_mode() function
#define TIME_STEP  32           // [ms]

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7
WbDeviceTag ps[NB_DIST_SENS];	/* proximity sensors */
int ps_value[NB_DIST_SENS]={0,0,0,0,0,0,0,0};
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300,300,300,300,300,300,300,300};
// *** TO BE ADAPTED TO YOUR ROBOT *** 
const int PS_OFFSET_REALITY[NB_DIST_SENS] = {480,170,320,500,600,680,210,640}; 

// 3 IR floor color sensors
#define NB_FLOOR_SENS 3
#define FS_WHITE 900
#define FS_LEFT 0
#define FS_CENTER 1
#define FS_RIGHT 2
WbDeviceTag fs[NB_FLOOR_SENS]; /* floor sensors */
unsigned short fs_value[NB_FLOOR_SENS]={0,0,0};

// LEDs
#define NB_LEDS    8
WbDeviceTag led[NB_LEDS];

// LEM defines:
#define LEM_FORWARD_SPEED 100
#define LEM_K_FS_SPEED 0.8
#define LEM_THRESHOLD 500

#define LEM_STATE_STANDBY 0
#define LEM_STATE_LOOKING_FOR_LINE 1
#define LEM_STATE_LINE_DETECTED 2
#define LEM_STATE_ON_LINE 3

int lem_active;
int lem_reset;
int lem_speed[2];
int lem_state, lem_black_counter;
int cur_op_fs_value, prev_op_fs_value;
//------------------------------------------------------------------------------

void LineEnteringModule(int side);
void LineFollowingModule(void);

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// LFM - Line Following Module
//
// This module implements a very simple, Braitenberg-like behavior in order
// to follow a black line on the ground. Output speeds are stored in
// lfm_speed[LEFT] and lfm_speed[RIGHT].

int lfm_speed[2];

#define LFM_FORWARD_SPEED 200
#define LFM_K_FS_SPEED 0.4
#define LFM_THRESHOLD 800

void LineFollowingModule(void)
{
  int DeltaS=0;
  
  DeltaS = fs_value[FS_RIGHT]-fs_value[FS_LEFT];
  
  if(fs_value[FS_RIGHT] <= LFM_THRESHOLD && fs_value[FS_LEFT] <= LFM_THRESHOLD) {
		lem_state = LEM_STATE_LOOKING_FOR_LINE;
  	LineEnteringModule(NO_SIDE);
  } else {
		lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_FS_SPEED * DeltaS;
		lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_FS_SPEED * DeltaS;
  }
}


////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
//
// The OAM routine first detects obstacles in front of the robot, then records
// their side in "oam_side" and avoid the detected obstacle by 
// turning away according to very simple weighted connections between
// proximity sensors and motors. "oam_active" becomes active when as soon as 
// an object is detected and "oam_reset" inactivates the module and set 
// "oam_side" to NO_SIDE. Output speeds are in oam_speed[LEFT] and oam_speed[RIGHT].

int oam_active, oam_reset;
int oam_speed[2];
int oam_side = NO_SIDE;

#define OAM_OBST_THRESHOLD 100
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.2
#define OAM_K_PS_45 0.9
#define OAM_K_PS_00 1.2
#define OAM_K_MAX_DELTAS 600

void ObstacleAvoidanceModule(void)
{
  int max_ds_value, DeltaS=0, i;
  int Activation[]={0,0};

  // Module RESET
  if (oam_reset)
  {
    oam_active = FALSE;
    oam_side = NO_SIDE;
  }
  oam_reset = 0;

  // Determine the presence and the side of an obstacle
  max_ds_value = 0;
  for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) {
    if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
    Activation[RIGHT] += ps_value[i];
  }
  for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) {
    if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
    Activation[LEFT] += ps_value[i];
  }
  if (max_ds_value > OAM_OBST_THRESHOLD) oam_active = TRUE;
  
  if (oam_active && oam_side == NO_SIDE) // check for side of obstacle only when not already detected
  {
    if (Activation[RIGHT] > Activation[LEFT]) oam_side = RIGHT;
    else oam_side = LEFT;
  }

  // Forward speed
  oam_speed[LEFT]  = OAM_FORWARD_SPEED;
  oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  // Go away from obstacle
  if (oam_active)
  {
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT)
    {
      DeltaS -= (int) (OAM_K_PS_90 * ps_value[PS_LEFT_90]); //(((ps_value[PS_LEFT_90]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_90]-PS_OFFSET)));
      DeltaS -= (int) (OAM_K_PS_45 * ps_value[PS_LEFT_45]); //(((ps_value[PS_LEFT_45]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_45]-PS_OFFSET)));
      DeltaS -= (int) (OAM_K_PS_00 * ps_value[PS_LEFT_00]); //(((ps_value[PS_LEFT_00]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_00]-PS_OFFSET)));
    }
    else // oam_side == RIGHT
    {
      DeltaS += (int) (OAM_K_PS_90 * ps_value[PS_RIGHT_90]);  //(((ps_value[PS_RIGHT_90]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_90]-PS_OFFSET)));
      DeltaS += (int) (OAM_K_PS_45 * ps_value[PS_RIGHT_45]);  //(((ps_value[PS_RIGHT_45]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_45]-PS_OFFSET)));
      DeltaS += (int) (OAM_K_PS_00 * ps_value[PS_RIGHT_00]);  //(((ps_value[PS_RIGHT_00]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_00]-PS_OFFSET)));
    }
    if (DeltaS > OAM_K_MAX_DELTAS) DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS) DeltaS = -OAM_K_MAX_DELTAS;

    // Set speeds
    oam_speed[LEFT] -= DeltaS;
    oam_speed[RIGHT] += DeltaS;
  }
}

////////////////////////////////////////////
// LLM - Line Leaving Module
//
// Since it has no output, this routine is not completely finished. It has
// been designed to monitor the moment while the robot is leaving the
// track and signal to other modules some related events. It becomes active
// whenever the "side" variable displays a rising edge (changing from -1 to 0 or 1).

int llm_active=FALSE, llm_inibit_ofm_speed, llm_past_side=NO_SIDE;

#define LLM_THRESHOLD 800

void LineLeavingModule(int side)
{
  // Starting the module on a rising edge of "side"
  if (!llm_active && side!=NO_SIDE && llm_past_side==NO_SIDE)
    llm_active = TRUE;

  // Updating the memory of the "side" state at the previous call
  llm_past_side = side;

  // Main loop
  if (llm_active) // Simply waiting until the line is not detected anymore
  {
    if (side == LEFT)
    {
      if ((fs_value[FS_CENTER]+fs_value[FS_LEFT])/2 > LLM_THRESHOLD)  // out of line
      {
        llm_active = FALSE;
        // *** PUT YOUR CODE HERE ***     
      }
      else  // still leaving the line
      {
        // *** PUT YOUR CODE HERE ***
      }
    }
    else // side == RIGHT
    {
      if ((fs_value[FS_CENTER]+fs_value[FS_RIGHT])/2 > LLM_THRESHOLD)  // out of line
      {
        llm_active = FALSE;
        // *** PUT YOUR CODE HERE ***
      }
      else // still leaving the line
      {
        // *** PUT YOUR CODE HERE ***
      }
    }
  }
}


////////////////////////////////////////////
// OFM - Obstacle Following Module
//
// This function just gives the robot a tendency to steer toward the side
// indicated by its argument "side". When used in competition with OAM it
// gives rise to an object following behavior. The output speeds are
// stored in ofm_speed[LEFT] and ofm_speed[RIGHT].

int ofm_active;
int ofm_speed[2];

#define OFM_DELTA_SPEED 150

void ObstacleFollowingModule(int side)
{
  if (side != NO_SIDE)
  {
    ofm_active = TRUE;
    if (side == LEFT)
    {
      ofm_speed[LEFT]  = -OFM_DELTA_SPEED;
      ofm_speed[RIGHT] = OFM_DELTA_SPEED;
    }
    else
    {
      ofm_speed[LEFT]  = OFM_DELTA_SPEED;
      ofm_speed[RIGHT] = -OFM_DELTA_SPEED;
    }
  }
  else // side = NO_SIDE
  {
    ofm_active = FALSE;
    ofm_speed[LEFT]  = 0;
    ofm_speed[RIGHT] = 0;
  }
}


////////////////////////////////////////////
// LEM - Line Entering Module
//
// This is the most complex module (and you might find easier to re-program it
// by yourself instead of trying to understand it ;-). Its purpose is to handle
// the moment when the robot must re-enter the track (after having by-passed
// an obstacle, e.g.). It is organized like a state machine, which state is
// stored in "lem_state" (see LEM_STATE_STANDBY and following #defines).
// The inputs are (i) the two lateral floor sensors, (ii) the argument "side"
// which determines the direction that the robot has to follow when detecting
// a black line, and (iii) the variable "lem_reset" that resets the state to
// standby. The output speeds are stored in lem_speed[LEFT] and
// lem_speed[RIGHT].



void LineEnteringModule(int side)
{
  int Side, OpSide, FS_Side, FS_OpSide;

  // Module reset
  if (lem_reset)
  {
  //printf("lem_reset: %d\n", lem_reset);
    lem_state = LEM_STATE_LOOKING_FOR_LINE;
  }
  lem_reset = FALSE;

  // Initialization
  lem_speed[LEFT]  = LEM_FORWARD_SPEED;
  lem_speed[RIGHT] = LEM_FORWARD_SPEED;
  if (side==LEFT)  // if obstacle on left side -> enter line rightward
  {
    Side = RIGHT;  // line entering direction
    OpSide = LEFT;
    FS_Side = FS_RIGHT;
    FS_OpSide = FS_LEFT;
  }
  else            // if obstacle on left side -> enter line leftward
  {
    Side = LEFT;  // line entering direction
    OpSide = RIGHT;
    FS_Side = FS_LEFT;
    FS_OpSide = FS_RIGHT;
  }
//printf("lem_state: %d\n", lem_state);
  // Main loop (state machine)
  switch (lem_state)
  {
    case LEM_STATE_STANDBY:
      lem_active = FALSE;
      break;
    case LEM_STATE_LOOKING_FOR_LINE:
      if (fs_value[FS_Side]<LEM_THRESHOLD)
      {
        lem_active = TRUE;
        // set speeds for entering line
        lem_speed[OpSide] = LEM_FORWARD_SPEED;
        lem_speed[Side] = LEM_FORWARD_SPEED; // - LEM_K_FS_SPEED * fs_value[FS_Side];
        lem_state = LEM_STATE_LINE_DETECTED;
        // save floor sensor value
        if (fs_value[FS_OpSide]<LEM_THRESHOLD)
        {
          cur_op_fs_value = BLACK;
          lem_black_counter = 1;
        }
        else
        {
          cur_op_fs_value = WHITE;
          lem_black_counter = 0;
        }
        prev_op_fs_value = cur_op_fs_value;
      }
      break;
    case LEM_STATE_LINE_DETECTED:
      // save the oposite floor sensor value
      if (fs_value[FS_OpSide]<LEM_THRESHOLD)
      {
        cur_op_fs_value = BLACK;
        lem_black_counter ++;
      }
      else cur_op_fs_value = WHITE;
      // detect the falling edge BLACK->WHITE
      if (prev_op_fs_value==BLACK && cur_op_fs_value==WHITE)
      {
        lem_state = LEM_STATE_ON_LINE;
        lem_speed[OpSide] = 0;
        lem_speed[Side]   = 0;
      }
      else
      {
        prev_op_fs_value = cur_op_fs_value;
        // set speeds for entering line
        lem_speed[OpSide] = LEM_FORWARD_SPEED + LEM_K_FS_SPEED * (FS_WHITE-fs_value[FS_Side]);
        lem_speed[Side] = LEM_FORWARD_SPEED - LEM_K_FS_SPEED * (FS_WHITE-fs_value[FS_Side]);
      }
      break;
    case LEM_STATE_ON_LINE:
      oam_reset = TRUE;
      lem_active = FALSE;
      lem_state = LEM_STATE_STANDBY;
      break;
  }
}

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
int main()
{
  int i, speed[2], ps_offset[NB_DIST_SENS], Mode=1;

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name); /* proximity sensors */
    wb_distance_sensor_enable(ps[i],TIME_STEP);
  }
  for (i = 0; i < NB_FLOOR_SENS; i++) {
    sprintf(name, "fs%d", i);
    fs[i] = wb_robot_get_device(name); /* floor sensors */
    wb_distance_sensor_enable(fs[i],TIME_STEP);
  }

  for(;;)   // Main loop
  {
  
    // Reset all BB variables when switching from simulation to real robot and back
    if (Mode!=wb_robot_get_mode())
    {
      oam_reset = TRUE;
      llm_active = FALSE;
      llm_past_side = NO_SIDE;
      ofm_active = FALSE;
      lem_active = FALSE;
      lem_state = LEM_STATE_STANDBY;
      Mode = wb_robot_get_mode();
      if (Mode == SIMULATION) {
        for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_SIMULATION[i];
        wb_differential_wheels_set_speed(0,0); wb_robot_step(TIME_STEP); // Just run one step to make sure we get correct sensor values
        printf("\n\n\nSwitching to SIMULATION and reseting all BB variables.\n\n");
      } else if (Mode == REALITY) { 
        for(i=0;i<NB_DIST_SENS;i++) ps_offset[i]=PS_OFFSET_REALITY[i];
        wb_differential_wheels_set_speed(0,0); wb_robot_step(TIME_STEP); // Just run one step to make sure we get correct sensor values
        printf("\n\n\nSwitching to REALITY and reseting all BB variables.\n\n");
      }
    }
    
    // read sensors value
    for(i=0;i<NB_DIST_SENS;i++) ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i])-ps_offset[i])<0)?0:((int)wb_distance_sensor_get_value(ps[i])-ps_offset[i]);
    for(i=0;i<NB_FLOOR_SENS;i++) fs_value[i] = wb_distance_sensor_get_value(fs[i]);

    // Reset all BB variables when switching from simulation to real robot and back
    if (Mode!=wb_robot_get_mode())
    {
      oam_reset = TRUE;
      llm_active = FALSE;
      llm_past_side = NO_SIDE;
      ofm_active = FALSE;
      lem_active = FALSE;
      lem_state = LEM_STATE_STANDBY;
      Mode = wb_robot_get_mode();
      if (Mode == SIMULATION) printf("\nSwitching to SIMULATION and reseting all BB variables.\n\n");
      else if (Mode == REALITY) printf("\nSwitching to REALITY and reseting all BB variables.\n\n");
    }

    // Speed initialization
    speed[LEFT] = 0;
    speed[RIGHT] = 0;

// *** START OF SUBSUMPTION ARCHITECTURE ***

    // LFM - Line Following Module
    LineFollowingModule();
    
    // Speed computation
    speed[LEFT]  = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];
 

// *** END OF SUBSUMPTION ARCHITECTURE ***

    // Debug display
    printf("OAM %d side %d   \n", oam_active, oam_side);

    // Set wheel speeds
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);

    // Run one simulation step
    wb_robot_step(TIME_STEP);
  }
  return 0;
}
