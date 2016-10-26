/*
 * File:         mybot_simple.c
 * Date:         August 8th, 2006
 * Description:  A really simple controller which moves the MyBot robot
 *               and avoids the walls
 * Author:       Simon Blanchoud
 * Modifications:
 *
 * Copyright (c) 2008 Cyberbotics - www.cyberbotics.com
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>


#define SPEED 60
#define TIME_STEP 64
#define THRESHOLD 500

#if 1
  #define PRINT_(a,b) printf (a,b)
#else
  #define PRINT_(a,b) (void)0
#endif

int main()
{
  wb_robot_init(); /* necessary to initialize webots stuff */
  
  /* Get and enable the distance sensors. */
  WbDeviceTag ir0 = wb_robot_get_device("ir0");
  WbDeviceTag ir1 = wb_robot_get_device("ir1");
  wb_distance_sensor_enable(ir0, TIME_STEP);
  wb_distance_sensor_enable(ir1, TIME_STEP);
  
  while(wb_robot_step(TIME_STEP)!=-1) {
    
    /* Get distance sensor values */
    double ir0_value = wb_distance_sensor_get_value(ir0);
    double ir1_value = wb_distance_sensor_get_value(ir1);
    PRINT_("ir0_value: %f\n", ir0_value);
    PRINT_("ir0_value: %f\n", ir1_value);

    /* Compute the motor speeds */
    double left_speed, right_speed;
    if (ir1_value > THRESHOLD) {

        /*
         * If both distance sensors are detecting something, this means that
         * we are facing a wall. In this case we need to move backwards.
         */
        if (ir0_value > THRESHOLD) {
            left_speed = -SPEED;
            right_speed = -SPEED / 2;
        } else {

            /*
             * We turn proportionnaly to the sensors value because the
             * closer we are from the wall, the more we need to turn.
             */
            left_speed = -ir1_value / 10;
            right_speed = (ir0_value / 10) + 5;
        }
    } else if (ir0_value > THRESHOLD) {
        left_speed = (ir1_value / 10) + 5;
        right_speed = -ir0_value / 10;
    } else {

        /*
         * If nothing has been detected we can move forward at maximal speed.
         */
        left_speed = SPEED;
        right_speed = SPEED;
    }

    /* Set the motor speeds. */
    PRINT_("left_speed: %f\n", left_speed);
    PRINT_("right_speed: %f\n", right_speed);
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
  
  return 0;
}
