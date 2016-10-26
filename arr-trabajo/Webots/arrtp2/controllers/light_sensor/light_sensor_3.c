/*
 * File:         light_sensor.c
 * Date:         August 17th, 2006
 * Description:  An example of a controller using a light sensor device.
 * Author:       Simon Blanchoud
 * Modifications:
 *
 * Copyright (c) 2006 Cyberbotics - www.cyberbotics.com
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/light_sensor.h>
#include <stdio.h>

#define SPEED 60
#define TIME_STEP 64

int main() {
  WbDeviceTag ls0, ls1, ls6, ls7;
  double ls0_value;
  double ls1_value;
	double ls6_value;
  double ls7_value;
  double left_speed, right_speed;
  
  wb_robot_init();

  /* get a handler to the distance sensors. */
  ls0 = wb_robot_get_device("ls0");
  ls1 = wb_robot_get_device("ls1");
	ls6 = wb_robot_get_device("ls6");
  ls7 = wb_robot_get_device("ls7");

  wb_light_sensor_enable(ls0, TIME_STEP);
  wb_light_sensor_enable(ls1, TIME_STEP);
	wb_light_sensor_enable(ls6, TIME_STEP);
  wb_light_sensor_enable(ls7, TIME_STEP);

  printf("You can move the light using your mouse, "
         "the robot will follow it\n");
  
  while(wb_robot_step(TIME_STEP)!=1) {
    /* read sensor values */
    ls0_value = wb_light_sensor_get_value(ls0);
    ls1_value = wb_light_sensor_get_value(ls1);
		ls6_value = wb_light_sensor_get_value(ls6);
    ls7_value = wb_light_sensor_get_value(ls7);

    printf("0: %f, 1: %f, 6: %f, 7: %f\n", ls0_value, ls1_value, ls6_value, ls7_value);

    left_speed  = (4096 - (ls0_value + ls1_value)) / 10.0f;
    right_speed = (4096 - (ls6_value + ls7_value)) / 10.0f;

    /* Set the motor speeds. */
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
  
  wb_robot_cleanup();

  return 0;
}
