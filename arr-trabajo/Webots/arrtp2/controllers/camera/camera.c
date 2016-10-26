/*
 * File:         camera.c
 * Date:         August 9th, 2006
 * Description:  An example of use of a camera device.
 * Author:       Simon Blanchoud
 * Modifications:
 *
 * Copyright (c) 2006 Cyberbotics - www.cyberbotics.com
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include <stdio.h>

#define SPEED 40
#define TIME_STEP 64
#define RED_BLOB_VALUE 100000
#define FILENAME_SIZE 16

char filename[FILENAME_SIZE];

void take_snapshot(WbDeviceTag camera, int width, int height, char* filename) {
	wb_camera_save_image(camera,filename,100);
}

int main() {
  WbDeviceTag camera;
  int width, height;
  int pause_counter=0;
  int left_speed, right_speed;
  int i, j, centering_weight;
  int red, blue, green;
  const unsigned char *image;
  int snapshot_index = 0;

  wb_robot_init();

  /*
   * First we get a handler to the camera device and then we open and place
   * it. We also store its height and width for further use.
   */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_move_window(camera, 0, 0);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  while(wb_robot_step(TIME_STEP)!=-1) {
    /* This is used to refresh the camera. */
    image = wb_camera_get_image(camera);
    
    left_speed = 0;
    right_speed = 0;

    /* This is used to stop the robot in front of the blob a few seconds. */
    if (pause_counter == 0) {
      red = 0;
      green = 0;
      blue = 0;

      /*
       * Here we analyse the image from the camera. The goal is to detect a
       * blob (a spot of color) of a defined color in the middle of our
       * screen.
       * In order to achieve that we simply parse the image pixel after
       * pixel and we sum for each color its value. We weight this value by
       * the distance of the current pixel to the center of the image so that
       * the blob will get a higher score if it is centered.
       */
      for (i = 0; i < width; i++) {
        centering_weight = (i < (width / 2)) ? i - 10 : (width - i - 10);
        /* We read only the lower part of the image to avoid the sky. */
        for (j = (height / 2); j < height; j++) {
          red += wb_camera_image_get_red(image, width, i, j) * centering_weight;
          blue += wb_camera_image_get_blue(image, width, i, j) * centering_weight;
          green += wb_camera_image_get_green(image, width, i, j) * centering_weight;
        }
      }
      //printf("red: %d, blue: %d, green: %d\n", red, blue, green);

      /* Now we compare the final result with the threshold of a blob. */
      
      if (red >= RED_BLOB_VALUE) {
        printf("Looks like I found a red blob !\n");
        pause_counter = 20;
        sprintf(filename, "red_blob%d.jpg", snapshot_index);
        take_snapshot(camera, width, height, filename);
        snapshot_index ++;
      } else {
        left_speed = -SPEED;
        right_speed = SPEED;
      }
    } else {
      pause_counter--;
      /* 
       * We need to move before we start to analyse the image because if we
       * do not, we will have exactly the same image (the one which has 
       * found the blob) on our camera and we will stop again.
       */
      if (pause_counter <= 3) {
        left_speed = -SPEED;
        right_speed = SPEED;
      }
    }
    /* Set the motor speeds. */
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
