#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include <webots/led.h>

#define TIME_STEP 16

#define RED 0
#define GREEN 1
#define BLUE 2

#define LEFT 0
#define RIGHT 1

#define NUM_LEDS 4

WbDeviceTag leds[NUM_LEDS];

WbDeviceTag camera;

int getMaxColor() {
	int i, j;
	const unsigned char *image;
	int red=0, blue=0, green=0;
	int width, height;

	/* This is used to refresh the camera. */
	image = wb_camera_get_image(camera);
	wb_robot_step(TIME_STEP);
	width = wb_camera_get_width(camera);
	height = wb_camera_get_height(camera);
	
	for (i = 0; i < width; i++) {
		/* We read only the lower part of the image to avoid the sky. */
		for (j = 0; j < height; j++) {
			red += wb_camera_image_get_red(image, width, i, j);
			blue += wb_camera_image_get_blue(image, width, i, j);
			green += wb_camera_image_get_green(image, width, i, j);
		}
	}
	
	//printf("Red: %d \t Green: %d \t Blue: %d \n", red, green, blue);
	
	if(red > green) {
		if(red > blue) {
			return RED;
		}
		return BLUE;
	}
	if(green > blue) {
		return GREEN;
	}
	return BLUE;
}


int main(int argc, char **argv) {
	/* define variables */

	/* necessary to initialize webots stuff */

	int i;

	wb_robot_init();


	leds[0] = wb_robot_get_device("led1");
	leds[1] = wb_robot_get_device("led3");
	leds[2] = wb_robot_get_device("led5");
	leds[3] = wb_robot_get_device("led7");	


	do {
		int actualColor = getMaxColor();
	
		for(i = 0; i < NUM_LEDS; i++) {
			wb_led_set(leds[i], 0);
		}
		
		wb_led_set(leds[actualColor], 1);
		wb_robot_step(TIME_STEP);
		
	} while (1);
	
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}
