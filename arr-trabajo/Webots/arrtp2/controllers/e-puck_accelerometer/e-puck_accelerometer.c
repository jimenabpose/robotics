#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define ENCODER_STEPS 1000
#define PI 3.14
#define RANGE (1024 / 2)

double x = 0;
double z = 0;

static void compute_odometry() {
//  double l = wb_differential_wheels_get_left_encoder();
//  double r = wb_differential_wheels_get_right_encoder();
//  double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
//  double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
//  double da = (dr - dl) / AXLE_LENGTH; // delta orientation
//  printf("estimated distance covered by left wheel: %g m.\n",dl);
//  printf("estimated distance covered by right wheel: %g m.\n",dr);
//  printf("estimated change of orientation: %g rad.\n",da);

  double l = wb_differential_wheels_get_left_encoder();
  double r = wb_differential_wheels_get_right_encoder();
  double f = PI * 2 * WHEEL_RADIUS / ENCODER_STEPS;
  double dl = f * l; // distance covered by left wheel in meter
  double dr = f * r; // distance covered by right wheel in meter
  double dc = (dr + dl) / 2;
  double da = (dr - dl) / AXLE_LENGTH; // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n",dl);
  printf("estimated distance covered by right wheel: %g m.\n",dr);
  printf("estimated distance covered by centre: %g m.\n",dc);
  printf("estimated change of orientation: %g rad.\n",da);

  x = x + sin(da);
  z = z + cos(da);

  printf("x: %g \t z: %g\n", x, z);

	wb_differential_wheels_set_encoders(0, 0);
}

int main(int argc, char *argv[]) {
	int samples = 0;
	FILE * file = fopen("/Users/fechesantos/accelerometerCSV.csv", "w");

  /* define variables */
  WbDeviceTag distance_sensor[8];
  int i,j;
  double speed[2];
  double sensors_value[8];
  double braitenberg_coefficients[8][2] =
    { {150, -35}, {100, -15}, {80, -10}, {-10, -10},
    {-10, -10}, {-10, 80}, {-30, 100}, {-20, 150} };
  
  /* initialize Webots */
  wb_robot_init();

  /* get and enable devices */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,TIME_STEP*16);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer,TIME_STEP);
  wb_differential_wheels_enable_encoders(TIME_STEP);


  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);
  }

  /* main loop */
  for (;;) {

    /* get sensors values */
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
    const double *a = wb_accelerometer_get_values(accelerometer);
    printf("accelerometer values = %0.2f %0.2f %0.2f\n", a[0], a[1], a[2]);
    
    /* compute odometry and speed values*/
    compute_odometry();
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      for (j = 0; j < 8; j++) {
        speed[i] += braitenberg_coefficients[j][i] * (1.0 - (sensors_value[j] / RANGE));
      }
    }
    
		//if(samples < 1000) {
			/* v1, v2, a0, a1, a2 */
			fprintf(file, "%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n", speed[0], speed[1], a[0], a[1], a[2]);
		//}

    /* set speed values */
    wb_differential_wheels_set_speed(speed[0],speed[1]);
    
    /* perform a simulation step */
    wb_robot_step(TIME_STEP);

    samples ++;

  }

  return 0;
}
