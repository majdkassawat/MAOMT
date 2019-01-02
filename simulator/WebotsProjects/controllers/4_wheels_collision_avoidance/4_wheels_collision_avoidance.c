/*
 * File:          4_wheels_collision_avoidance.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/touch_sensor.h>
#include <stdio.h>

#define MAX_SPEED 2
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
// entry point of the controller
#include <webots/motor.h>
int main(int argc, char **argv)
{
  // initialize the Webots API
  wb_robot_init();

  WbDeviceTag force_sensor = wb_robot_get_device("touch sensor");
  WbDeviceTag traction_motor = wb_robot_get_device("traction motor");
   WbDeviceTag sphero_motor = wb_robot_get_device("spheromotor");
   // initialize motors
   WbDeviceTag wheels[4];
   char wheels_names[4][8] = {
     "wheel1", "wheel2", "wheel3", "wheel4"
   };
   int j;
   for (j=0; j<4 ; j++)
     wheels[j] = wb_robot_get_device(wheels_names[j]);
   
   
  // internal variables
  int i;
  WbDeviceTag ps[2];
  char ps_names[2][4] = {
    "ps0", "ps1"
  };

  // initialize devices
  for (i = 0; i < 2 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

   for (j=0; j<4 ; j++){ 
   
     wb_motor_set_position(wheels[j], INFINITY);
     wb_motor_set_velocity(wheels[j], 0);
   
   }

    wb_motor_set_position(traction_motor,INFINITY);
     wb_motor_set_velocity(traction_motor, 0);
      wb_motor_set_position(sphero_motor,INFINITY);
     wb_motor_set_velocity(sphero_motor, 0);

    wb_touch_sensor_enable(force_sensor,100);


  // feedback loop: step simulation until an exit event is received
  while (wb_robot_step(TIME_STEP) != -1) {
    // read sensors outputs
    
    
    double ps_values_0 = wb_distance_sensor_get_value(ps[0]);
    double ps_values_1 = wb_distance_sensor_get_value(ps[1]);
    double force_sensor_value =  wb_touch_sensor_get_value(force_sensor);
    
    
    
    double wheel_speeds[4];

    // detect obstacles
    bool right_obstacle = ps_values_1 < 700;
    bool left_obstacle =  ps_values_0 < 700;

    // initialize motor speeds at 50% of MAX_SPEED.
    for (j=0; j<4 ; j++){
      wheel_speeds[j] = 0.5 * MAX_SPEED;
    }
    
    
    //printf("sensor value is %f\n", ps_values_0);
    //printf("sensor value is %f\n", force_sensor_value);
    if(force_sensor_value>20)
    {
      wb_motor_set_velocity(sphero_motor, 0.2);
      
      
   for (j=0; j<4 ; j++)
     wb_motor_set_velocity(wheels[j], 0);
   
      wb_motor_set_velocity(traction_motor, 0);
  }
      
      else {
      
      for (j=0; j<4 ; j++)
       wb_motor_set_velocity(wheels[j],  wheel_speeds[j]);
   
      wb_motor_set_velocity(traction_motor, 0.2);
  }
      }
      
   
    

   // modify speeds according to obstacles
    // if (left_obstacle) {
    //  turn right
      // wheel_speeds[0]  = -0.5 * MAX_SPEED;
      // wheel_speeds[3]  = -0.5 * MAX_SPEED;
      // wheel_speeds[1] = -0.5 * MAX_SPEED;
      // wheel_speeds[2] = -0.5 * MAX_SPEED;
    // }
    // else if (right_obstacle) {
    //  turn left
      // wheel_speeds[0]  -= 0.5 * MAX_SPEED;
      // wheel_speeds[3]  -= 0.5 * MAX_SPEED;
      // wheel_speeds[1] += 0.5 * MAX_SPEED;
      // wheel_speeds[2] += 0.5 * MAX_SPEED;
    // }


  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}