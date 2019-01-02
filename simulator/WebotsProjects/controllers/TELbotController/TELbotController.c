/*
 * File:          TELbotController.c
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
#include <webots/motor.h>
#include <stdio.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 16
#define MAX_SPEED 2
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  WbDeviceTag wheel_motor1 = wb_robot_get_device("WheelMotor1");
  WbDeviceTag wheel_motor2 = wb_robot_get_device("WheelMotor2");
  WbDeviceTag wheel_motor3 = wb_robot_get_device("WheelMotor3");
  WbDeviceTag traction_motor1 = wb_robot_get_device("TractionMotor1");
  WbDeviceTag traction_motor2 = wb_robot_get_device("TractionMotor2");
  
  
  
  
   wb_motor_set_position(wheel_motor1,INFINITY);
   wb_motor_set_position(wheel_motor2,INFINITY);
   wb_motor_set_position(wheel_motor3,INFINITY);
   wb_motor_set_position(traction_motor1,INFINITY);
   wb_motor_set_position(traction_motor2,INFINITY);
   wb_motor_set_velocity(wheel_motor1, 0);
   wb_motor_set_velocity(wheel_motor2, 0);
   wb_motor_set_velocity(wheel_motor3, 0);
   wb_motor_set_velocity(traction_motor1, 0);
   wb_motor_set_velocity(traction_motor2, 0);

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

  wb_motor_set_velocity(wheel_motor1, -0.5);
  wb_motor_set_velocity(wheel_motor2, 0);
  wb_motor_set_velocity(wheel_motor3, 0.5);
  wb_motor_set_velocity(traction_motor1, -0.5);
  wb_motor_set_velocity(traction_motor2, -0.5);
  
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
