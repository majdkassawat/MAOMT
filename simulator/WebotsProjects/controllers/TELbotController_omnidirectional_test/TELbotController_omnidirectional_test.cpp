// File:          TELbotController_omnidirectional_test.cpp
// Date:
// Description:
// Author: Majd Kassawat
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <stdio.h>
#include <limits>
#include <math.h>


#define TIME_STEP 64
#define MAX_SPEED 2
#define PI 3.14159265
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
const float r = 0.5;
const float R = 0.5;
const float alpha1 = -90;
const float alpha2 = 150;
const float alpha3 = 30;

float cs(float angle)
{
	return cos (angle * PI / 180.0);
}
float sn(float angle)
{
	return sin (angle * PI / 180.0);
}

void omnidicontroller(float Theta, float k,float W, float Vz, float Vx, float &V1, float &V2, float &V3)
{
	//V1 = k*(-0.156*Vz -0.578*Vx +0.422*W/r);
	//V2 = k*(-0.636*Vz +0.365*Vx +0.365*W/r);
	//V3 = k*(+0.788*Vz +0.211*Vx +0.211*W/r);
	V1 = k*(cs(Theta+alpha1)*Vz+sn(Theta+alpha1)*Vx+R*W)/r;
	V2 = k*(cs(Theta+alpha2)*Vz+sn(Theta+alpha2)*Vx+R*W)/r;
	V3 = k*(cs(Theta+alpha3)*Vz+sn(Theta+alpha3)*Vx+R*W)/r;
	

}
float getAngle(const double *n)
{
	return atan2(n[2],n[0]);
}



int main(int argc, char **argv)
{
  // create the Robot instance.
	Robot *robot = new Robot();

  // get the time step of the current world.
	int timeStep = (int) robot->getBasicTimeStep();


	Motor *wheel_motor1 = robot->getMotor("WheelMotor1");
	Motor *wheel_motor2 = robot->getMotor("WheelMotor2");
	Motor *wheel_motor3 = robot->getMotor("WheelMotor3");
	Motor *traction_motor1 = robot->getMotor("TractionMotor1");
	Motor *traction_motor2 = robot->getMotor("TractionMotor2");
	Compass *comp = robot->getCompass("compass");


	double inf = std::numeric_limits<double>::infinity();

	wheel_motor1->setPosition(inf);
	wheel_motor2->setPosition(inf);
	wheel_motor3->setPosition(inf);
	traction_motor1->setPosition(inf);
	traction_motor2->setPosition(inf);

	wheel_motor1->setVelocity(0);
	wheel_motor2->setVelocity(0);
	wheel_motor3->setVelocity(0);
	traction_motor1->setVelocity(0);
	traction_motor2->setVelocity(0);

	comp->enable(50);


	float V1 = 0;
	float V2 = 0;
	float V3 = 0;
	float Vz = 0;
	float Vx = 0;
	float W = 0;
	float k = 2;


  /*
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
*/
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  LED *led = robot->getLED("ledname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
	while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  led->set(1);



		Vz = 0.5/3;
		Vx = 0.866/3;
		W  = 0;
		
		double desiredAngle=0;
		const double *n;
		n=comp->getValues();
		printf("Angle : %f\n", getAngle(n));
	/*	if(getAngle(n)>desiredAngle+0.01)
			W  = 0.2;
		else if (getAngle(n)<desiredAngle-0.01)
			W  = -0.2;
		else W  = 0;
*/



		//float Theta = getAngle(n);
		float Theta = 0;
		omnidicontroller(Theta,k,W,Vz,Vx,V1,V2,V3);
		wheel_motor1->setVelocity(V1);
		wheel_motor2->setVelocity(V2);
		wheel_motor3->setVelocity(V3);
		traction_motor1->setVelocity(-0.5);
		traction_motor2->setVelocity(-0.5);


	};



  // Enter here exit cleanup code.

	delete robot;
	return 0;
}
