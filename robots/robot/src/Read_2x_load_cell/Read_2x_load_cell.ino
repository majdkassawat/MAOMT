//-------------------------------------------------------------------------------------
// HX711_ADC.h
// Arduino master library for HX711 24-Bit Analog-to-Digital Converter for Weigh Scales
// Olav Kallhovd sept2017
// Tested with      : HX711 asian module on channel A and YZC-133 3kg load cell
// Tested with MCU  : Arduino Nano
//-------------------------------------------------------------------------------------
// This is an example sketch on how to use this library for two ore more HX711 modules
// Settling time (number of samples) and data filtering can be adjusted in the config.h file

#include <HX711_ADC.h>
#include <ros.h>
#include <std_msgs/Int16.h>

//Ros Serial setup
ros::NodeHandle nh;

std_msgs::Int16 int_msg_sensor1;
ros::Publisher pub1("force_sensor_right", &int_msg_sensor1);
std_msgs::Int16 int_msg_sensor2;
ros::Publisher pub2("force_sensor_left", &int_msg_sensor2);


//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(3, 2); //HX711 1
HX711_ADC LoadCell_2(5, 4); //HX711 2

long t;

void setup() {
  
  float calValue_1; // calibration value load cell 1
  float calValue_2; // calibration value load cell 2
  
  calValue_1 = 1160; // uncomment this if you want to set this value in the sketch 
  calValue_2 = 1093; // uncomment this if you want to set this value in the sketch 

  LoadCell_1.begin();
  LoadCell_2.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilisingtime);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilisingtime);
  }
  
  LoadCell_1.setCalFactor(calValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calValue_2); // user set calibration value (float)

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub1);
  nh.advertise(pub2);
  
}

void loop() {
  //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
  //longer delay in scetch will reduce effective sample rate (be carefull with use of delay() in the loop)
  LoadCell_1.update();
  LoadCell_2.update();

  float a = -1*LoadCell_1.getData();
  float b = -1*LoadCell_2.getData();

  int_msg_sensor1.data = (int)a;
  int_msg_sensor2.data = (int)b;
  
  pub1.publish( &int_msg_sensor1 );
  pub2.publish( &int_msg_sensor2 );
  
  nh.spinOnce();

}
