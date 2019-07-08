#include <ros.h>
#include <std_msgs/UInt32.h>

//Ros Serial setup
ros::NodeHandle nh;

//Define channels pins
#define Ch1Pin 2
#define Ch2Pin 3
#define Ch3Pin 4
#define Ch4Pin 5
#define Ch5Pin 6
#define Ch6Pin 7
std_msgs::UInt32 ulong_msg_ch1;
ros::Publisher pub1("remote_transmitter/ch1", &ulong_msg_ch1);
std_msgs::UInt32 ulong_msg_ch2;
ros::Publisher pub2("remote_transmitter/ch2", &ulong_msg_ch2);
std_msgs::UInt32 ulong_msg_ch3;
ros::Publisher pub3("remote_transmitter/ch3", &ulong_msg_ch3);
std_msgs::UInt32 ulong_msg_ch4;
ros::Publisher pub4("remote_transmitter/ch4", &ulong_msg_ch4);
std_msgs::UInt32 ulong_msg_ch5;
ros::Publisher pub5("remote_transmitter/ch5", &ulong_msg_ch5);
std_msgs::UInt32 ulong_msg_ch6;
ros::Publisher pub6("remote_transmitter/ch6", &ulong_msg_ch6);
void setup()
{
  //  Serial.begin(57600);
  // while(!Serial); // If this line is activated, you need to open Serial Terminal.
  pinMode(Ch1Pin, INPUT);
  pinMode(Ch2Pin, INPUT);
  pinMode(Ch3Pin, INPUT);
  pinMode(Ch4Pin, INPUT);
  pinMode(Ch5Pin, INPUT);
  pinMode(Ch6Pin, INPUT);




  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);
  nh.advertise(pub5);
  nh.advertise(pub6);

}

void loop()
{
  
  unsigned long channel1 = pulseIn(Ch1Pin, HIGH);
  unsigned long channel2 = pulseIn(Ch2Pin, HIGH);
  unsigned long channel3 = pulseIn(Ch3Pin, HIGH);
  unsigned long channel4 = pulseIn(Ch4Pin, HIGH);
  unsigned long channel5 = pulseIn(Ch5Pin, HIGH);
  unsigned long channel6 = pulseIn(Ch6Pin, HIGH);
  
  ulong_msg_ch1.data = channel1;
  ulong_msg_ch2.data = channel2;
  ulong_msg_ch3.data = channel3;
  ulong_msg_ch4.data = channel4;
  ulong_msg_ch5.data = channel5;
  ulong_msg_ch6.data = channel6;


  pub1.publish( &ulong_msg_ch1 );
  pub2.publish( &ulong_msg_ch2 );
  pub3.publish( &ulong_msg_ch3 );
  pub4.publish( &ulong_msg_ch4 );
  pub5.publish( &ulong_msg_ch5 );
  pub6.publish( &ulong_msg_ch6 );
  
  nh.spinOnce();
  //delay(1);

}
