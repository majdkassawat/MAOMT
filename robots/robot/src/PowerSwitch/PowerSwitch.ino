
#include <ros.h>
#include <std_msgs/Bool.h>

const int green_led_pin = 10;
const int red_led_pin = 12;
const int set_pin = 9;
const int reset_pin = 8;

ros::NodeHandle nh;

void messageCb(const std_msgs::Bool &command_msg)
{
  if (!command_msg.data)
  {
    digitalWrite(set_pin, LOW); 
    delay(2000);                      

    digitalWrite(reset_pin, HIGH);
    delay(2000);                   
    
  }
  else
  {
    digitalWrite(set_pin, HIGH); 
    delay(2000);                 
  
    digitalWrite(reset_pin, LOW);
    delay(2000);
  }
}

ros::Subscriber<std_msgs::Bool> sub("PowerSwitch", &messageCb);

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(green_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(set_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
}

// the loop function runs over and over again forever
void loop()
{
  digitalWrite(green_led_pin, HIGH); // turn the LED on (HIGH is the voltage level)
  nh.spinOnce();
  delay(1);
}
