
const int green_led_pin = 10;
const int red_led_pin = 12;
const int set_pin = 9;
const int reset_pin = 8;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(green_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(set_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(green_led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(green_led_pin, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second


  digitalWrite(red_led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(red_led_pin, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second


  digitalWrite(set_pin, LOW);   // turn the LED on (HIGH is the voltage level)
  //delay(20000);                       // wait for a second
  //digitalWrite(set_pin, LOW);    // turn the LED off by making the voltage LOW
  //delay(20000);                       // wait for a second

  digitalWrite(reset_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(20000);                       // wait for a second
  //digitalWrite(reset_pin, LOW);    // turn the LED off by making the voltage LOW
  //delay(20000);                       // wait for a second
  
}
