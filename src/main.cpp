#include <Arduino.h>


// Hot to calibrate:
// Start with these values, COmpile and Upload the code :rocket:
// Read with a voltmeter the value for the IN voltage (B+, battery positive)
// Write in the A0 voltage what your board reads. It will be on the Serial Output.
//
// Warning, remember that the imput voltage of A0 must be below 1V, or 3.3V for some custom boards (Wemos, NodeMCU..)
// Adjust the input voltage with a voltage divider, in my case, I use the following values
//   GND -- R1 (100K) -- A0 (analog input) -- R2 (400K) - B+
// It will give 3.12 for 15.6v
// My alternator gives a maximum of 16v, that converted, will be less than the maximum input for the A0.
// I want to have less resolution in order to be on the safe side, as it will not affect the result a lot.

#define CALIBRATION_IN_VOLTAGE 15.25
#define CALIBRATION_A0_VOLTAGE 2.90
#define INPUT_VOLTAGE A0
#define VOLTAGE_THRESHOLD 14.70
#define LED_PIN D4
#define RELAY_PIN D1
#define RELAY_ACTIVATION_DELAY 500
#define C_SDA D7
#define C_SCL D6
#define C_VIN D5
#define C_GND D0

float calculate_battery_voltage();
void check_relay(float voltage);

unsigned int next_relay_check = 0;
bool relay_state = HIGH;
bool led_state = HIGH;

void setup() {
  Serial.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(C_GND, OUTPUT);
  pinMode(C_VIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);


  digitalWrite(C_GND, LOW);
  digitalWrite(C_VIN, HIGH);
}

void toggle_led(){
  if(relay_state){
      led_state = true;
  }else{
    led_state = !led_state;
  }

  digitalWrite(LED_PIN, led_state);
}

void loop() {
  toggle_led();
  float battery_voltage = calculate_battery_voltage();
  Serial.print("Voltage: ");
  Serial.println(battery_voltage);

  check_relay(battery_voltage);

  if(relay_state){
    Serial.println("ON");
  }else{
    Serial.println("OFF");
  }
  delay(500);
}


float calculate_battery_voltage() {
  int analog_read = analogRead(INPUT_VOLTAGE);
  float read = float(analog_read) / 1024 * 3.3;
  Serial.print("Read:");
  Serial.println(read);
  return CALIBRATION_IN_VOLTAGE/CALIBRATION_A0_VOLTAGE * read ;
}

void check_relay(float voltage){
  bool over_voltage = voltage > VOLTAGE_THRESHOLD;
  bool is_time_to_check_relay = millis() > next_relay_check;

  if(!is_time_to_check_relay){
    return;
  }

  if(over_voltage){
    Serial.println("Over");
    relay_state = LOW;
    next_relay_check = millis() + RELAY_ACTIVATION_DELAY;
  }else{
    Serial.println("Under");
    relay_state = HIGH;
  }

  digitalWrite(RELAY_PIN, relay_state);
}