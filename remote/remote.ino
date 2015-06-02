#include <RH_ASK.h>
#include <RHDatagram.h>
#include <SPI.h>

#define speed 2000
#define rxPin 11
#define txPin 12
#define enablePin 10

#define BASE_ADDR   0
#define QUAD_ADDR   1

#define RIGHT_SW A0
#define RIGHT_Y  A1
#define RIGHT_X  A2
#define LEFT_SW  A3
#define LEFT_Y   A4
#define LEFT_X   A5


enum Radio_state {IDLE, ACTIVE};
enum Toggle_state {PRESSED, RELEASED};


int roll;
int pitch;
int thrust;
int left_sw;
int right_sw;
Radio_state radio_state;
Toggle_state toggle_state;
uint8_t joy_input[3];

RH_ASK driver(speed, rxPin, txPin, enablePin, false);
RHDatagram radio(driver, BASE_ADDR);

void setup() {
  Serial.begin(9600);
  delay(1000);
  toggle_state = RELEASED;
  radio_state = IDLE;
  if (!radio.init()) {
    Serial.println("Radio init failed ... :/");
  } else {
    Serial.println("Radio init OK");
  }
}

void loop () {
  
  switch (radio_state) {
  
    case (IDLE):
    if(check_power_toggle()) {
      radio_state = ACTIVE;
      delay(1000);
      Serial.println("Changing state to ACTIVE");
      break;
    }
    Serial.println("State idle");
    break;
    
    case (ACTIVE):
    if (check_power_toggle()) {
      radio_state = IDLE;
      Serial.println("Changing state to IDLE");
      delay(1000);
      break;
    }
    
    roll = analogRead(RIGHT_Y);
    pitch = analogRead(RIGHT_X);
    thrust = analogRead(LEFT_X);
    
    roll = map(roll, 0, 1023, 0, 255);
    pitch = map(pitch, 0, 1023, 0, 255);
    thrust = map(thrust, 0, 1023, 0, 255);
    
    joy_input[0] = (uint8_t)roll;
    joy_input[1] = (uint8_t)pitch;
    joy_input[2] = (uint8_t)thrust;
    
    Serial.println("Readings: ");
    for (int i=0; i<3; i++) {
      Serial.println(joy_input[i]);
    }
    radio.sendto((uint8_t*)joy_input, sizeof(joy_input), QUAD_ADDR);
    radio.waitPacketSent(); // Measure the time and include a delay
    break;
  }
  
  delay(100);
}


int check_power_toggle() {
  left_sw = analogRead(LEFT_SW);
  right_sw = analogRead(RIGHT_SW);
  
  int retval = 0;
  bool condition = ((left_sw < 10) && (right_sw < 10));

  switch (toggle_state) {
  case PRESSED:
    if (!condition) {
      Serial.println("Ready to toggle!");
      toggle_state = RELEASED;
      retval = 1;
    }
    break;
  case RELEASED:
    if (condition) {
      Serial.println("Button press detected!");
      toggle_state = PRESSED;
    }
    break;
  }
  return retval;
}



