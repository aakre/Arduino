#include <RH_ASK.h>
#include <RHDatagram.h>
#include <SPI.h>

#define speed 2000
#define rxPin 11
#define txPin 12
#define enablePin 10

#define BASE_ADDR   0
#define QUAD_ADDR   1

#define JoyX A2
#define JoyY A1
#define JoyS A0

#define X_MIN -20
#define X_MAX 10
#define Y_MIN -30
#define Y_MAX 30

int x;
int y;
uint8_t joy_input[3];

RH_ASK driver(speed, rxPin, txPin, enablePin, false);
RHDatagram radio(driver, BASE_ADDR);

void setup() {
  Serial.begin(9600);
  delay(1000);
  if (!radio.init()) {
    Serial.println("Radio init failed ... :/");
  } else {
    Serial.println("Radio init OK");
  }
}

void loop () {
  x = analogRead(JoyX);
  x = map(x, 0, 1023, X_MIN, X_MAX); // +/-30 radians ~ 17 deg
  y = analogRead(JoyY);
  y = map(y, 0, 1023, Y_MIN, Y_MAX);
  x = max(X_MIN, min(X_MAX, x));
  y = max(Y_MIN, min(Y_MAX, y));
//  Serial.println("x, y");
//  Serial.println(x);
//  Serial.println(y);
//  Serial.println();
  joy_input[0] = (uint8_t)x;
  joy_input[1] = (uint8_t)y;
  joy_input[2] = 0;
  radio.sendto((uint8_t*)joy_input, sizeof(joy_input), QUAD_ADDR);
  radio.waitPacketSent(); // Measure the time and include a delay
  delay(250);
}



