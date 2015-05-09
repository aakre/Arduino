#include <Servo.h>

#define ESC_PIN 9
#define ESC_MAX 2000
#define ESC_MIN 1000

Servo myESC;
int motor_speed;

void setup() {
    Serial.begin(9600);

    while (!Serial); // Wait for serial port (Arduino Leonardo)
    Serial.println(F("The battery should be disconnected before starting!"));
    Serial.println(F("Write anything to the termnial to start."));
    wait_for_user_input();
    myESC.attach(ESC_PIN);
    myESC.writeMicroseconds(ESC_MIN);
    delay(5);
    Serial.println(F("Connect battery, send when ready!"));
    wait_for_user_input();
}
void loop() {
    while (Serial.available() > 0) {
      motor_speed = Serial.parseInt();
      motor_speed = max(ESC_MIN, min(ESC_MAX, motor_speed));
      Serial.print(F("Now writing: "));
      Serial.print(motor_speed);
      Serial.print("\r\n");
      myESC.writeMicroseconds(motor_speed);
    }
}

void wait_for_user_input() {
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
}

