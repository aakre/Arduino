#include <Servo.h>

#define ESC_PIN 9
#define LED_PIN 13
#define ESC_MAX 2000
#define ESC_MIN 1000

Servo myESC;

void setup() {
    Serial.begin(9600);

    while (!Serial); // Wait for serial port (Arduino Leonardo)

}
void loop() {
    Serial.println(F("The battery should be disconnected before starting!"));
    Serial.println(F("*** Write anything to the serial port to continue... ***\n"));

    wait_for_user_input();

    Serial.println(F("Now writing maximum PWM value.\n"));
    myESC.attach(ESC_PIN);
    myESC.writeMicroseconds(ESC_MAX);
    delay(5);
    Serial.println(F("*** Connect battery ***\n"));

    Serial.println(F("A beep-beep value should be emmited, confirming maximum throttle."));
    Serial.println(F("*** Write anything to the serial port when throttle is confirmed ***\n"));

    wait_for_user_input();

    myESC.writeMicroseconds(ESC_MIN);
    Serial.println(F("Now writing minimum PWM value."));
    Serial.println(F("Antother beep-beep value confirms no. cells."));
    Serial.println(F("Lastly, a long beep indicates confirmed low position.\n"));

    Serial.println(F("*** Press any key to repeat procedure ***\n\n"));

    wait_for_user_input();
}

void wait_for_user_input() {
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
}
