#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

void setup(){
  Serial.begin(9600);
  mpu.init();
};
float acc[3];
float gyro[3];
void loop(){
  mpu.update(acc, gyro);
  Serial.println();
  for (int i=0; i<3; i++) {
    Serial.print(acc[i]);
    Serial.print("\t");
    Serial.print(gyro[i]);
    Serial.print("\n");
  }
  delay(1000);
};


