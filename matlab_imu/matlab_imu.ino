#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>
int continous = 1;
HMC5883L magsens;
MPU6050  mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu.init();
  mpu.enableSlaveConfig();
  magsens.init(continous);
  mpu.disableSlaveConfig();
  mpu.addSlave(HMC5883L_ADDRESS, HMC5883L_DATAX_H);
  delay(500);
}

float acc[3];
float gyro[3];
float mag[3];

int dt = 10; //10ms = 100 hz

void loop() {
  // put your main code here, to run repeatedly:

  mpu.update(acc, gyro, mag);
  Serial.print("\n");
  for (int i=0; i<3; i++) {
    Serial.print(acc[i]);
    Serial.print(",");
  }
  for (int i=0; i<3; i++) {
    Serial.print(gyro[i]);
    Serial.print(",");
  }
  Serial.print(mag[0]);
  Serial.print(",");
  Serial.print(mag[1]);
  Serial.print(",");
  Serial.print(mag[2]);
  delay(dt);
}

float calc_yaw(float *mag) {
  //http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
  float mx = (float)mag[0];
  float my = (float)mag[1];
  float ratio = mx/my;
  float yaw = -1;
  if (my>0) {
    yaw = 90.0-atan(ratio)*57.3;
  } else if (my<0) {
    //Serial.println(90-atan(ratio)*57.3);
    yaw = 270.0 - atan(ratio)*57.3;
  } else if ((my==0) && (mx<0)) {
    yaw = 180.0;
  } else {
    yaw = 0.0;
  }
  return yaw;
}

float calc_roll(float *acc) {
  return atan(acc[1]/acc[2]);
}

float calc_pitch(float *acc) {
  return atan(acc[0]/(sqrt(acc[1]*acc[1]+acc[2]*acc[2])));
}

