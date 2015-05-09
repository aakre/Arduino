#include <Wire.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <PassiveObserver.h>
#include <Quaternion.h>
#include <MatrixMath.h>
#include <PID.h>
#include <Servo.h>
#include <Quadcopter.h>
#include <RH_ASK.h>
#include <RHDatagram.h>
#include <SPI.h>
#include <RefMod.h>

#define ESC_MAX 2000
#define ESC_MIN 1000
#define U_MAX 4
#define U_MIN -4
#define radio_speed 2000
#define rxPin 52
#define txPin 38//Random
#define enablePin 5 //Random

#define BASE_ADDR   0
#define QUAD_ADDR   1

// Timing
unsigned int now = 0;
unsigned int prev = 0;
long printCounter = 0;
int T = 10;
float dt = (float)T/1000;

// Measurement data
float acc[3];
float gyro[3];
float mag[3];

// Controller gains, limits, reference model parameters
float I_MAX = 0.7*U_MAX;
float I_MIN = -I_MAX; 
float k_p = 5.0;
float k_i = 0.0;
float k_d = 150.0;
float zeta = 1;
float omega = 1/(2*dt);

// Loop variables and constants
float tau[4];
float e_roll = 0;
float e_pitch = 0;
float e_yaw = 0;
float roll_d = 0;
float pitch_d = 0;
float yaw_d = PI/4;
float eps[3];
float altd = -20; //Altitude
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
Quaternion q_d;


// Misc.
uint8_t buflen = sizeof(buf);
int magContinous = 1;
int motorPin[4] = {5,8,12,2}; 


// Modules
HMC5883L magsens;
MPU6050  mpu;
PassiveObserver npo(dt);
PID pidRoll(k_p, k_i, k_d, I_MIN, I_MAX, U_MIN, U_MAX);
PID pidPitch(k_p, k_i, k_d, I_MIN, I_MAX, U_MIN, U_MAX);
PID pidYaw(k_p, k_i, k_d, I_MIN, I_MAX, U_MIN, U_MAX);
Quadcopter quad(U_MIN, U_MAX, ESC_MIN, ESC_MAX);
RH_ASK driver(radio_speed, rxPin, txPin, enablePin, false); // Last bool deals with signal inversion
RHDatagram radio(driver, QUAD_ADDR);
RefMod refmod_thrust(zeta, omega, dt);


int initError;
void setup() {
  Serial.begin(9600);
  if (mpu.init()) {
    initError += 1;
  }
  delay(5);
  mpu.enableSlaveConfig();
  delay(5);
  if (magsens.init(magContinous)) {
    initError += 1;
  }
  mpu.disableSlaveConfig();
  delay(5);
  mpu.addSlave(HMC5883L_ADDRESS, HMC5883L_DATAX_H);
  
  /* Run the observer N times to obtain the current attitude.
   * Use the heading measured during startup as the heading reference.
   * Due to inaccuracies in the IMU installation, the roll and pitch angles will not be zero
   * at start up. Measure these and use as bias. */
  for (int i=0; i<10; i++) {
    mpu.update(acc, gyro, mag);
    npo.update(acc, gyro, mag);
    delay(T);
  }
  float RPY[3];
  npo.getRPY((float*)RPY);
  npo.Offset(); // Stores the current attitude as a bias that will be subtracted from the measurement
  for (int i=0; i<10; i++) {
    mpu.update(acc, gyro, mag);
    npo.update(acc, gyro, mag);
    delay(T);
  }
  q_d.eta = npo.qhat.eta;
  q_d.eps1 = npo.qhat.eps1;
  q_d.eps2 = npo.qhat.eps2;
  q_d.eps3 = npo.qhat.eps3;
  
  if (!radio.init()) {
    initError += 1;
  }
  
  if (!initError) {
    delay(2000);
    quad.init((int*) motorPin);
    delay(2000);
  } else {
    Serial.println("\nError during initialization");
  }
}

int count = 0;
int print = 0;
void loop() {
  if (!initError) {
    now = millis();
    if (now-prev > T) {
      prev = now;
      readRemote();
      count++;
  
      mpu.update(acc, gyro, mag);
      npo.update(acc, gyro, mag);
      npo.getImag((float*)eps);
      
      e_roll  = q_d.eps1-eps[0];
      e_pitch = q_d.eps2-eps[1];
      e_yaw   = q_d.eps3-eps[2];
      
      tau[0] = 0;//pidRoll.update(e_roll, eps[0], 0);
      tau[1] = 0;//pidPitch.update(e_pitch, eps[1],0);
      tau[2] = 0;//pidYaw.update(e_yaw, eps[2],0);
      tau[3] = altd;
      
      print = 0;
      if (count==50) {
  //      Serial.println();
  //      for (int i=0; i<4; i++) {Serial.println(tau[i]);}
        print = 1;
        count = 0;
        npo.printRPY();
  //      Serial.println();
  //      Serial.println(roll_d);
  //      Serial.println(pitch_d);
  //      Serial.println(altd);
      }
      quad.input((float*)tau, print);
    }
  }
}


void readRemote() {
  // Execution 10-600 us
  if (radio.recvfrom(buf, &buflen)) {
      altd = (float)(int8_t)buf[0];
      altd = refmod_thrust.update(altd);
      //pitch_d = (float)(int8_t)buf[1];
      //altd = (float)(int8_t)buf[2];
      //q_d.fromEuler(roll_d, pitch_d, yaw_d);
    }
}


