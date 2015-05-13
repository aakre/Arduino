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

#define U_MAX       400
#define U_MIN      -U_MAX
#define U_MAX_ROLL  U_MAX/2
#define U_MIN_ROLL -U_MAX_ROLL
#define U_MAX_PITCH U_MAX/2
#define U_MIN_PITCH -U_MAX_PITCH
#define U_MAX_YAW   U_MAX/4
#define U_MIN_YAW -U_MAX_YAW

#define ROLL       0
#define PITCH      1
#define YAW        2
#define ROLL_RATE  3
#define PITCH_RATE 4
#define YAW_RATE   5


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

// Controller gains and limits
float I_MAX_ROLL = 0.7*U_MAX_ROLL;
float I_MIN_ROLL = -I_MAX_ROLL; 
float kp_roll = 4.5;
float ki_roll = 0.0;
float kp_roll_rate = 0.7;

float I_MAX_PITCH = 0.7*U_MAX_PITCH;
float I_MIN_PITCH = -I_MAX_PITCH; 
float kp_pitch = 4.5;
float ki_pitch = 0.0;
float kp_pitch_rate = 0.7;

float I_MAX_YAW = 0.7*U_MAX_YAW;
float I_MIN_YAW = -I_MAX_YAW; 
float kp_yaw = 2;
float ki_yaw = 0.0;
float kp_yaw_rate = 0.4;

//Reference model parameters
float zeta = 1;
float omega = 1/(2*dt);

// Loop variables and constants
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
float tau[4];
float error_angle[3] = {0,0,0};
float error_angular_rate[3] = {0,0,0};
float RPY[3] = {0,0,0};
float RPY_REF[3] = {0,0,0};
float thrust = 300;


// Misc.
uint8_t buflen = sizeof(buf);
int magContinous = 1;
int motorPin[4] = {5,8,12,2};
float rad2deg = 180/PI;


// Modules
HMC5883L magsens;
MPU6050  mpu;
PassiveObserver npo(dt);

PID PIDS[6];
Quadcopter quad(U_MIN, U_MAX, ESC_MIN, ESC_MAX);
RH_ASK driver(radio_speed, rxPin, txPin, enablePin, false); // Last bool deals with signal inversion
RHDatagram radio(driver, QUAD_ADDR);
RefMod refmod_thrust(zeta, omega, dt);

void setup() {
  int initError = 0;
  Serial.begin(9600);
  
  PIDS[ROLL].init(kp_roll, ki_roll, 0, I_MIN_ROLL, I_MAX_ROLL, U_MIN_ROLL, U_MAX_ROLL);
  PIDS[PITCH].init(kp_pitch, ki_pitch, 0, I_MIN_PITCH, I_MAX_PITCH, U_MIN_PITCH, U_MAX_PITCH);
  PIDS[YAW].init(kp_yaw, ki_yaw, 0, I_MIN_YAW, I_MAX_YAW, U_MIN_YAW, U_MAX_YAW);
  PIDS[ROLL_RATE].init(kp_roll_rate, 0, 0, 0, 0, U_MIN, U_MAX);
  PIDS[PITCH_RATE].init(kp_pitch_rate, 0, 0, 0, 0, U_MIN, U_MAX);
  PIDS[YAW_RATE].init(kp_yaw_rate, 0, 0, 0, 0, U_MIN, U_MAX);
  
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
  npo.getRPY((float*)RPY);
  RPY_REF[YAW] = RPY[YAW];
  //npo.Offset(); // Stores the current attitude as a bias that will be subtracted from the measurement
  
  if (!radio.init()) {
    initError += 1;
  }
  
  if (!initError) {
    quad.init((int*) motorPin);
    delay(4000);
  } else {
    Serial.println("\nError during initialization");
    while(1) {
      delay(200);
    }
  }
}

int count = 0;
int print = 0;
int radioLost = 0;

void loop() {
  now = millis();
  if (now-prev > T) {
    prev = now;
    readRemote();
    count++;

    mpu.update(acc, gyro, mag);
    npo.update(acc, gyro, mag);
    npo.getRPY(RPY);
    npo.getGyro(gyro);
    
    tau[4] = thrust;
    
    if (thrust > 0) {
      for (int i=0; i<3; i++) {
        error_angle[i] = RPY_REF[i]-rad2deg*RPY[i];
        error_angular_rate[i] = PIDS[i].update(error_angle[i],0,0)-rad2deg*gyro[i];
        tau[i] = PIDS[i+3].update(error_angular_rate[i],0,0);
      }
    }
    
    print = 0;
    if (count==50) {
      //for (int i=0; i<4; i++) {Serial.println(tau[i]);}
      print = 1;
      count = 0;
    }
    
    if (radioLost >= 30) {
      tau[0] = 0;
      tau[1] = 0;
      tau[2] = 0;
      tau[3] = -1000;
    }
 
    quad.input((float*)tau, print);
  }  
}


void readRemote() {
  // Execution 10-600 us
  if (radio.recvfrom(buf, &buflen)) {
      Serial.println();
      Serial.println(thrust);
      //RPY_REF[ROLL] = (float)(int8_t)buf[0];
      //RPY_REF[PITCH] = (float)(int8_t)buf[1];
      thrust = 10*((float)(int8_t)buf[0]);
      thrust = refmod_thrust.update(thrust);
      radioLost = 0;
  } else {
      //radioLost++;
  }
}


