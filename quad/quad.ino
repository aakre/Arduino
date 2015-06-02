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

#define U_MAX        100
#define U_MIN       -U_MAX
#define U_MAX_ROLL   U_MAX // 250
#define U_MIN_ROLL  -U_MAX_ROLL
#define U_MAX_PITCH  U_MAX_ROLL
#define U_MIN_PITCH -U_MAX_PITCH
#define U_MAX_YAW    U_MAX/4
#define U_MIN_YAW   -U_MAX_YAW
#define THRUST_MIN  -100
#define THRUST_MAX  35
#define TAU_MIN     -120
#define TAU_MAX     120

#define ROLL       0
#define PITCH      1
#define YAW        2
#define ROLL_RATE  3
#define PITCH_RATE 4
#define YAW_RATE   5
#define THRUST     3


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
int T = 5;
float dt = (float)T/1000;

// Measurement data
float acc[3];
float gyro[3];
float mag[3];

// Controller gains and limits
float I_MAX_ROLL = 0.7*U_MAX_ROLL;
float I_MIN_ROLL = -I_MAX_ROLL; 
float kp_roll = 0.0;
float ki_roll = 0.0;
float kd_roll = 6.0;

float I_MAX_PITCH = 0.7*U_MAX_PITCH;
float I_MIN_PITCH = -I_MAX_PITCH; 
float kp_pitch = 0.0;
float ki_pitch = 0.0;
float kd_pitch = 0.0;

float I_MAX_YAW = 0.7*U_MAX_YAW;
float I_MIN_YAW = -I_MAX_YAW; 
float kp_yaw = 0.7;
float ki_yaw = 0.0;
float kd_yaw = 0.01;

//Reference model parameters
float zeta = 1;
float omega = 1/(2*dt);

// Loop variables and constants
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
float tau[4] = {0,0,0,0};
float error_angle = 0;
float error_angular_rate = 0;
float RPY[3] = {0,0,0};
float REF[4] = {0,0,0,0};
float PILOT[3] = {0,0,0};


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
Quadcopter quad(TAU_MIN, TAU_MAX, ESC_MIN, ESC_MAX);
RH_ASK driver(radio_speed, rxPin, txPin, enablePin, false); // Last bool deals with signal inversion
RHDatagram radio(driver, QUAD_ADDR);
RefMod REFMODS[4];

void setup() {
  int initError = 0;
  Serial.begin(9600);
  PIDS[ROLL].init(kp_roll, ki_roll, kd_roll, I_MIN_ROLL, I_MAX_ROLL, U_MIN_ROLL, U_MAX_ROLL);
  PIDS[PITCH].init(kp_pitch, ki_pitch, 0.0, I_MIN_PITCH, I_MAX_PITCH, U_MIN_PITCH, U_MAX_PITCH);
  PIDS[YAW].init(kp_yaw, ki_yaw, 0, I_MIN_YAW, I_MAX_YAW, U_MIN_YAW, U_MAX_YAW);
  PIDS[ROLL_RATE].init(kd_roll, 0, 0, 0, 0, U_MIN, U_MAX);
  PIDS[PITCH_RATE].init(kd_pitch, 0, 0, 0, 0, U_MIN, U_MAX);
  PIDS[YAW_RATE].init(kd_yaw, 0, 0, 0, 0, U_MIN, U_MAX);
  
  for (int i=0; i<4; i++) {
    REFMODS[i].init(zeta, omega, dt);
  }
  
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
  REF[YAW] = RPY[YAW];
  npo.setBias(); // Stores the current attitude as a bias that will be subtracted from the measurement
  
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
int i = 0;

int start = 0;
int end = 0;

void loop() {
  now = millis();
  if (now-prev > T) {
    prev = now;
    
    readRemote();
    if (radioLost >= 80) {
      PILOT[0] = 0;
      PILOT[1] = 0;
      PILOT[2] = -90; // Set this to slightly below the value to hover
    }
    
    REF[ROLL] = REFMODS[ROLL].update(PILOT[0]);
    REF[PITCH] = REFMODS[PITCH].update(PILOT[1]);
    REF[THRUST] = REFMODS[THRUST].update(PILOT[2]);
    
    mpu.update(acc, gyro, mag);
    npo.update(acc, gyro, mag);
    npo.getRPY((float*)RPY);
    npo.getGyro((float*)gyro);
    
    
    
    tau[THRUST] = REF[THRUST];
    
    if (tau[THRUST] > -70) {
     error_angle = -rad2deg*RPY[ROLL];
     error_angular_rate = -rad2deg*gyro[0];
     tau[ROLL] = PIDS[ROLL].update(0,error_angular_rate,print);
      /* for (i=0; i<2; i++) { */
      /*   error_angle = REF[i]-rad2deg*RPY[i]; */
      /*   error_angular_rate = PIDS[i].update(error_angle,0,0) - rad2deg*gyro[i]; */
      /*   tau[i] = PIDS[i+3].update(error_angular_rate,0,print); */
      /* } */
    } else {
        tau[ROLL] = 0;
        tau[PITCH] = 0;
        tau[YAW] = 0;
    }
    
    print = 0;
    count++;
    if (count==100) {
      /* Serial.println("\nRef \t RPY"); */
      /* for (int i=0; i<3; i++) { */
      /*   Serial.print(REF[i]); */
      /*   Serial.print("\t"); */
      /*   Serial.print(RPY[i]*57); */
      /*   Serial.print("\n"); */
      /* } */
      npo.printRPY(0);
      print = 0;
      count = 0;
    }
    
    quad.input((float*)tau, print);
    
  } // End quad routine  
}


void readRemote() {
  // Execution 10-600 us
  if (radio.recvfrom(buf, &buflen)) {
      //RPY_REF[ROLL] = (float)(int8_t)buf[0];
      //RPY_REF[PITCH] = (float)(int8_t)buf[1];
      buf[0] = max(0, min(255, buf[0]));
      buf[1] = max(0, min(255, buf[1]));
      buf[2] = max(0, min(255, buf[2]));
      PILOT[0] = map(buf[0], 0, 255, -45, 45);
      PILOT[1] = map(buf[1], 0, 255, -45, 45);
      PILOT[2] = map(buf[2], 0, 255, THRUST_MIN, THRUST_MAX);
      radioLost = 0;
  } else {
      radioLost++;
  }
}


