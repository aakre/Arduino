// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// By arduino.cc user "Krodal"
// Rewritten to a C++ class by Øyvind Løberg Aakre / oyvinaak
//
// June 2012
//      first version
// July 2013 
//      The 'int' in the union for the x,y,z
//      changed into int16_t to be compatible
//      with Arduino Due.
// February 2014
//      Rewritten to a C++ class
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version, 
// since Wire.endTransmission() uses a parameter 
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-6000 and MPU-6050 Product Specification",
//     PS-MPU-6000A.pdf
//   - "MPU-6000 and MPU-6050 Register Map and Descriptions",
//     RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
//   - "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
//     AN-MPU-6000EVB.pdf
// 
// The accuracy is 16-bits.
//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
// 

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

#ifndef __sensor_data_union__
#define __sensor_data_union__
typedef union sensor_data_union {
  struct {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
    uint8_t mx_l;
    uint8_t mx_h;
    uint8_t mz_l;
    uint8_t mz_h;
    uint8_t my_l;
    uint8_t my_h;
  } reg;
  struct {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
    int16_t mx;
    int16_t mz;
    int16_t my;
  } value;
} sensor_data_union;
#endif

#ifndef __MPU6050__
#define __MPU6050__

#include "MPU6050_defines.h"

class MPU6050 {
  int read(int start, uint8_t *buffer, int size);
  int write(int start, const uint8_t *pData, int size);
  int write_reg(int reg, uint8_t data);
  sensor_data_union sensor_data;
public:
  MPU6050() {};
  int init();
  void update(float *acc, float *gyro);
  void update(float *acc, float *gyro, float *mag);
  void addSlave(uint8_t slave_addr, uint8_t read_reg);
  void enableSlaveConfig();
  void disableSlaveConfig();
  int16_t getAX();
  int16_t getAY();
  int16_t getAZ();
  int16_t getGX();
  int16_t getGY();
  int16_t getGZ();
};

#endif




