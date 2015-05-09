#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

#define HMC5883L_ADDRESS            0x1E
#define HMC5883L_DEFAULT_ADDRESS    0x1E

#define HMC5883L_CRA                0x00
#define HMC5883L_CRB                0x01
#define HMC5883L_MR                 0x02
#define HMC5883L_DATAX_H            0x03
#define HMC5883L_DATAX_L            0x04
#define HMC5883L_DATAZ_H            0x05
#define HMC5883L_DATAZ_L            0x06
#define HMC5883L_DATAY_H            0x07
#define HMC5883L_DATAY_L            0x08
#define HMC5883L_STATUS             0x09
#define HMC5883L_ID_A               0x0A
#define HMC5883L_ID_B               0x0B
#define HMC5883L_ID_C               0x0C

#define HMC5883L_CRA0               0
#define HMC5883L_CRA1               1
#define HMC5883L_CRA2               2
#define HMC5883L_CRA3               3
#define HMC5883L_CRA4               4
#define HMC5883L_CRA5               5
#define HMC5883L_CRA6               6
#define HMC5883L_CRA7               7

#define HMC5883L_CRB0               0
#define HMC5883L_CRB1               1
#define HMC5883L_CRB2               2
#define HMC5883L_CRB3               3
#define HMC5883L_CRB4               4
#define HMC5883L_CRB5               5
#define HMC5883L_CRB6               6
#define HMC5883L_CRB7               7

#define HMC5883L_MR0                0
#define HMC5883L_MR1                1


#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

typedef union mag_union {
  struct {
    uint8_t mx_l;
    uint8_t mx_h;
    uint8_t mz_l;
    uint8_t mz_h;
    uint8_t my_l;
    uint8_t my_h;
  } reg;
  struct {
    int16_t mx;
    int16_t mz;
    int16_t my;
  } value;
};


class HMC5883L {
  mag_union mag;
  int read(int start, uint8_t *buffer, int size);
  int write(int start, const uint8_t *pData, int size);
  int write_reg(int reg, uint8_t data);
public:
  HMC5883L() {};
  int init(int);
  void singleRead(int *buffer);
  void update(int *buffer);
};
#endif
