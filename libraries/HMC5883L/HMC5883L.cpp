#include "HMC5883L.h"

int HMC5883L::init(int continuous) {
  /* Configuration Register A:
     - 8 sample average
     - Normal mode
     - 75 Hz output rate
  */
  int error;

  uint8_t config = 0;
  config  = HMC5883L_AVERAGING_8 << HMC5883L_CRA5;
  config |= HMC5883L_BIAS_NORMAL << HMC5883L_CRA0;
  config |= HMC5883L_RATE_75     << HMC5883L_CRA2;

  if (write_reg(HMC5883L_CRA, config)) {
    Serial.println("HMC5883L: Error setting CRA");
    return -1;
  }
  /* Configuration Register B:
     - 1090 LSB/Gauss gain
     - rest is zeros
  */
  config = 0;
  config |= HMC5883L_GAIN_1090 << HMC5883L_CRB5;
  if (write_reg(HMC5883L_CRB, config)) {
    Serial.println("HMC5883L: Error setting CRB");
    return -1;
  }
  /* Continuous mode... */
  if (continuous) {
    read(HMC5883L_MR, &config, 1);
    Serial.print("\nHMC5883L mode register: ");
    Serial.print(config, BIN);

    config = HMC5883L_MODE_CONTINUOUS << HMC5883L_MR0;
    write_reg(HMC5883L_MR, config);

    read(HMC5883L_MR, &config, 1);
    Serial.print("\nHMC5883L mode register: ");
    Serial.print(config, BIN);
  }
  return 0;
}

void HMC5883L::singleRead(int *buffer) {
  write_reg(HMC5883L_MR, HMC5883L_MODE_SINGLE);
  delay(6); //busy wait
  update(buffer);
}


int HMC5883L::read(int start, uint8_t *buffer, int size) {
  int i, n, error;

  Wire.beginTransmission(HMC5883L_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(HMC5883L_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
    {
      buffer[i++]=Wire.read();
    }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}

int HMC5883L::write(int start, const uint8_t *pData, int size) {
  // --------------------------------------------------------
  // MPU6050_write
  //
  // This is a common function to write multiple bytes to an I2C device.
  //
  // If only a single register is written,
  // use the function MPU_6050_write_reg().
  //
  // Parameters:
  //   start : Start address, use a define for the register
  //   pData : A pointer to the data to write.
  //   size  : The number of bytes to write.
  //
  // If only a single register is written, a pointer
  // to the data has to be used, and the size is
  // a single byte:
  //   int data = 0;        // the data to write
  //   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
  //
  int n, error;

  Wire.beginTransmission(HMC5883L_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

int HMC5883L::write_reg(int reg, uint8_t data) {
  // --------------------------------------------------------
  // MPU6050_write_reg
  //
  // An extra function to write a single register.
  // It is just a wrapper around the write()
  // function, and it is only a convenient function
  // to make it easier to write a single register.
  //
  int error;

  error = write(reg, &data, 1);

  return (error);
}

void HMC5883L::update(int *buffer) {
  read(HMC5883L_DATAX_H, (uint8_t*) &mag, sizeof(mag));
  uint8_t swap;

  #define SWAP(x,y) swap = x; x = y; y = swap;

  SWAP(mag.reg.mx_h, mag.reg.mx_l);
  SWAP(mag.reg.mz_h, mag.reg.mz_l);
  SWAP(mag.reg.my_h, mag.reg.my_l);

  // Return in the "right" order
  buffer[0] = mag.value.mx;
  buffer[1] = mag.value.my;
  buffer[2] = mag.value.mz;
}
