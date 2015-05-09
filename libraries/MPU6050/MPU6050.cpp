#include "MPU6050.h"


int MPU6050::read(int start, uint8_t *buffer, int size) {
  // --------------------------------------------------------
  // MPU6050_read
  //
  // This is a common function to read multiple bytes 
  // from an I2C device.
  //
  // It uses the boolean parameter for Wire.endTransMission()
  // to be able to hold or release the I2C-bus. 
  // This is implemented in Arduino 1.0.1.
  //
  // Only this function is used to read. 
  // There is no function for a single byte.
  //
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


int MPU6050::write(int start, const uint8_t *pData, int size) {
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

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
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


int MPU6050::write_reg(int reg, uint8_t data) {
  // --------------------------------------------------------
  // MPU6050_write_reg
  //
  // An extra function to write a single register.
  // It is just a wrapper around the MPU_6050_write()
  // function, and it is only a convenient function
  // to make it easier to write a single register.
  //
  int error;

  error = write(reg, &data, 1);

  return (error);
}

int MPU6050::init() {
  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //
  Wire.begin();
  uint8_t c;
  int error;
  error = read(MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // Set up low pass filter on the MPU6050
  write_reg(MPU6050_CONFIG, MPU6050_DLPF_5HZ);

  // Clear the 'sleep' bit to start the sensor.
  write_reg(MPU6050_PWR_MGMT_1, 0);
  return error;
}

void MPU6050::addSlave(uint8_t slave_addr, uint8_t read_reg) {
  // Let the MPU wait for external sensors to deliver data
  uint8_t mst_ctrl;
  read(MPU6050_I2C_MST_CTRL, &mst_ctrl, 1);
  bitSet(mst_ctrl, MPU6050_WAIT_FOR_ES);
  write_reg(MPU6050_I2C_MST_CTRL, mst_ctrl);

  // Set up the slave 0 control register
  // Enable read operation
  // Enable the slave, set length to 6 bytes
  uint8_t slv0_addr = bit(MPU6050_I2C_SLV0_RW) | slave_addr; //
  uint8_t slv0_reg = read_reg;
  uint8_t slv0_ctrl = bit(MPU6050_I2C_SLV0_EN) | bit(MPU6050_I2C_SLV0_LEN1) | bit(MPU6050_I2C_SLV0_LEN2);
  write_reg(MPU6050_I2C_SLV0_ADDR, slv0_addr);
  write_reg(MPU6050_I2C_SLV0_REG, slv0_reg);
  write_reg(MPU6050_I2C_SLV0_CTRL, slv0_ctrl);
}

void MPU6050::enableSlaveConfig() {
  // Use this function to enable direct control of the MPU6050's slave
  // Useful when you want to configure the slave
  uint8_t pin_cfg, user_ctrl;
  read(MPU6050_INT_PIN_CFG, &pin_cfg, 1);
  read(MPU6050_USER_CTRL, &user_ctrl, 1);
  Serial.print("\nstart Enable bypass");
  Serial.print("\npin_cfg: ");
  Serial.print(pin_cfg);
  Serial.print("\nuser_ctrl: ");
  Serial.print(user_ctrl);
  
  bitSet(pin_cfg, MPU6050_I2C_BYPASS_EN);
  bitClear(user_ctrl, MPU6050_I2C_MST_EN);
  
  write_reg(MPU6050_INT_PIN_CFG, pin_cfg); //enable bypass
  write_reg(MPU6050_USER_CTRL, user_ctrl);
}

void MPU6050::disableSlaveConfig() {
  // Use this function to enable direct control of the MPU6050's slave
  // Useful when you want to configure the slave
  uint8_t pin_cfg, user_ctrl;
  read(MPU6050_INT_PIN_CFG, &pin_cfg, 1);
  read(MPU6050_USER_CTRL, &user_ctrl, 1);

  Serial.print("\nstart disable bypass");
  Serial.print("\npin_cfg: ");
  Serial.print(pin_cfg);
  Serial.print("\nuser_ctrl: ");
  Serial.print(user_ctrl);
  
  bitClear(pin_cfg, MPU6050_I2C_BYPASS_EN);
  bitSet(user_ctrl, MPU6050_I2C_MST_EN);

  Serial.print("\nend disable bypass");
  Serial.print("\npin_cfg: ");
  Serial.print(pin_cfg);
  Serial.print("\nuser_ctrl: ");
  Serial.print(user_ctrl);
  
  write_reg(MPU6050_INT_PIN_CFG, pin_cfg); //disable bypass
  write_reg(MPU6050_USER_CTRL, user_ctrl); //enable MPU6050 as master to the aux-devices
  uint8_t fifo_en;
  read(MPU6050_FIFO_EN, &fifo_en, 1);
  Serial.print("\nFIFO EN after disable bypass: ");
  Serial.print(fifo_en, BIN);
}



void MPU6050::update(float *acc, float *gyro) {
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  int bytesToRead = 14;
  int error = read(MPU6050_ACCEL_XOUT_H,
                            (uint8_t *) &sensor_data, bytesToRead);
  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap;
  SWAP (sensor_data.reg.x_accel_h, sensor_data.reg.x_accel_l);
  SWAP (sensor_data.reg.y_accel_h, sensor_data.reg.y_accel_l);
  SWAP (sensor_data.reg.z_accel_h, sensor_data.reg.z_accel_l);
  SWAP (sensor_data.reg.t_h, sensor_data.reg.t_l);
  SWAP (sensor_data.reg.x_gyro_h, sensor_data.reg.x_gyro_l);
  SWAP (sensor_data.reg.y_gyro_h, sensor_data.reg.y_gyro_l);
  SWAP (sensor_data.reg.z_gyro_h, sensor_data.reg.z_gyro_l);

  acc[0] = sensor_data.value.x_accel;
  acc[1] = sensor_data.value.y_accel;
  acc[2] = sensor_data.value.z_accel;
  gyro[0] = sensor_data.value.x_gyro;
  gyro[1] = sensor_data.value.y_gyro;
  gyro[2] = sensor_data.value.z_gyro;
}

void MPU6050::update(float *acc, float *gyro, float *mag) {
  int bytesToRead = 20;
  int error = read(MPU6050_ACCEL_XOUT_H,
                            (uint8_t *) &sensor_data, bytesToRead);
  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap;

  SWAP (sensor_data.reg.x_accel_h, sensor_data.reg.x_accel_l);
  SWAP (sensor_data.reg.y_accel_h, sensor_data.reg.y_accel_l);
  SWAP (sensor_data.reg.z_accel_h, sensor_data.reg.z_accel_l);
  SWAP (sensor_data.reg.t_h, sensor_data.reg.t_l);
  SWAP (sensor_data.reg.x_gyro_h, sensor_data.reg.x_gyro_l);
  SWAP (sensor_data.reg.y_gyro_h, sensor_data.reg.y_gyro_l);
  SWAP (sensor_data.reg.z_gyro_h, sensor_data.reg.z_gyro_l);

  SWAP (sensor_data.reg.mx_h, sensor_data.reg.mx_l);
  SWAP (sensor_data.reg.mz_h, sensor_data.reg.mz_l);
  SWAP (sensor_data.reg.my_h, sensor_data.reg.my_l);

  acc[0]  = sensor_data.value.x_accel;
  acc[1]  = sensor_data.value.y_accel;
  acc[2]  = sensor_data.value.z_accel;
  gyro[0] = sensor_data.value.x_gyro;
  gyro[1] = sensor_data.value.y_gyro;
  gyro[2] = sensor_data.value.z_gyro;
  mag[0]  = sensor_data.value.mx;
  mag[1]  = sensor_data.value.my;
  mag[2]  = sensor_data.value.mz;
}

int16_t MPU6050::getAX() {
  return sensor_data.value.x_accel;
}

int16_t MPU6050::getAY() {
  return sensor_data.value.y_accel;
}

int16_t MPU6050::getAZ() {
  return sensor_data.value.z_accel;
}

int16_t MPU6050::getGX() {
  return sensor_data.value.x_gyro;
}

int16_t MPU6050::getGY() {
  return sensor_data.value.y_gyro;
}

int16_t MPU6050::getGZ() {
  return sensor_data.value.z_gyro;
}

