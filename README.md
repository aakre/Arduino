# Arduino
My repository for Arduino projects. It contains libraries written by me
and some that I've found somwehere on the Internet.

So far the coolest project is a homemade quadcopter. The autopilot and IMU
are custom, and it all runs on an Ardunino Due.

## Sensor configuration
* MPU6050 for accelerometer and gyroscope data
* HMC5883L magnetometer

The MPU6050 controls the magnetometer on an auxiliary I2C bus in order
to obtain easy access to synchronized data. Radio control is done by
cheap 433 Mhz radio modules courtesy of China. I'm using the
[RadioHead library](http://www.airspayce.com/mikem/arduino/RadioHead/) for communication.

## Attitude estimation
Attitude and gyro bias estimation is done using a quaternion based
passive observer as detailed in
Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control", Ch. 11.

## Attitude control
The current version use linear PD-controllers for roll and pitch. More advanced
control will hopefully be implemented in the future.

