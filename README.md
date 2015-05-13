# Arduino
My repository for Arduino projects. It contains libraries written by me
and some that I've found somwehere on the Internet.

So far the coolest project is a homemade quadcopter. It features an Arduino Due,
custom IMU, breadboards and a nice wire spaghetti.

## Sensor configuration
MPU6050 for accelerometer and gyroscope data
HMC5883L magnetometer.
The MPU is controller the magnetometer on an auxiliary I2C bus in order
to obtain easy access to synchronized data.
Cheap 433 Mhz radio modules courtesy of China. I'm using the RadioHead library.

## Attitude estimation
Attitude and gyro bias estimation is done using a quaternion based
passive observer as detailed in
Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control", Ch. 11.

## Attitude control
PD-like control through cascading a PI- (I is obtional) and a P-controller.
The outer control loop corrects deviations from desired angle and outputs
the setpoint for the inner loop. The inner loop is thus in charge of correcting
the angular velocity.
In total, six controllers are used, 3 on each axis.


