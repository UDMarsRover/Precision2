#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

class Attitude {
  public:
    Attitude();
    void initialize();
    Quaternion quaternion;          // [w, x, y, z]         quaternion value
    VectorFloat yawPitchRoll;       // [x,y,z]
    VectorFloat linear_acc;         // [x,y,z]
    VectorFloat angular_vel;        // [x,y,z]
    void spin();

  private:
    // MPU control/status vars
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars

    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    Quaternion q;          // [w, x, y, z]         quaternion value
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float acc[3];           // [ax, ay, az]         Accelerometer data for x, y, and z
    float gyr[3];           // [gx, gy, gz]         Gyroscope data for x, y, and z

    MPU6050 mpu;
};


#endif
// ATTITUDE_H