// attitude.cpp
#include "attitude.h"

Attitude::Attitude() {

}

void Attitude::initialize(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Attitude::mpu.initialize();
  Attitude::devStatus = Attitude::mpu.dmpInitialize();

  if (Attitude::devStatus == 0) {

    Attitude::mpu.CalibrateAccel(7);
    Attitude::mpu.CalibrateGyro(7);
    Attitude::mpu.PrintActiveOffsets();
    Attitude::mpu.setDMPEnabled(true);

    Attitude::mpuIntStatus = Attitude::mpu.getIntStatus();

    Attitude::packetSize = Attitude::mpu.dmpGetFIFOPacketSize();
  }
}


void Attitude::spin(){
  /** @brief This function is used to update the values of this class

    The goal of the spin function is to update all of the values for the specific vector. 
    This is doen by getting a reading from the gyroscope and accelerometer and saving it to 
    a class variable. This function is meant to be used for every timestep.

    @return None, this function updates class variables
  */
  if (mpu.dmpGetCurrentFIFOPacket(Attitude::fifoBuffer)) {
    Attitude::mpu.dmpGetQuaternion(&(Attitude::q), Attitude::fifoBuffer);    //Update Q Value
    Attitude::mpu.dmpGetGravity(&(Attitude::gravity), &(Attitude::q));          //Update gravity

    /* Get the Accel*/
    Attitude::mpu.dmpGetAccel(&(Attitude::aa), Attitude::fifoBuffer);
    Attitude::mpu.dmpConvertToWorldFrame(&(Attitude::aaWorld), &(Attitude::aa), &(Attitude::q)); 

    /* Get the Gyro*/
    Attitude::mpu.dmpGetGyro(&(Attitude::gg), Attitude::fifoBuffer);
    Attitude::mpu.dmpConvertToWorldFrame(&(Attitude::ggWorld), &(Attitude::gg), &(Attitude::q));

    /* Update the YPR*/
    Attitude::mpu.dmpGetYawPitchRoll(Attitude::ypr, &(Attitude::q), &(Attitude::gravity));

    /* Update Public Variables */
    Attitude::quaternion = Attitude::q;
    Attitude::linear_acc.x = Attitude::aaWorld.x * Attitude::mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    Attitude::linear_acc.y = Attitude::aaWorld.y * Attitude::mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    Attitude::linear_acc.z = Attitude::aaWorld.z * Attitude::mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

    Attitude::yawPitchRoll.x = Attitude::ypr[0] * (RAD_TO_DEG);
    Attitude::yawPitchRoll.y = Attitude::ypr[1] * (RAD_TO_DEG);
    Attitude::yawPitchRoll.z = Attitude::ypr[2] * (RAD_TO_DEG);

    Attitude::angular_vel.x = Attitude::ggWorld.x * Attitude::mpu.get_gyro_resolution() * DEG_TO_RAD;
    Attitude::angular_vel.x = Attitude::ggWorld.x * Attitude::mpu.get_gyro_resolution() * DEG_TO_RAD;
    Attitude::angular_vel.x = Attitude::ggWorld.x * Attitude::mpu.get_gyro_resolution() * DEG_TO_RAD;


  }
}

/** @bug TO BE DELETED****
Quaternion * Attitude::getQrt() {
  if (!dmpReady) return &q;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //qrt[0] = q.w;
    //qrt[1] = q.x;
    //qrt[2] = q.y;
    //qrt[3] = q.z;
    return &q;
  }
}


void Attitude::getYpr(float* data) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    mpu.dmpGetGyro(&gg, fifoBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    mpu.dmpGetYawPitchRoll(&ypr, &q, &gravity);
    data[0] = ypr[0] * (RAD_TO_DEG);
    data[1] = ypr[1] * (RAD_TO_DEG);
    data[2] = ypr[2] * (RAD_TO_DEG);
  }
}

void Attitude::getAcc(float* data) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    data[0] = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    data[1] = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    data[2] = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
  }
}

void Attitude::getGyr(float* data) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetGyro(&gg, fifoBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    data[0] = aaWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
    data[1] = aaWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
    data[2] = aaWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;
  }
}
*/