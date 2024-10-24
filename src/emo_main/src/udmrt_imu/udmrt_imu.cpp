#ifndef UDMRT_IMU_CPP
#define UDMRT_IMU_CPP

#include "udmrt_imu.h"

UDMRT_IMU::UDMRT_IMU(char* name, 
                     ros::NodeHandle* node, 
                     int rollWarningLimit,
                     int rollErrorLimit, 
                     int pitchWarningLimit,
                     int pitchErrorLimit):
    UDMRT_Sensor<sensor_msgs::Imu>(name, node){

    UDMRT_IMU::rollError = rollErrorLimit;
    UDMRT_IMU::pitchError = pitchErrorLimit;
    UDMRT_IMU::rollWarning = rollWarningLimit;
    UDMRT_IMU::pitchWarning = pitchWarningLimit;

    

}

void UDMRT_IMU::spin(){
    updateData();
    UDMRT_Sensor::spin();
}

void UDMRT_IMU::init(ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher){
    imu.initialize();
    UDMRT_Sensor::init(dataPublisher,diagnosticPublisher);
}

void UDMRT_IMU::updateData(){
    imu.spin();
    data_msg.orientation.w = imu.quaternion.w;
    data_msg.orientation.x = imu.quaternion.x;
    data_msg.orientation.y = imu.quaternion.y;
    data_msg.orientation.z = imu.quaternion.z;

    data_msg.angular_velocity.x = imu.angular_vel.x;
    data_msg.angular_velocity.y = imu.angular_vel.y;
    data_msg.angular_velocity.z = imu.angular_vel.z;
    
    data_msg.linear_acceleration.x = imu.linear_acc.x;
    data_msg.linear_acceleration.y = imu.linear_acc.y;
    data_msg.linear_acceleration.z = imu.linear_acc.z;

    float roll = imu.yawPitchRoll.z;
    float pitch = imu.yawPitchRoll.y;

    if (abs(roll) >= rollError) errorState(roll,"4","Roll Emergency");
    else if (abs(pitch) >= pitchError) errorState(pitch,"3","Pitch Emergency");
    else if (abs(roll) >= rollWarning) warningState(roll,"2","Roll Warning");
    else if (abs(pitch) >= pitchWarning) warningState(pitch,"1","Pitch Warning");
    else okState();
}

#endif