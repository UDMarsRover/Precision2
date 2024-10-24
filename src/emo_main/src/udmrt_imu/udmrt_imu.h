/**
 * @file udmrt_imu.h
 * @author Greg Molskow
 *      Paul Lachcik
 * @brief The goal of this class is to create an interface for a imu that is to be used by UDMRT. This class expects the sensor to be a MPU6050 and be interfaced the I2C pins. This class inherits the UDMRT_Sensor class.
 * @version 2
 * @date 2024-07-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UDMRT_IMU_H
#define UDMRT_IMU_H

#include <ros.h>
#include "MPU6050/attitude.h"
#include "../udmrt_sensor.cpp"
#include <sensor_msgs/Imu.h>

#include <vector>

class UDMRT_IMU: public UDMRT_Sensor<sensor_msgs::Imu>{

    public:
        UDMRT_IMU(char* name, 
                     ros::NodeHandle* node, 
                     int rollWarningLimit = 30,
                     int rollErrorLimit = 60, 
                     int pitchWarningLimit = 30,
                     int pitchErrorLimit = 60);

        /**
         * @brief The function that pulls data from the sensor and updates the messages. Called by spin()
         * 
         */
        void updateData() override;

        /**
         * @brief The function that is to be called every process step. This function calls the updateData function and publishes the messages currently stored in memory.
         * 
         */
        void spin();

        void init(ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher);

        Attitude imu;

    private:

        int rollWarning;
        int rollError;
        int pitchWarning;
        int pitchError;

};


#endif