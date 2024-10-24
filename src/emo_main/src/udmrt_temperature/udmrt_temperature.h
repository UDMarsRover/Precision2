/**
 * @file udmrt_temperature.h
 * @author Greg Molskow
 * @brief Thi class is created to interface with the temperature sensor onboard the arduino nano BLE sense.
 * @version 0.1
 * @date 2024-07-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UDMRT_TEMPERATURE_H
#define UDMRT_TEMPERATURE_H

#include <Arduino.h>
#include "../udmrt_sensor.cpp"
#include <sensor_msgs/Temperature.h>
#include <Arduino_HTS221.h> 

class UDMRT_Temperature : public UDMRT_Sensor<sensor_msgs::Temperature>{

    public:
        UDMRT_Temperature(char* name, ros::NodeHandle* node);

        void init(ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher);

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

};

#endif