/**
 * @file udmrt_voltage_sensor.h
 * @author Greg Molskow
 * @brief This file is meant to interface with custom voltage sensor. This class inherits the UDMRT_Sensor class.
 * @version 2
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UMRT_VOLTAGE_SENSOR_H
#define UMRT_VOLTAGE_SENSOR_H

#include "../udmrt_sensor.cpp"
#include <sensor_msgs/BatteryState.h>

class UDMRT_Voltage_Sensor: public UDMRT_Sensor<sensor_msgs::BatteryState>{

    public:
        UDMRT_Voltage_Sensor(char* name, 
                         ros::NodeHandle* node,
                         int inputPin,
                         float underVoltEmergency,
                         float underVoltWarning);

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

    private:
        float underVoltEm;
        float underVoltWarn;
        int in;

        float analogVal = 0;
        float prevValue = 0;
        const float maxAnalogValue = 683.934;
        const float minAnalogValue = 630.235;

        const float steps = maxAnalogValue - minAnalogValue;
        

};

#endif