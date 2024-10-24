/**
 * @file udmrt_thermistor.h
 * @author Greg Molskow
 *      Paul lachcik
 * @brief This class is meant to interface with voltage divider circuit where one of the resistors is a thermistor. This class handles the interface with ROS and the mathmatical calculations to convert the analog reading into a temperature reading. 
 * @version 2
 * @date 2024-07-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UDMRT_THERMISOR_H
#define UDMRT_THERMISOR_H

#include "../udmrt_sensor.cpp"
#include <sensor_msgs/Temperature.h>

class UDMRT_Thermistor: public UDMRT_Sensor<sensor_msgs::Temperature> {

    public:
        UDMRT_Thermistor(char* name, 
                         ros::NodeHandle* node,
                         int inputPin,
                         int thermistor_value,
                         int r_2_value,
                         int b_coefficient,
                         float overHeatEmergency,
                         float underHeatEmergency,
                         float overHeatWarning,
                         float underHeatWarning);

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
        int in;
        int thermVal;
        int r2;
        int bCoeff;

        float overEmer;
        float underEmer;
        float overWarn;
        float underWarn;

};

#endif 