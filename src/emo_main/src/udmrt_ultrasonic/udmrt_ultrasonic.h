/**
 * @file udmrt_ultrasonic.h
 * @author Greg Molskow
 * @brief This class is used with the ultrasonic sensors for object detection on the UMDRT mars rover. This class is intended to represent a single sensor and is intended to be used on Arduino.
 * @version 0.1
 * @date 2024-07-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UDMRT_ULTRASONIC_H
#define UDMRT_ULTRASONIC_H

#include "../udmrt_sensor.cpp"
#include <sensor_msgs/Range.h>
#include <NewPing.h>



class UDMRT_Ultrasonic: public UDMRT_Sensor<sensor_msgs::Range>{

    public:
        UDMRT_Ultrasonic(char* name, 
                         ros::NodeHandle* node,
                         int trigger_pin,
                         int echo_pin,
                         int max_distance = 100,
                         int min_distance = 3,
                         double alpha_value = 0.5);

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

        void init(NewPing* sensor, ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher);
    
    private:

        double alphaValue;

        float currentDistance;

        NewPing* ultrasonicSensor;


};

#endif