/**
 * @file udmrt_sensor_.cpp
 * @author Greg Molskow
 * @brief This file contains the necessary imports for a basic sensor that is integrated into ROS1 using the rosserial arduino library. This file was created with the intention of providing a base to build a variety of custom sensors quickly and easily in a uniform fashion.
 * @version 0.1
 * @date 2024-07-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#ifndef UDMRT_SENSOR_
#define UDMRT_SENSOR_

#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <avr/dtostrf.h>
#include <diagnostic_msgs/KeyValue.h>

#define DIAGNOSTIC_STATUS_LENGTH 1
#define OK diagnostic_msgs::DiagnosticStatus::OK
#define WARN diagnostic_msgs::DiagnosticStatus::WARN
#define ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#define STALE diagnostic_msgs::DiagnosticStatus::STALE

class SensorNotConfiguredError : public std::runtime_error {
public:
    SensorNotConfiguredError(const std::string& message)
        : std::runtime_error(message) {}
};


template <typename ros_data_type> class UDMRT_Sensor{
    /**
     * @brief This class is meant to be a parent class to all UDMRT sensors that are connected to a Arduino Nano BLE sense 33. This class initializes the required publishers and sets up the required values for the diagnostics topic. The goal of this class is to standardize the creation of custom sensors and to make development and debugging more streamlined.
     *
     * 
     */


    public:

        UDMRT_Sensor(char* name, 
                     ros::NodeHandle* node){
            /**
             * @brief The constructor for the UDMRT_Sensor class
             * 
             * @param name The name of the sensor
             * @param node The ROS node that this sensor is connected too
             * 
             */
            diag_msg.name=name;
            diag_msg.message="Starting Up Sensor...";
            diag_msg.hardware_id=name;
            diag_msg.level=OK;
            diag_msg.values_length=DIAGNOSTIC_STATUS_LENGTH;

            nh=node;

            
        }

        void init(ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher){
            
            diag_pub = diagnosticPublisher;
            data_pub = dataPublisher;

            nh->advertise(*diag_pub);
            nh->advertise(*data_pub);

            setUp = true;
        }

        void spin(){

            //if (!setUp) throw SensorNotConfiguredError("Senor not initalized!");
            diag_msg.values = &errorCode;
            diag_pub->publish(&diag_msg);
            data_pub->publish(&data_msg);
        }

        virtual void updateData() = 0;

       
        
        diagnostic_msgs::KeyValue errorCode;
        diagnostic_msgs::DiagnosticStatus diag_msg;
        ros_data_type data_msg;


    protected:
        ros::Publisher* diag_pub;
        ros::Publisher* data_pub;
        ros::NodeHandle* nh;
        

        bool setUp = false;


        /**
         * @brief This function updates the diagnostic values to indicate an error
         * 
         */
        void errorState(double sensorValue, char* errorKey, char* errorMessage){
            char value[10];
            dtostrf(sensorValue, 5, 1, value);
            diag_msg.level = ERROR; 
            diag_msg.message = value;
            errorCode.key = errorKey;
            errorCode.value = errorMessage;
        }

        /**
         * @brief This function updates the diagnostic values to indicate a warning
         * 
         */
        void warningState(double sensorValue, char* warningKey,  char* warningMessage){
            char value[10];
            dtostrf(sensorValue, 5, 1, value);
            diag_msg.level = WARN;
            diag_msg.message = value;
            errorCode.key = warningKey;
            errorCode.value = warningMessage;
        }

        /**
         * @brief This function updates the diagnotic values to indicate all OK
         * 
         */
        void okState(){
            diag_msg.level=OK;
            diag_msg.message="";
            errorCode.key="0";
            errorCode.value="All Good";
        }

        double expFilter(double alpha, double prevReading, double curReading){ 
            return (alpha * curReading) + ((1 - alpha) * prevReading);
        }
        





};

#endif