/**
 * @file udmrt_gps.h
 * @author Greg Molskow
 * @brief This class is created for UDMRT to be used with their GPS senor. This class is a child class of the UDMRT_Senor class which handles the ROS setup and interface. Additionally, this class handles the reading of serial messages from the GPS sensor which is hooked up via the Serial1 port. This clas is designed to work with the Arduino Nano 33 BLE Sense board but can be used with any arduino that has the Serial1 hardware serial port (RX/TX).
 * @version 2
 * @date 2024-07-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef UDMRT_GPS_H
#define UDMRT_GPS_H

#include "../udmrt_sensor.cpp"
#include <TinyGPSPlus.h>
#include <float.h>
#include <sensor_msgs/NavSatFix.h>


class UDMRT_GPS: public UDMRT_Sensor<sensor_msgs::NavSatFix>{
  /** IMPORTANT: This code is written specifically for a GPS unit that receives signal at a frequency of 1Hz. 
    If a GPS with a higher refresh rate is used in the future, this code will need to be reworked. -Kaiden
  */

  public:
    /**
     * @brief Construct a new udmrt gps object
     * 
     * @param name - The name of the senor
     * @param node - The ROS node this sensor is associated with.
     */
    UDMRT_GPS(char* name, ros::NodeHandle* node);

    /**
     * @brief This function initalized the ROS publishers as well as the Serial1 port
     * 
     * @param dataPublisher - A pointer to the ROS publisher that is to be used for the Data
     * @param diagnosticPublisher - A pointer to the ROS publisher that is to be used for the diagnostics message
     * @return true - Returns true if Serial1 started, else false
     */
    bool init(ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher);

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

    TinyGPSPlus gps;

    bool gpsConnected;
    bool gpsError;
    int lastTimeStamp = millis();

    /**
     * @brief This function updates the diagnostic values to inicate an error
     * 
     */
    void errorState();

    /**
     * @brief This function updates the diagnostic values to indicate a warning
     * 
     */
    void warningState();

    /**
     * @brief This function updates the diagnotic values to indicate all OK
     * 
     */
    void okState();

};

#endif