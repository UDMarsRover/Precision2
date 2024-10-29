#ifndef UDMRT_TEMPERATURE_CPP
#define UDMRT_TEMPERATURE_CPP

#include "udmrt_temperature.h"

UDMRT_Temperature::UDMRT_Temperature(char* name, NodeHandle* node):
    UDMRT_Sensor<sensor_msgs::msg::Temperature>(name, node) {
}

void UDMRT_Temperature::init(Publisher* dataPublisher, Publisher* diagnosticPublisher) {
    if (!HTS.begin()) {
        node->logerror("Unable to start box temperature sensor!!!");
        errorState(0, "1", "Unable to start sensor");
    }
    UDMRT_Sensor::init(dataPublisher, diagnosticPublisher);
}

void UDMRT_Temperature::updateData() {
    data_msg.temperature = HTS.readTemperature() - 3.0; // -3 because of constant on-board temperature increase
    /** @bug Why does this only work with a delay?????
    */
    //delay(100);  // Only works with delay greater than or equal to 40 ms

    if (data_msg.temperature >= 75) errorState(data_msg.temperature, "3", "Overheat Error!");
    else if (data_msg.temperature >= 65) warningState(data_msg.temperature, "2", "Overheat Warning");
    else okState();
}

void UDMRT_Temperature::spin() {
    updateData();
    UDMRT_Sensor::spin();
}

#endif