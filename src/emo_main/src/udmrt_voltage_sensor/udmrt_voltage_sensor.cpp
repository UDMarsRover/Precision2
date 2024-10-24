#ifndef UDMRT_VOLTAGE_SENSOR_CPP
#define UDMRT_VOLTAGE_SENSOR_CPP

#include "udmrt_voltage_sensor.h"

UDMRT_Voltage_Sensor::UDMRT_Voltage_Sensor(char* name, 
                         ros::NodeHandle* node,
                         int inputPin,
                         float underVoltEmergency,
                         float underVoltWarning):
    UDMRT_Sensor<sensor_msgs::BatteryState>(name,node){

    nh = node;
    underVoltEm = underVoltEmergency;
    underVoltWarn = underVoltWarning;
    in = inputPin;

}

void UDMRT_Voltage_Sensor::updateData(){
    analogVal = analogRead(in);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    analogVal = expFilter(0.1, prevValue, analogVal);
    prevValue = analogVal;

    data_msg.voltage = (analogVal * 0.2008451735 ) - 80.97365371;
    data_msg.percentage = (analogVal - minAnalogValue) / steps;

    if (data_msg.voltage <= underVoltEm) errorState(data_msg.voltage,"2","Undervolt Emergency");
    else if (data_msg.voltage <= underVoltWarn) warningState(data_msg.voltage,"1","Undervolt Warning");
    else okState();
}

void UDMRT_Voltage_Sensor::spin(){
    updateData();
    UDMRT_Sensor::spin();

}
#endif