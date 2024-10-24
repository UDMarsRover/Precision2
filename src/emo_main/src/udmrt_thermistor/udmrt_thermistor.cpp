#ifndef UDMRT_THERMISOR_CPP
#define UDMRT_THERMISOR_CPP

#include "udmrt_thermistor.h"

#define AVERAGE_SAMPLE_SIZE 5

UDMRT_Thermistor::UDMRT_Thermistor(char* name, 
                         ros::NodeHandle* node,
                         int inputPin,
                         int thermistor_value,
                         int r_2_value,
                         int b_coefficient,
                         float overHeatEmergency,
                         float underHeatEmergency,
                         float overHeatWarning,
                         float underHeatWarning):
    UDMRT_Sensor<sensor_msgs::Temperature>(name,node){
        in = inputPin;
        thermVal = thermistor_value;
        r2 = r_2_value;
        bCoeff = b_coefficient;
        overEmer = overHeatEmergency;
        underEmer = overHeatEmergency;
        overWarn = overHeatWarning;
        underWarn = underHeatWarning;
}

void UDMRT_Thermistor::updateData(){

    double average = 0;

    for (int i =0; i< AVERAGE_SAMPLE_SIZE; i++) average += analogRead(in);

    average /= AVERAGE_SAMPLE_SIZE;
    average = ((1023 * r2) / average ) - r2;
  
    //  1 / ( (ln(R/Ro)/B) + (1/To) ) - 273.15  
    data_msg.temperature = ( 1 / ( ( ( log(average / thermVal) ) / bCoeff ) + ( 1.0 / (25 + 273.15) ) ) ) - 273.15;

    if (data_msg.temperature >= overEmer) errorState(data_msg.temperature,"4","Overheat Emergency");
    else if(data_msg.temperature >= overWarn) warningState(data_msg.temperature,"2","Overheat Warning");
    else if (data_msg.temperature <= underEmer) errorState(data_msg.temperature,"3","Underheat Error");
    else if (data_msg.temperature <= underWarn) warningState(data_msg.temperature,"1","Underheat Warning");
    else okState();
    
}

void UDMRT_Thermistor::spin(){
    updateData();
    UDMRT_Sensor::spin();
}

#endif