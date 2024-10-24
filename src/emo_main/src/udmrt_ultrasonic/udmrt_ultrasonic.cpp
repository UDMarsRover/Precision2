#ifndef UDMRT_ULTRASONIC_CPP
#define UDMRT_ULTRASONIC_CPP

#include "udmrt_ultrasonic.h"

UDMRT_Ultrasonic::UDMRT_Ultrasonic(char* name, 
                         ros::NodeHandle* node,
                         int trigger_pin,
                         int echo_pin,
                         int max_distance,
                         int min_distance,
                         double alpha_value):
    UDMRT_Sensor<sensor_msgs::Range>(name,node){

    alphaValue = alpha_value;
    data_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    data_msg.min_range = min_distance;
    data_msg.max_range = max_distance;
    data_msg.range = 30;

}

void UDMRT_Ultrasonic::init(NewPing* sensor, ros::Publisher* dataPublisher, ros::Publisher* diagnosticPublisher){
    ultrasonicSensor = sensor;
    UDMRT_Sensor::init(dataPublisher,diagnosticPublisher);

}

void UDMRT_Ultrasonic::updateData(){
    currentDistance = ultrasonicSensor->ping_cm();
        
    data_msg.range = expFilter(alphaValue,data_msg.range, currentDistance);

    if (data_msg.range <= data_msg.min_range) {
        okState();
        diag_msg.message = "No detection";
        errorCode.value = "No detection";
    }
    else if (data_msg.range <= 30) errorState(data_msg.range,"2","Within 30 CM");
    else if (data_msg.range <= 50) warningState(data_msg.range,"1","Within 50 CM");
    else okState();

    
}

void UDMRT_Ultrasonic::spin(){
    updateData();
    UDMRT_Sensor::spin();
}


#endif