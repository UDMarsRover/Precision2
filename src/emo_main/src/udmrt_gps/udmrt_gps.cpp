#include "udmrt_gps.h"

UDMRT_GPS::UDMRT_GPS(char* name, NodeHandle* node): 
  UDMRT_Sensor<sensor_msgs::msg::NavSatFix>(name, node)
{
  UDMRT_GPS::gpsConnected = false;
  UDMRT_GPS::gpsError = false;
}

bool UDMRT_GPS::init(Publisher* dataPublisher, Publisher* diagnosticPublisher){
  
  Serial1.begin(9600);

  int count = 50;
  while (!Serial1 && (count > 0)){
    count++;
    delay(100);
  }
  
  UDMRT_GPS::UDMRT_Sensor::init(dataPublisher, diagnosticPublisher);
  
  return Serial1;
}

void UDMRT_GPS::spin(){
  updateData();
  UDMRT_Sensor::spin();
}

void UDMRT_GPS::updateData(){

  bool readReady = false;
  bool connectedToSatellites = false;

  while (Serial1.available() > 0)
  {
    readReady = gps.encode(Serial1.read());
    
    connectedToSatellites = !((gps.location.lat() == 0 && gps.location.lng() == 0) || (gps.location.age() > 500));
  
    if (readReady) {
      okState();

      node->loginfo("serial connect");
    
      if (connectedToSatellites){
        data_msg.header.stamp.sec = gps.time.second();
        data_msg.header.stamp.nanosec = gps.time.centisecond() * 10000000;
        data_msg.latitude = gps.location.lat();
        data_msg.longitude = gps.location.lng();
        data_msg.altitude = gps.altitude.meters();
        data_msg.status.status = 15;
        gpsError = false;

        node->loginfo("sat connect");
      } else{
        warningState();
        node->loginfo("no sat");
      }

      lastTimeStamp = millis();
    } 
  }

  if (1050 < (millis() - lastTimeStamp)){
    data_msg.status.status = 14;
    data_msg.latitude = 0;
    data_msg.longitude = 0;
    errorState();
    node->loginfo("no serial");
  }
}

void UDMRT_GPS::errorState(){
  diag_msg.message = "Unable to start serial connection with GPS sensor!!";
  diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  gpsError = true;
  errorCode.key = "2";
  errorCode.value = "Unable to communicate with sensor";
}

void UDMRT_GPS::warningState(){
  diag_msg.message = "Unable to see satellites. Location data may be inaccurate.";
  diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  gpsError = true;
  errorCode.key = "1";
  errorCode.value = "Unable to communicate with satellites";
}

void UDMRT_GPS::okState(){
  diag_msg.message = "GPS connected to serial and satellites.";
  diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  gpsError = false;
  errorCode.key = "0";
  errorCode.value = "GPS Connected";
}