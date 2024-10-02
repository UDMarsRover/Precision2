/*
 * MoogMotor.cpp - Library for controlling Moog Smart Motors via HardwareSerial.
 * Created by Joe Ditz on Jan 27, 2021
 * Edited by Greg Moldkow Mar 27, 2021
 */
#include "MoogMotor.h"

float rps;
float eCounts;

MoogMotor::MoogMotor(int id, 
                     HardwareSerial* serial, 
                     int gearRatio, 
                     float wheelDiameter, 
                     int resolution, 
                     int samplerate, 
                     int delayTime, 
                     float acc
                    )
{  
  MoogMotor::delayTime = delayTime;
  MoogMotor::gearRatio = gearRatio;
  MoogMotor::resolution = resolution;
  MoogMotor::sampleRate = samplerate;
  MoogMotor::serial = serial;
  MoogMotor::id = id;
  MoogMotor::acc = acc;
  MoogMotor::row = wheelDiameter * 3.14159 * 60 * 60;
  if(&(MoogMotor::serial)){
    MoogMotor::serial->end();
    delay(100);
    MoogMotor::serial->begin(9600);
    while(!&(MoogMotor::serial));
  }
  MoogMotor::enable();
  delay(100);
  MoogMotor::serial->end();
  delay(100);
}

MoogMotor::MoogMotor(){}

bool MoogMotor::setID(){

  if (MoogMotor::id != 0) {

    String setAddress = "SADDR" + String(id);
    sendCommand(setAddress,true);  // Set address of motor
    delay(MoogMotor::delayTime);

    int setID = MoogMotor::getData("RADDR");
    bool complete = (setID == MoogMotor::id);

    if (!complete && (MoogMotor::setIDTrys <= MoogMotor::setIDMax)) {
      MoogMotor::setIDTrys += 1;
      delay(MoogMotor::delayTime);
      MoogMotor::setID();
    }
    return complete;
  }
  return true;
}

void MoogMotor::enable(){
  MoogMotor::sendCommand("WAKE",true);
  MoogMotor::sendCommand("ECHO",true);

  //Remove Software Limits
  MoogMotor::sendCommand("EIGN(2)",true);
  MoogMotor::sendCommand("EIGN(3)",true);
  MoogMotor::sendCommand("SLD",true);
  MoogMotor::sendCommand("ZS",true);
  MoogMotor::sendCommand("MDT");
  MoogMotor::sendCommand("MDB");
  MoogMotor::sendCommand("BRKSRV");   // Engage Break When not Moving
  MoogMotor::sendCommand("G");        // Go Command to turn on coils
  MoogMotor::sendCommand("X");        // Send Stop Request'
  MoogMotor::sendCommand("AT="+String(MoogMotor::acc * ACCMAX));      //Set the acceration/deceleration
  MoogMotor::sendCommand("BAUD115200");
}

bool MoogMotor::sendCommand(String command, bool global){
  MoogMotor::writeToPort(command);
}

void MoogMotor::writeToPort(String data){
  // Select the proper output channel of the mux IF it is not the "gobal" motor -1
  if (MoogMotor::id != -1) MoogMotor::openPort();
  //delay(MoogMotor::delayTime/2);
  Serial1.print(data + " ");
  delay(MoogMotor::delayTime);
  if (MoogMotor::id != -1) MoogMotor::closePort();
  //delay(MoogMotor::delayTime/2);
}

void MoogMotor::openPort(){
  digitalWrite(MoogMotor::id, HIGH);
}

void MoogMotor::closePort(){
  digitalWrite(MoogMotor::id, LOW);
}

void MoogMotor::statusCheck(){
  statusCode = getData("RW(0)");
  statusCode1 = getData("RW(1)");
  statusCode2 = getData("RW(2)");
  statusCode3 = getData("RW(3)");
  statusCode4 = getData("RW(4)");
  statusCode5 = getData("RW(5)");
  statusCode6 = getData("RW(6)");
  statusCode7 = getData("RW(7)");
}

int MoogMotor::getData(char command[]){
  MoogMotor::sendCommand(command);
  if (MoogMotor::serial->available()) MoogMotor::connected = true;
  else MoogMotor::connected = false;
  int data = MoogMotor::serial->parseInt();
  return data;
}

unsigned int MoogMotor::getStatusCode(){
  return MoogMotor::statusCode;
}

bool MoogMotor::isConnected(){
  return MoogMotor::connected;
}

bool MoogMotor::resetStatusCodes(){
  sendCommand("ZS");       // Clear All Warning Tags
  MoogMotor::enable();
  delay(100);
  //park();
  return statusCode == 1;
}

void MoogMotor::stop()
{
  sendCommand("X"); //Standard stop command
}

/* ESHUTDOWN
 * This command will imediatly kill power to the motor coils AND throw an overcurrent and positon
 * error to prevent further use. Use this function in case of an ESTOP button press only. This 
 * command WILL NOT ENGAGE BREAKING. To do a hard stop with breaking, call the stop() function.
 */
void MoogMotor::ESTOP(){
  sendCommand("S");    // Hard Stop
  sendCommand("OFF");  // Turn Coils Off
  sendCommand("B(0,0)=0");
}

boolean MoogMotor::setVelocity(float kmph){

  rps = kmph / MoogMotor::row;
  eCounts = rps * MoogMotor::gearRatio * (MoogMotor::resolution/MoogMotor::sampleRate) * 65536;  

  MoogMotor::sendCommand("MV VT="+String(eCounts)+" "+"G");       //Set motors to velocity mode (MV), the rps (VT=), and tell it to go (G)

  return true;
}

boolean MoogMotor::setTorque(float torque){
  if(abs(torque) <= 1){
    torque = torque * TRQMAX;
    
    sendCommand("MT");     //Set motor to torque mode
    sendCommand("T=" + String(torque));        //Set the torque value
    sendCommand("G");     //Go command
    
    return true;
  }
  return false;
}

boolean MoogMotor::setPosition(float position, float velocity){
  
  if(abs(position) <= 1 && 0 <= MoogMotor::acc <=1){
    position = position * POSMAX;         
    float acceleration = MoogMotor::acc * ACCMAX; 
    
    
    sendCommand("MP");                          //Set to motor position mode
    sendCommand("VT=" + String(velocity));      //Set the rpm
    sendCommand("ADT=" + String(acceleration)); //Set the acceration/deceleration
    sendCommand("PT=" + String(position));      //Set the position
    sendCommand("G");                           //Go Command
    return true;                                //Retrun a true to confirm operation complete
  }
  return false;
}

void MoogMotor::holdOFF(){
  sendCommand("BRKRLS");   // Disengage breaking system
  sendCommand("OFF");       // Turn Coils Off
}

void MoogMotor::holdON(){
  sendCommand("BRKSRV");   // Engage Break When not Moving
  sendCommand("G");        // Go Command to turn on coils
  sendCommand("X");        // Send Stop Request
}

