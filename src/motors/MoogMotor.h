/*
 * MoogMotor.h - Library for controlling Moog Smart Motors via HardwareSerial.
 * Created by Joe Ditz on Jan 27, 2021
 * Edited by Greg Molskow on Mar 27, 2021
 */

#ifndef MoogMotor_h
#define MoogMotor_h

#include "Arduino.h"
#include "HardwareSerial.h"
//#include "SoftwareSerial.h"
#include "MoogMotorStatusWords.h"

#define ACCMAX 1000
#define RPMMAX 4423680
#define TRQMAX 32767
#define POSMAX 32768
#define REVSPERROTATION 4000
using namespace std;


class MoogMotor
 {
  public:
    MoogMotor();
    MoogMotor(int id, 
              HardwareSerial* serial, 
              int gearRatio, 
              float wheelDiameter, 
              int resolution=4000, 
              int samplerate=8000, 
              int delayTime=5, 
              float acc=0.25);

    
    /** @breif: Function to set the motors velocity
    @param kmph The desired speed in kilometers per hour
    @return Boolean value indicating the command has been sent
    This function takes in the desired kmph, converts it to 
    motor encoder counts, and sends the required commands
    */
    boolean setVelocity(float kmph);


    boolean setTorque(float torque);
    boolean setPosition(float value,float velocity);
    
    
    void ESTOP();
    void stop();
    void enable();
    void off();
    void holdON();
    void holdOFF();
    void statusCheck();
    bool isConnected();
    unsigned int getStatusCode();
    bool resetStatusCodes();
    int getData(char[]);
    bool sendCommand(String command, bool global = false);
    int id = -1;
    bool setID(); 
    void openPort();
    void closePort();

    
    
  private:
    unsigned int statusCode:17;
    unsigned int statusCode1:17;
    unsigned int statusCode2:17;
    unsigned int statusCode3:17;
    unsigned int statusCode4:17;
    unsigned int statusCode5:17;
    unsigned int statusCode6:17;
    unsigned int statusCode7:17;
    HardwareSerial* serial;
    bool connected;
    float gearRatio;
    float resolution = 0;
    float sampleRate = 0;
    int delayTime;   // Time to wait for motors to update in ms
    int setIDMax = 3;    // Number of times the software should try to set the ID on startup
    int setIDTrys = 0;    // The current number of ID sets tried
    void writeToPort(String data);
    float acc;
    /** @breif: The velocity to rotations multipyer*/
    float row;
     
 };


#endif
