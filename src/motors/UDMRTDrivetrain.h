#ifndef UDMRTDrivetrain_h
#define UDMRTDrivetrain_h

#include "MoogMotor.h"
#include <vector>

#define PARK 0
#define DRIVE 1
#define NEUTRAL 2





class UDMRTDrivetrain{

  public:
    UDMRTDrivetrain(std::vector<MoogMotor> leftMotors, 
                    std::vector<MoogMotor> rightMotors, 
                    const std::vector<float> lengths, 
                    float max_length=0.00065855, 
                    float max_lin_vel=1);
  
    UDMRTDrivetrain();
    void ESTOP();
    void stop();
    void off();
    void setGear(int gear = PARK);
    bool drive(float kmPerHour_prec, float degPerSecond_prec, float acc);
    void reset();


  private:
    std::vector<MoogMotor> leftMotors;
    std::vector<MoogMotor> rightMotors;
    bool sendCommand(String command, int id=0);
    bool rosSetup;
    int numberOfMotors;
    float roverWidth;
    float tireDiameter;
    float max_lin_vel;
    float max_ang_vel;

    std::vector<float> lengths;

};

#endif