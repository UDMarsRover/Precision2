//#include "HardwareSerial.h"
//#include "SoftwareSerial.h"
//#include <ros.h>
//#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include "rclcpp/rclcpp.hpp"
#include "UDMRTDrivetrain.h"
#include "MoogMotor.h"

#define WHEELRADIUS 0.00025 // In KM
#define RED 22     
#define BLUE 24     
#define GREEN 23

#define CH1 2
#define CH2 3
#define CH3 4
#define CH4 5
#define CH5 6
#define CH6 7

class DrivetrainNode : public rclcpp::Node {
public:
    DrivetrainNode()
    : Node("drivetrain_node"),
      driveTrain(UDMRTDrivetrain()) {

        // Initialize motors
        initMotors();

        // Initialize publishers and subscribers
        drive_status_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DriveStatus", 10);
        drive_gear_publisher_ = this->create_publisher<std_msgs::msg::String>("DriveGear", 10);
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("DriveVelocity", 10, std::bind(&DrivetrainNode::runTankDrive, this, std::placeholders::_1));

        // Initialize LED pins
        pinMode(RED, OUTPUT);
        pinMode(BLUE, OUTPUT);
        pinMode(GREEN, OUTPUT);
        // Initialize status
        currentDriveStatus_.name = "Drivetrain Motors Status";
        currentDriveStatus_.hardware_id = "Arduino Nano - Drivetrain";
    }

private:
    void initMotors() {
        // Initialize left and right motors
        std::vector<MoogMotor> leftMotors = {
            MoogMotor(CH5, &Serial1, 28, WHEELRADIUS, 4000, 8000, 5, acceleration * accRatio),
            MoogMotor(CH6, &Serial1, 40, WHEELRADIUS, 4000, 8000, 5, acceleration),
            MoogMotor(CH4, &Serial1, 40, WHEELRADIUS, 4000, 8000, 5, acceleration)
        };

        std::vector<MoogMotor> rightMotors = {
            MoogMotor(CH3, &Serial1, 40, WHEELRADIUS, 4000, 8000, 5, acceleration),
            MoogMotor(CH1, &Serial1, 40, WHEELRADIUS, 4000, 8000, 5, acceleration),
            MoogMotor(CH2, &Serial1, 28, WHEELRADIUS, 4000, 8000, 5, acceleration * accRatio)
        };

        driveTrain = UDMRTDrivetrain(leftMotors, rightMotors, lengths, 0.00065855, 5);
        Serial1.begin(115200);
        while(!Serial1);
    }

    void runTankDrive(const geometry_msgs::msg::Twist::SharedPtr command) {
        float kmh_prec = command->linear.y;    // km/h in x
        float dps_prec = command->angular.z;   // km/h in y
        float reset = command->angular.x;      // Current indicator for motor reset

        if(reset != 0){
            driveTrain.reset();
            RCLCPP_INFO(this->get_logger(), "RESETTING Tank Drive ...");
            currentDriveStatus_.level = 1;
            currentDriveStatus_.message = "Restarting Drive Train";
        } else {
            if (driveTrain.drive(kmh_prec, dps_prec, acceleration)) {
                RCLCPP_INFO(this->get_logger(), "Running Tank Drive ...");
                currentDriveStatus_.level = 0;
                currentDriveStatus_.message = "Running";
            } else {
                RCLCPP_ERROR(this->get_logger(), "!!! Tank Drive Error !!!");
                currentDriveStatus_.level = 2;
                currentDriveStatus_.message = "Error Communicating With Motors!";
            }
        }
        // Update LED status based on drive status
        digitalWrite(BLUE, LOW);
        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, LOW);
    }

    void publishStatus() {
        drive_status_publisher_->publish(currentDriveStatus_);
    }

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr drive_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drive_gear_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    diagnostic_msgs::msg::DiagnosticStatus currentDriveStatus_;
    UDMRTDrivetrain driveTrain;
    const std::vector<float> lengths = { 0.00064536, 0.00049, 0.00065855, 0.00064536, 0.00049, 0.00065855 };
    float acceleration = 0.25;
    float accRatio = 28.0 / 40.0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivetrainNode>());
    rclcpp::shutdown();
    return 0;
}
