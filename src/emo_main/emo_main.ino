#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include "src/udmrt_gps/udmrt_gps.h"
#include "src/udmrt_imu/udmrt_imu.h"
#include "src/udmrt_thermistor/udmrt_thermistor.h"
#include "src/udmrt_temperature/udmrt_temperature.h"
#include "src/udmrt_voltage_sensor/udmrt_voltage_sensor.h"
#include "src/udmrt_ultrasonic/udmrt_ultrasonic.h"
#include <NewPing.h>

class EmoNode : public rclcpp::Node {
public:
    EmoNode() : Node("emo_node") {
        // GPS Definitions
        gps = std::make_shared<UDMRT_GPS>("gps", this);
        gpsData = this->create_publisher<sensor_msgs::msg::NavSatFix>("/emo/gps", 10);
        gpsDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/gps", 10);

        // IMU Definitions
        imu = std::make_shared<UDMRT_IMU>("IMU", this, 30, 60, 30, 60);
        imuData = this->create_publisher<sensor_msgs::msg::Imu>("/emo/imu", 10);
        imuDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/imu", 10);

        // Battery Temperature Definitions
        batteryTemp = std::make_shared<UDMRT_Thermistor>("batteryTemperature", this, A1, 2000, 2010, 3965, 80, 0, 60, 30);
        batTempData = this->create_publisher<sensor_msgs::msg::Temperature>("/emo/batteryTemperature", 10);
        batTempDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/batteryTemperature", 10);

        // Box Temperature Definitions
        boxTemp = std::make_shared<UDMRT_Temperature>("boxTemperature", this);
        boxTempData = this->create_publisher<sensor_msgs::msg::Temperature>("/emo/boxTemperature", 10);
        botTempDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/boxTemperature", 10);

        // Voltage Converter Temperature Definitions
        voltageConverterTemp = std::make_shared<UDMRT_Thermistor>("voltageConverterTemperature", this, A2, 100000, 100100, 4615, 80, 0, 60, 30);
        voltTempData = this->create_publisher<sensor_msgs::msg::Temperature>("/emo/voltageConverterTemperature", 10);
        voltTempDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/voltageConverterTemperature", 10);

        // Voltage Sensor Definitions
        batteryVoltage = std::make_shared<UDMRT_Voltage_Sensor>("voltageSensor", this, A0, 45.5, 47.5);
        voltData = this->create_publisher<sensor_msgs::msg::BatteryState>("/emo/batteryVoltage", 10);
        voltDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/batteryVoltage", 10);

        // Ultrasonic Definitions
        ultraNE = std::make_shared<UDMRT_Ultrasonic>("ultraNE", this, TRIG, NE_ECHO);
        ne = std::make_shared<NewPing>(TRIG, NE_ECHO, 100);
        ultraNEData = this->create_publisher<std_msgs::msg::Float32>("/emo/ultraNE", 10);
        ultraNEDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/ultraNE", 10);

        ultraNW = std::make_shared<UDMRT_Ultrasonic>("ultraNW", this, TRIG, NW_ECHO);
        nw = std::make_shared<NewPing>(TRIG, NW_ECHO, 100);
        ultraNWData = this->create_publisher<std_msgs::msg::Float32>("/emo/ultraNW", 10);
        ultraNWDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/ultraNW", 10);

        ultraSE = std::make_shared<UDMRT_Ultrasonic>("ultraSE", this, TRIG, SE_ECHO);
        se = std::make_shared<NewPing>(TRIG, SE_ECHO, 100);
        ultraSEData = this->create_publisher<std_msgs::msg::Float32>("/emo/ultraSE", 10);
        ultraSEDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/ultraSE", 10);

        ultraSW = std::make_shared<UDMRT_Ultrasonic>("ultraSW", this, TRIG, SW_ECHO);
        sw = std::make_shared<NewPing>(TRIG, SW_ECHO, 100);
        ultraSWData = this->create_publisher<std_msgs::msg::Float32>("/emo/ultraSW", 10);
        ultraSWDiag = this->create_publisher<std_msgs::msg::String>("/emo/status/ultraSW", 10);

        // Initialize components
        imu->init(imuData, imuDiag);
        gps->init(gpsData, gpsDiag);
        batteryTemp->init(batTempData, batTempDiag);
        boxTemp->init(boxTempData, botTempDiag);
        voltageConverterTemp->init(voltTempData, voltTempDiag);
        batteryVoltage->init(voltData, voltDiag);
        ultraNE->init(ne, ultraNEData, ultraNEDiag);
        ultraNW->init(nw, ultraNWData, ultraNWDiag);
        ultraSE->init(se, ultraSEData, ultraSEDiag);
        ultraSW->init(sw, ultraSWData, ultraSWDiag);

        // Timer for loop function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&EmoNode::loop, this)
        );

        // Initialize RGB control
        pinMode(22, OUTPUT);
        pinMode(23, OUTPUT);
        pinMode(24, OUTPUT);
        rgbControl(1, 0, 0);
        delay(500);
        rgbControl(0, 1, 0);
        delay(500);
        rgbControl(0, 0, 1);
        delay(500);
    }

private:
    void loop() {
        rgbControl(0, 1, 0);
        rclcpp::spin_some(this->get_node_base_interface());
        gps->spin();
        imu->spin();
        batteryTemp->spin();
        boxTemp->spin();
        voltageConverterTemp->spin();
        batteryVoltage->spin();
        ultraNE->spin();
        ultraNW->spin();
        ultraSE->spin();
        ultraSW->spin();
        rgbControl(1, 0, 1);
    }

    void rgbControl(float red, float green, float blue) {
        analogWrite(22, (1023 - (1023 * red)));
        analogWrite(23, (1023 - (1023 * green)));
        analogWrite(24, (1023 - (1023 * blue)));
    }

    std::shared_ptr<UDMRT_GPS> gps;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gpsDiag;

    std::shared_ptr<UDMRT_IMU> imu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr imuDiag;

    std::shared_ptr<UDMRT_Thermistor> batteryTemp;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr batTempData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr batTempDiag;

    std::shared_ptr<UDMRT_Temperature> boxTemp;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr boxTempData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr botTempDiag;

    std::shared_ptr<UDMRT_Thermistor> voltageConverterTemp;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr voltTempData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voltTempDiag;

    std::shared_ptr<UDMRT_Voltage_Sensor> batteryVoltage;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr voltData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voltDiag;

    std::shared_ptr<UDMRT_Ultrasonic> ultraNE;
    std::shared_ptr<NewPing> ne;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultraNEData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ultraNEDiag;

    std::shared_ptr<UDMRT_Ultrasonic> ultraNW;
    std::shared_ptr<NewPing> nw;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultraNWData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ultraNWDiag;

    std::shared_ptr<UDMRT_Ultrasonic> ultraSE;
    std::shared_ptr<NewPing> se;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultraSEData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ultraSEDiag;

    std::shared_ptr<UDMRT_Ultrasonic> ultraSW;
    std::shared_ptr<NewPing> sw;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultraSWData;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ultraSWDiag;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmoNode>());
    rclcpp::shutdown();
    return 0;
}