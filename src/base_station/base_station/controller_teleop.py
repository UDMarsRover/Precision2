#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time
import numpy as np
import sys
sys.path.append("..")
from base_station.UDMRT_datatypes import Arm_Position, LogitechF310


class udmrtController(Node):
    def __init__(self):
        super().__init__('controller_teleop')
        # rospy.init_node("Controller_teleop", anonymous=True)

        # ros1 - self.drivePub = rospy.Publisher("DriveVelocity", Twist, queue_size=1)
        self.drivePub = self.create_publisher(Twist, "DriveVelocity",qos_profile_system_default)

        # self.armPosPub = rospy.Publisher("arm/cmd/position", Float32MultiArray, queue_size=1)
        self.armPosPub = self.create_publisher(Float32MultiArray,"arm/cmd/position",qos_profile_system_default)

        # self.armMotorPub = rospy.Publisher("arm/cmd/motors", Float32MultiArray, queue_size=1)
        self.armMotorPub = self.create_publisher(Float32MultiArray,"arm/cmd/motors",qos_profile_system_default)

        # self.armGripPub = rospy.Publisher("arm/cmd/grip", Bool, queue_size=1)
        self.armGripPub = self.create_publisher(Bool,"arm/cmd/grip",qos_profile_system_default)

        #changed ros to rclpy, may not work
        # self.rate = rclpy.Rate(60)
        # self.armRate = rclpy.Rate(5)

        self.jog_pose_value = 0.02  # meters
        self.arm_jog_count = 0
        self.arm_reset_count = 0
        self.arm_cmd = Arm_Position()
        self.current_arm_command = Float32MultiArray()
        self.current_arm_command.data = (
            self.arm_cmd.getPosition()
        )  # command to update and publish
        self.current_arm_motor_command = Float32MultiArray()
        self.current_arm_motor_command.data = (
            self.arm_cmd.getMotors()
        )  # command to update and publish

        self.grip_command = False
        #self.armPosPub.publish(self.current_arm_command)
        #self.armGripPub.publish(self.grip_command)
        

        self.linVelY = 0
        self.angVelZ = 0
        self.sec = time.time()
        self.velOut = Twist()
        self.velOut.linear.y = 0.0
        self.velOut.angular.z = 0.0
        self.velOut.angular.x = 0.0
        self.drivePub.publish(self.velOut)
        self.current_start_state = 0

        self.controller = LogitechF310()

        print("Controller TeleOp Started Successfully!")

        self.callback_timer = self.create_timer(1.0/10.0, self.spin_ros)


    def spin_ros(self):
        motorRunning = self.__motor_command_check__()


    def __motor_command_check__(self):
        """
        This function checks to see if the controller is trying to send a command to the motors. If a command is trying to be sent, this function packages and sends the command over ROS.

        :return: An indication if the motors have been commanded to run or not
        :rtype: bool
        """
        self.current_start_state = (
            self.controller.start
            if self.controller.start != self.current_start_state
            else 0
        )
        linVelY_temp = self.controller.left_joy_y
        angVelZ_temp = self.controller.left_joy_x

        linVelY_temp = linVelY_temp if np.abs(linVelY_temp) > 0.1 else 0
        angVelZ_temp = angVelZ_temp if np.abs(angVelZ_temp) > 0.1 else 0

        valueCheck = bool(
            (self.linVelY != linVelY_temp) or (angVelZ_temp != self.angVelZ) or (self.current_start_state)
        )
        self.linVelY = linVelY_temp
        self.angVelZ = angVelZ_temp

        print(valueCheck)

        if valueCheck:
            self.velOut.linear.y = float(self.linVelY)
            self.velOut.angular.z = float(self.angVelZ)
            self.velOut.angular.x = float(self.current_start_state)
            self.drivePub.publish(self.velOut)

        return (self.velOut.linear.y != 0) and (self.velOut.angular.z != 0)

    def __arm_command_check__(self):

        if self.controller.x: self.arm_jog_count += 1

        if self.arm_jog_count == 2:  # 1 lt press gets registered as 2
            self.jog_pose_value = 0.01
        elif self.arm_jog_count == 4:
            self.jog_pose_value = 0.005
        else:
            self.jog_pose_value = 0.02
            self.arm_jog_count = 0

        self.arm_cmd.x = self.jog_pose_value * self.controller.left_joy_y
        self.arm_cmd.y = self.jog_pose_value * self.controller.left_joy_x
        self.arm_cmd.z = self.jog_pose_value * self.controller.right_joy_y  # z
        #self.arm_cmd.roll = self.jog_pose_value * self.controller.rb  # roll
        #self.arm_cmd.roll += (
        #    self.jog_pose_value * self.controller.lb * -1
        #)  # negative roll

        #self.arm_cmd.pitch = self.jog_pose_value * self.controller.rt  # pitch
        #self.arm_cmd.pitch += (self.jog_pose_value * self.controller.lt * -1)  # negative pitch
        self.arm_cmd.yaw = self.jog_pose_value * self.controller.y  # yaw
        self.arm_cmd.yaw += self.jog_pose_value * self.controller.a * -1  # negative yaw

        if self.controller.rb and not self.controller.lb: self.arm_cmd.m0 = 1
        elif self.controller.lb and not self.controller.rb: self.arm_cmd.m0 = -1
        else: self.arm_cmd.m0 = 0


        self.grip_command = self.controller.b
        self.current_arm_command.data = self.arm_cmd.getPosition()
        self.current_arm_motor_command.data = self.arm_cmd.getMotors()

        if self.arm_cmd.nonZeroPosition():
            print("Publishing to Position:",self.current_arm_command)
            self.armPosPub.publish(self.current_arm_command)
        elif self.arm_cmd.nonZeroMotors():
            print("Publishing to Motors:",self.current_arm_motor_command)
            self.armMotorPub.publish(self.current_arm_motor_command)
        
        self.armGripPub.publish(self.grip_command)
        # self.armRate.sleep()

def main(args=None):
    rclpy.init(args=args)
    controller = udmrtController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
