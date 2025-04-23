#!/usr/bin/env python3
import pygame
from pygame.locals import *

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

pygame.init()

AXIS_CENTER = 0.004 # axis centers at 0.0039

joystick = pygame.joystick.Joystick(0)

class ControllerPublisher(Node):

    def __init__(self):
        super().__init__('controller_publisher')
        self.arm_publisher_ = self.create_publisher(Float32MultiArray, 'arm_controller', 10)
        self.rover_publisher_ = self.create_publisher(Float32MultiArray, 'rover_controller', 10)
        self.publish_rover = True
    

        self.msg = Float32MultiArray()
        self.no_msg = Float32MultiArray()
        self.no_msg.data = [0.0] * 15

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.get_input)

        self.button_list = [0] * 12

        self.x_btn = False
        self.a_btn = False
        self.b_btn = False
        self.y_btn = False
        self.lb_btn = False
        self.rb_btn = False
        self.lt_btn = False
        self.rt_btn = False
        self.back_btn = False
        self.start_btn = False
        self.left_stick_btn =False
        self.right_stick_btn = False
        self.back_btn = False

    def get_input(self):
        for event in pygame.event.get():
            if (event.type == pygame.JOYBUTTONDOWN):
                self.button_list[event.button] = joystick.get_button(event.button)
            if (event.type == pygame.JOYBUTTONUP):
                self.button_list[event.button] = joystick.get_button(event.button)

        self.x_btn = self.button_list[0] # x
        self.a_btn = self.button_list[1]# a
        self.b_btn = self.button_list[2]# b
        self.y_btn = self.button_list[3] # y
        self.lb_btn = self.button_list[4] # lb
        self.rb_btn = self.button_list[5]# rb
        self.lt_btn = self.button_list[6] # lt
        self.rt_btn = self.button_list[7]# rt
        self.back_btn = self.button_list[8] # back
        self.start_btn = self.button_list[9]# start
        self.left_stick_btn = self.button_list[10] # left stick in
        self.right_stick_btn = self.button_list[11] # right stick in
    
        # left stick: left-right = axis 0
        #             up-down = axis 1
        # right stick: left-right = axis 3
        #               up-down = axis 4 
        if (abs(joystick.get_axis(0)) > AXIS_CENTER):
            axis_0 = joystick.get_axis(0)
        else:
            axis_0 = 0.0
        if (abs(joystick.get_axis(1)) > AXIS_CENTER):
            axis_1 = joystick.get_axis(1)
        else:
            axis_1 = 0.0
        if (abs(joystick.get_axis(2)) > AXIS_CENTER):
            axis_2 = joystick.get_axis(2)
        else:
            axis_2 = 0.0
        if (abs(joystick.get_axis(3)) > AXIS_CENTER):
            axis_3= joystick.get_axis(3)
        else:
            axis_3 = 0.0

        

        self.msg.data = [axis_0, axis_1, axis_2, axis_3, 
                         float(self.x_btn), float(self.a_btn), float(self.b_btn), float(self.y_btn), 
                         float(self.lb_btn), float(self.rb_btn), float(self.lt_btn), float(self.rt_btn),
                            float(self.start_btn), float(self.left_stick_btn), float(self.right_stick_btn)]

        if(self.back_btn and self.publish_rover): 
            self.publish_rover = False
            print('publishing arm')
        elif(self.back_btn and not self.publish_rover): 
            print('publishing rover')
            self.back_last = self.back_btn    
            self.publish_rover = True

        if(self.publish_rover):
            self.rover_publisher_.publish(self.msg)
            self.arm_publisher_.publish(self.no_msg)
        else:
            self.arm_publisher_.publish(self.msg)
            self.rover_publisher_.publish(self.no_msg)   

        # reset all buttons to regesiter only 1 press for rover
        if(self.publish_rover):
            for i in range(len(self.button_list)):
                self.button_list[i] = False
        else: # for arm continuously register button press
            self.button_list[8] = False # only publish 1 back button press


def main(args=None):
    rclpy.init(args=args)
    
    controller_publisher = ControllerPublisher()

    rclpy.spin(controller_publisher)

    controller_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()