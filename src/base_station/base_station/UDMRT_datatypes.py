#!/usr/bin/env python3

# Composes and decomposes the messageOuts sent between rover and base
from inputs import get_gamepad
import _thread
import numpy as np

class LogitechF310:
    def __init__(self):
        self.left_joy_x = 0.0
        self.left_joy_y = 0.0
        self.right_joy_x = 0.0
        self.right_joy_y = 0.0
        self.dpad_x = 0.0
        self.dpad_y = 0.0
        self.back = 0
        self.start = 0
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0
        self.rb = 0
        self.lb = 0
        self.rt = 0
        self.lt = 0

        try:
            print("Starting Controller Interface Thread...")
            _thread.start_new_thread(self.update,("Hi", ))
            print("Controller Interface Thread Started!")
        except Exception as e:
            print("ERROR - Unable to start controller thread")
            print(e)
            exit()

    def update(self,__):
        while True:
            #print("Thread Updating..")
            event = get_gamepad()[0]
            if event.code == "BTN_THUMB":
                self.a = event.state
            elif event.code == "BTN_THUMB2":
                self.b = event.state
            elif event.code == "BTN_TRIGGER":
                self.x = event.state
            elif event.code == "BTN_TOP":
                self.y = event.state
            elif event.code == "ABS_RZ":
                self.left_joy_y = round((int(event.state) - 128) / -128, 1)
            elif event.code == "ABS_Z":
                self.left_joy_x = round((int(event.state) - 128) / 128, 1)
            elif event.code == "ABS_Y":
                self.right_joy_y = round((int(event.state) - 128) / -128, 1)
            elif event.code == "ABS_X":
                self.right_joy_x = round((int(event.state) - 128) / 128, 1)
            elif event.code == "ABS_HAT0X":
                self.dpad_x = event.state
            elif event.code == "ABS_HAT0Y":
                self.dpad_y = int(event.state) * -1
            elif event.code == "BTN_BASE4":
                self.start = event.state
            elif event.code == "BTN_BASE3":
                self.back = event.state
            elif event.code == "BTN_PINKIE":
                self.rb = event.state
            elif event.code == "BTN_BASE2":
                self.rt = event.state
            elif event.code == "BTN_TOP2":
                self.lb = event.state
            elif event.code == "BTN_BASE":
                self.lt = event.state
            else:
                None


class Arm_Position:
    def __init__(self):

        # Gripper Orientation in [meter,meter,meter,rad,rad,rad]
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.grip = False
        self.theta_xy = 0.0 # in rad
        self.theta_xz = 0.0 # in rad
        self.vector_mag_xy = 0.0
        self.vector_mag_xz = 0.0

        # motor position in radiants
        self.m0 = 0.0
        self.m1 = 0.0
        self.m2 = 0.0
        self.m3 = 0.0
        self.m4 = 0.0
        self.m5 = 0.0

    

    def getMotors(self):
        """
        This function returns a list of the saved motor position values
        :return: A list of the last recorded motor values for this object
        :rtype: (flaot)list[6]
        """
        return [self.m0,self.m1,self.m2,self.m3,self.m4,self.m5]
    
    def setMotors(self,state:list):
        """
        This function sets the motor position values

        This function takes in a list of 6 elements where each element corresponds to the position of each motor in radiants. Motor 0 is the first element and Motor 5 is the last element. 

        :param state: This is the state of the motors to be saved
        :type state: (float)list[6]
        :rtype: None
        """
        self.m0 = state[0]
        self.m1 = state[1]
        self.m2 = state[2]
        self.m3 = state[3]
        self.m4 = state[4]
        self.m5 = state[5]
    
    def nonZeroMotors(self):
        """
        This function checks to see if the values of the motor buffer are zero or not. This function is intended to be used with the jog functionality of the arm.

        :return: A bool value indicating if all the motor values are zero or not
        :rtype: Bool
        """
        return ((self.m0 != 0.0) or
                (self.m1 != 0.0) or
                (self.m2 != 0.0) or
                (self.m3 != 0.0) or
                (self.m4 != 0.0) or
                (self.m5 != 0.0))
    
    def updateMotors(self,state:list):
        """
        This function updates the motor values by adding to the prior value. This function is meant to be used with the jog functionality of the arm.

        :param state: The values to be added to the motor buffer
        :type state: (float)list[6]
        :return: Returns a list of the updated motor values
        :rtype: (float)list[6]
        """
        self.setMotors(list(sum(i) for i in zip(state, self.getMotors())))
        return self.getMotors()

    def getPosition(self):
        """
        This function returns the saved position values for the end effector

        :return: The current orientation of the end effector in meters and rads [x,y,z,roll,pitch,yaw]
        :rtype: (float)list[6]
        """
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
    
    def nonZeroPosition(self):
        """
        This function checks to see if all of the orientation values are 0

        :returns: A bool indicating if all the orientation values are zero
        :rtype: Bool
        """
        return ((self.x != 0.0) or
                (self.y != 0.0) or
                (self.z != 0.0) or
                (self.roll != 0.0) or
                (self.pitch != 0.0) or
                (self.yaw != 0.0))
    
    def setPosition(self,state:list):
        """
        This function sets the orientation of the end effector based in the input values

        :param state: The new values to set
        :type state: (float)list[6]
        """
        self.x = state[0]
        self.y = state[1]
        self.z = state[2]
        self.roll = state[3]
        self.pitch = state[4]
        self.yaw = state[5]

    
        self.theta_xy = np.arctan2(self.y,self.x) if self.x is not 0 or self.y is not 0 else 0
        self.theta_xz = np.arctan2(self.z,self.x) if self.x is not 0 or self.z is not 0 else 0
        self.vector_mag_xy = np.sqrt((self.x ** 2) + (self.y ** 2))
        self.vector_mag_xy = np.sqrt((self.x ** 2) + (self.z ** 2))

    def updatePosition(self,state:list):
        """
        This function updates the orientation values by adding the input values to the saved values

        :param state: A list of the 6 orientation values that are to be added to the current values saved in the buffer
        :type state: (float)list[6]
        :return: The updated orientation values
        :rtype: (float)list[6]
        """
        x = state[0]
        y = state[1]
        z = state[2]
        roll = state[3]
        pitch = state[4]
        yaw = state[5]


        if np.abs(x) > 0 or np.abs(y) > 0:
            new_mag_xy = self.vector_mag_xy + np.sqrt((x**2)+(y**2))
            x = np.cos(self.theta_xy) * new_mag_xy
            y = np.sin(self.theta_xy) * new_mag_xy
        elif z > 0:
            z += self.z

        roll += self.roll
        pitch += self.pitch
        yaw += self.yaw



        self.setPosition([x,y,z,roll,pitch,yaw])
        return self.getPosition()










class UDMRTDataBuffer:
    def __init__(self):
        # Variables
        self.__errorMessageData__ = ""
        self.__driveMotor1Data__ = ""
        self.__driveMotor2Data__ = ""
        self.__driveMotor3Data__ = ""
        self.__driveMotor4Data__ = ""
        self.__armMotor1Data__ = ""
        self.__armMotor2Data__ = ""
        self.__armMotor3Data__ = ""
        self.__armMotor4Data__ = ""
        self.__armMotor5Data__ = ""
        self.__armMotor6Data__ = ""
        self.__gyroscopeData__ = ""
        self.__boxTempData__ = ""
        self.__busMonitorData__ = ""
        self.__batteryTempData__ = ""
        self.__voltageConverterTempData__ = ""
        self.__currentConversionData__ = ""
        self.__ultrasonicSensor1Data__ = ""
        self.__ultrasonicSensor2Data__ = ""
        self.__ultrasonicSensor3Data__ = ""
        self.__ultrasonicSensor4Data__ = ""

        self.__driveMotorErrorData__ = ""
        self.__armMotorErrorData__ = ""
        self.__emoErrorData__ = ""
        self.__cameraData__ = ""

        self.__messageOut__ = ""

    # Getter and setter methods... lots of them...

    def setErrorMessageData(self):
        # Do some logic for determining error code

        strOut = "NULL"
        self.__errorMessageData__ = strOut

    def getErrorMessageData(self):
        return self.__errorMessageData__

    def getDriveMotor1Data(self):
        return self.__driveMotor1Data__

    def getDriveMotor2Data(self):
        return self.__driveMotor2Data__

    def getDriveMotor3Data(self):
        return self.__driveMotor3Data__

    def getDriveMotor4Data(self):
        return self.__driveMotor4Data__

    def getArmMotor1Data(self):
        return self.__armMotor1Data__

    def getArmMotor2Data(self):
        return self.__armMotor2Data__

    def getArmMotor3Data(self):
        return self.__armMotor3Data__

    def getArmMotor4Data(self):
        return self.__armMotor4Data__

    def getArmMotor5Data(self):
        return self.__armMotor5Data__

    def getArmMotor6Data(self):
        return self.__armMotor6Data__

    def getGyroscopeData(self):
        return self.__gyroscopeData__

    def getBoxTempData(self):
        return self.__boxTempData__

    def getBusMonitorData(self):
        return self.__busMonitorData__

    def getBatteryTempData(self):
        return self.__batteryTempData__

    def getVoltageConverterTempData(self):
        return self.__voltageConverterTempData__

    def getCurrentConversionData(self):
        return self.__currentConversionData__

    def getUltrasonicSensor1Data(self):
        return self.__ultrasonicSensor1Data__

    def getUltrasonicSensor2Data(self):
        return self.__ultrasonicSensor2Data__

    def getUltrasonicSensor3Data(self):
        return self.__ultrasonicSensor3Data__

    def getUltrasonicSensor4Data(self):
        return self.__ultrasonicSensor4Data__

    def setDriveMotorData(self, dataIn):
        str = dataIn.data
        self.__driveMotorErrorData__ = str[0:2]
        self.__driveMotor1Data__ = str[2:5]
        self.__driveMotor2Data__ = str[5:8]
        self.__driveMotor3Data__ = str[8:11]
        self.__driveMotor4Data__ = str[11:14]

    def setArmMotorData(self, dataIn):
        str = dataIn.data
        self.__armMotorErrorData__ = str[0:2]
        self.__armMotor1Data__ = str[2:7]
        self.__armMotor2Data__ = str[7:12]
        self.__armMotor3Data__ = str[12:17]
        self.__armMotor4Data__ = str[17:22]
        self.__armMotor5Data__ = str[22:27]
        self.__armMotor6Data__ = str[27:32]

    def setEmoData(self, dataIn):
        str = dataIn.data
        self.__emoErrorData__ = str[0:2]
        self.__gyroscopeData__ = str[2:5]
        self.__boxTempData__ = str[5:8]
        self.__busMonitorData__ = str[8:12]
        self.__batteryTempData__ = str[12:15]
        self.__voltageConverterTempData__ = str[15:18]
        self.__currentConversionData__ = str[18:22]
        self.__ultrasonicSensor1Data__ = str[22:26]
        self.__ultrasonicSensor2Data__ = str[26:30]
        self.__ultrasonicSensor3Data__ = str[30:34]
        self.__ultrasonicSensor4Data__ = str[34:38]

    # def setCameraData(self, dataIn):
    #    self.__cameraData__ = dataIn.data

    def composeMessageOut(self):
        # Concatenate all of the variables together into one __messageOut__
        self.__messageOut__ += self.getErrorMessageData()
        self.__messageOut__ += self.getDriveMotor1Data()
        self.__messageOut__ += self.getDriveMotor2Data()
        self.__messageOut__ += self.getDriveMotor3Data()
        self.__messageOut__ += self.getDriveMotor4Data()
        self.__messageOut__ += self.getArmMotor1Data()
        self.__messageOut__ += self.getArmMotor2Data()
        self.__messageOut__ += self.getArmMotor3Data()
        self.__messageOut__ += self.getArmMotor4Data()
        self.__messageOut__ += self.getArmMotor5Data()
        self.__messageOut__ += self.getArmMotor6Data()
        self.__messageOut__ += self.getGyroscopeData()
        self.__messageOut__ += self.getBoxTempData()
        self.__messageOut__ += self.getBusMonitorData()
        self.__messageOut__ += self.getBatteryTempData()
        self.__messageOut__ += self.getVoltageConverterTempData()
        self.__messageOut__ += self.getCurrentConversionData()
        self.__messageOut__ += self.getUltrasonicSensor1Data()
        self.__messageOut__ += self.getUltrasonicSensor2Data()
        self.__messageOut__ += self.getUltrasonicSensor3Data()
        self.__messageOut__ += self.getUltrasonicSensor4Data()

        return self.__messageOut__

    # Checking for any errors
    def checkForError(self):
        # If true, no error. If false, there is an error.
        return self.__emoErrorData__.lower() == "ff"
