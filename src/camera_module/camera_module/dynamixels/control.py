# import sys, tty, termios
from dynamixel_sdk import * # Uses Dynamixel SDK library

MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_VELOCITY          = 104
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
OPERATING_MODE              = 11
VELOCITY_MODE               = 1
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold
PROTOCOL_VERSION            = 2.0


class DynamixelMX:
    def __init__(self, port, ID, baud):
        self.port = port
        self.ID = ID
        self.baud = baud
        # self.fd = sys.stdin.fileno()
        # self.old_settings = termios.tcgetattr(self.fd)
        self.baudrate = baud
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.open()
        self.set_baudrate()
        
        # self.set_velocity_mode()
        self.set_torque_enable() # Will lock eeprom
        self.index = 0

    def open(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

    def set_baudrate(self):
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")

    def read_present_position(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_position
    
    # Position is in range 0-4095
    def write_goal_position(self, goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def write_goal_velocity(self, goal_velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_GOAL_VELOCITY, goal_velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_torque_enable(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def set_velocity_mode(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, OPERATING_MODE, VELOCITY_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")


    # def getch(self):
    #     try:
    #         tty.setraw(sys.stdin.fileno())
    #         ch = sys.stdin.read(1)
    #     finally:
    #         termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    #     return ch
    
    
    

