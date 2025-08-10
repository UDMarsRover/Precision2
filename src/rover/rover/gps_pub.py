import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class GPS_Publisher(Node):

    def __init__(self, serial):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(String, 'gps_pub', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.serial = serial

    def timer_callback(self):
        msg = String()
        waiting = False
        lat, lon = None, None

        line = self.serial.readline()
        line_d = line.decode('ascii')
        try:
            lat, lon, geiger = line_d.split(",")
        except:
            pass
       
        if lat is not None and lon is not None:
            msg.data = str(lat) + ", " + str(lon), + ". Geiger: " + str(geiger)
            self.publisher_.publish(msg)
            self.get_logger().info('Lat, long: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
   
    ser = serial.Serial()

    ser.port = '/dev/ttyACM1'
    ser.baudrate = 9600
    ser.open()
   
    gps_pub = GPS_Publisher(ser)
    try:
        rclpy.spin(gps_pub)
    except Exception as e:
        gps_pub.get_logger().error(f"Error in GPS Publisher: {e}")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

