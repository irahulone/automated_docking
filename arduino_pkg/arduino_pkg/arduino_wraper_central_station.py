import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import serial
import time

class publisher(Node):

    def __init__(self):
        super().__init__('arduino_wraper_central_station')
        self.serial = serial.Serial("/dev/ttyACM0", 9600)
        time.sleep(1)

        self.pub_a = self.create_publisher(Int16, '/cs/value_a', 10)
        self.pub_b = self.create_publisher(Int16, '/cs/chrg_current', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.subscription = self.create_subscription(Int16, 'write', self.listener_callback, 10)
        self.sub_chrg_ctrl = self.create_subscription(Int16, '/cs/chrg_ctrl', self.chrg_ctrl_callback, 1)
        self.sub_valv_ctrl = self.create_subscription(Int16, '/cs/valv_ctrl', self.valv_ctrl_callback, 1)
        self.sub_pump_ctrl = self.create_subscription(Int16, '/cs/pump_ctrl', self.pump_ctrl_callback, 1)
        
        self.sub_chrg_ctrl  # prevent unused variable warning
        self.sub_valv_ctrl  # prevent unused variable warning
        self.sub_pump_ctrl  # prevent unused variable warning

        self.chrg_state = 0
        self.valv_state = 0
        self.pump_state = 0

    def chrg_ctrl_callback(self, msg):
        val = int(msg.data)
        if val >= 0 or val <= 9:
            self.chrg_state = val
        
    def valv_ctrl_callback(self, msg):
        val = int(msg.data)
        if val >= 0 or val <= 9:
            self.valv_state = val

    def pump_ctrl_callback(self, msg):
        val = int(msg.data)
        if val >= 0 or val <= 9:
            self.pump_state = val

    def timer_callback(self):
        payload_rec = self.serial.readline()
        a = int(chr(payload_rec[0]))
        b1 = payload_rec[2]; b2 = payload_rec[3]; b3 = payload_rec[4]
        b = int(chr(b1) + chr(b2) + chr(b3))
        #print(a)
        msg_a = Int16()
        msg_b = Int16()
        msg_a.data =  a
        msg_b.data =  b
        self.pub_a.publish(msg_a)
        self.pub_b.publish(msg_b)

        payload_tx = str(int(self.chrg_state)) + "," + str(int(self.valv_state)) + "," +str(int(self.pump_state)) + ","
        self.serial.write(str.encode(payload_tx))
        print(payload_tx)
    
    def exit_node(self):
        payload_tx = str(int(0)) + "," + str(int(0)) + "," +str(int(0)) + ","
        self.serial.write(str.encode(payload_tx))

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = publisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()

    minimal_publisher.exit_node()   # set gpios to 0
    
    rclpy.shutdown()

# ros2 topic pub /cs/chrg_ctrl std_msgs/Int16 "data: 1"

if __name__ == '__main__':
    main()
