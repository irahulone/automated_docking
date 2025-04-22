import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
class publisher(Node):

    def __init__(self):
        super().__init__('arduino_wraper_rover_i')
        self.serial = serial.Serial("/dev/ttyACM0", 9600, timeout=0.1)
        time.sleep(1)

        # create one reentrant group for all “to‐Arduino” callbacks
        self.to_ard_group = ReentrantCallbackGroup()
        # and one for “from‐Arduino” timers
        self.from_ard_group = ReentrantCallbackGroup()

        self.ros_publishers = ReentrantCallbackGroup()

        # subscriptions that send commands → group #1
        self.create_subscription(
            Bool, '/ri/chrg_ctrl', self.chrg_ctrl_callback,
            1, callback_group=self.to_ard_group)
        self.create_subscription(
            Bool, '/ri/valv_ctrl', self.valv_ctrl_callback,
            1, callback_group=self.to_ard_group)
        self.create_subscription(
            Bool, '/ri/pump_ctrl', self.pump_ctrl_callback,
            1, callback_group=self.to_ard_group)

        # timer that reads Serial → group #2
        self.create_timer(
            0.1, self.timer_callback,
            callback_group=self.from_ard_group)

        self.pub_b = self.create_publisher(Bool, '/ri/liquid_lvl', 1, callback_group=self.ros_publishers)
        self.pub_a = self.create_publisher(Bool, '/ri/chrg_align', 1, callback_group=self.ros_publishers)


        self.state = 0x00
    #def listener_callback(self, msg):
    #    val = msg.data
    #    payload_send = str(int(val)) + ","
    #    self.serial.write(str.encode(payload_send))
    #    #print(payload_send)

    def state_callback(self, shift: int, msg: bool):
        if(not((self.state>>shift&1)^(1 if msg else 0))):
            return
        self.state ^= 1 << shift
        self.serial.write(bytes([self.state]))
        self.get_logger().info(f"Sent 0b{self.state:03b}")
           
    def chrg_ctrl_callback(self, msg: Bool):
        self.state_callback(0, msg.data)
    def valv_ctrl_callback(self, msg: Bool):
        self.state_callback(1, msg.data)
    def pump_ctrl_callback(self, msg: Bool):
        self.state_callback(2, msg.data)


    def timer_callback(self):
        # check if there's at least one full line waiting
        if self.serial.in_waiting and b'\n' in self.serial.read(self.serial.in_waiting):
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return
            print("Arduino: ", line)
        # otherwise just return—don’t block


    
    def exit_node(self):
        self.serial.write(bytes([0x00]))

def main(args=None):
    rclpy.init(args=args)
    node = publisher()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


# ros2 topic pub /cs/chrg_ctrl std_msgs/Int16 "data: 1"
# ros2 topic pub /fcmc/chrg_req std_msgs/Bool "data: False"

if __name__ == '__main__':
    main()
