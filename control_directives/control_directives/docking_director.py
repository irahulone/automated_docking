import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from std_msgs.msg import String

class publisher(Node):

    def __init__(self):
        super().__init__('fluid_charge_management_controller')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.pub_z_id = self.create_publisher(Int16, '/ri/zone_id', 2)
        self.pub_z21_req = self.create_publisher(Bool, '/ri/zone21_req', 2)
        self.pub_z10_req = self.create_publisher(Bool, '/ri/zone10_req', 2)

        self.pub_chrg_req = self.create_publisher(Bool, '/fcmc/chrg_req', 2)
        self.pub_flow_req = self.create_publisher(Bool, '/fcmc/flow_req', 2)
        
        self.sub_z21_stats = self.create_subscription(String, '/ri/zone21_stats', self.zone21_stats_callback, 1)
        self.sub_z10_stats = self.create_subscription(String, '/ri/zone10_stats', self.zone10_stats_callback, 1)

        self.sub_z21_stats  # prevent unused variable warning
        self.sub_z10_stats  # prevent unused variable warning

        self.zone_current = 2

    def zone21_stats_callback(self, msg):
        k = msg.data
        if k == "DONE":
            if self.zone_current == 2 and self.zone_current != 1:
                self.zone_current = 1   # change zone to 1
                time.sleep(2)

    def zone10_stats_callback(self, msg):
        k = msg.data
        if k == "DONE":
            if self.zone_current == 1 and self.zone_current != 0:
                self.zone_current = 0
                time.sleep(2)

        
    def timer_callback(self):

        msg_z21 = Bool()
        if self.zone_current == 2:
            msg_z21.data =  True
        else:
            msg_z21.data =  False

        msg_z10 = Bool()
        if self.zone_current == 1:
            msg_z10.data =  True
        else:
            msg_z10.data =  False

        msg_chrg = Bool()
        msg_flow = Bool()
        
        if self.zone_current == 0:
            msg_chrg.data =  True
            msg_flow.data =  True
            self.pub_chrg_req.publish(msg_chrg)
            self.pub_flow_req.publish(msg_flow)
        else:   
            msg_chrg.data =  False
            msg_flow.data =  False
            self.pub_chrg_req.publish(msg_chrg)
            self.pub_flow_req.publish(msg_flow)

        msg_z_id = Int16()
        msg_z_id.data =  self.zone_current

        self.pub_z_id.publish(msg_z_id)
        self.pub_z21_req.publish(msg_z21)
        self.pub_z10_req.publish(msg_z10)

     
    def exit_node(self):
        k = 0

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = publisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()

    minimal_publisher.exit_node()   # set gpios to 0
    
    rclpy.shutdown()

# ros2 topic pub /cs/chrg_ctrl std_msgs/Int16 "data: 1"
# ros2 topic pub /fcmc/chrg_req std_msgs/Bool "data: False"

if __name__ == '__main__':
    main()
