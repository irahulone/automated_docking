import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from std_msgs.msg import String

class publisher(Node):

    def __init__(self):
        super().__init__('fluid_charge_management_controller')

        self.pub_chrg_stats = self.create_publisher(String, '/fcmc/chrg_stats', 10)
        self.pub_flow_stats = self.create_publisher(String, '/fcmc/flow_stats', 10)

        self.pub_cs_chrg = self.create_publisher(Int16, '/cs/chrg_ctrl', 1)
        self.pub_cs_valv = self.create_publisher(Int16, '/cs/valv_ctrl', 1)
        self.pub_cs_pump = self.create_publisher(Int16, '/cs/pump_ctrl', 1)
        self.pub_ri_chrg = self.create_publisher(Int16, '/ri/chrg_ctrl', 1)
        self.pub_ri_valv = self.create_publisher(Int16, '/ri/valv_ctrl', 1)
        self.pub_ri_pump = self.create_publisher(Int16, '/ri/pump_ctrl', 1)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sub_chrg_req = self.create_subscription(Bool, '/fcmc/chrg_req', self.chrg_req_callback, 1)
        self.sub_flow_req = self.create_subscription(Bool, '/fcmc/flow_req', self.flow_req_callback, 1)

        self.sub_chrg_align = self.create_subscription(Bool, '/ri/chrg_align', self.chrg_align_callback, 1)
        self.sub_liquid_ful = self.create_subscription(Bool, '/ri/liquid_ful', self.liquid_ful_callback, 1)

        self.sub_batt_volts = self.create_subscription(Int16,'/ri/batt_volts', self.batt_volts_callback, 1)
        
        self.sub_chrg_req  # prevent unused variable warning
        self.sub_flow_req  # prevent unused variable warning

        self.chrg_stats = "INITIALISED"
        self.flow_stats = "INITIALISED"

        self.chrg_req = False
        self.flow_req = False
        self.flag_chrg_align = False
        self.flag_liquid_ful = False
        self.batt_volts = 0.0  
      
    def chrg_req_callback(self, msg):
        val = msg.data
        self.chrg_req = val
    
    def flow_req_callback(self, msg):
        self.flow_req = msg.data
    
    def chrg_align_callback(self, msg):
        self.flag_chrg_align = msg.data

    def liquid_ful_callback(self, msg):
        self.flag_liquid_ful = msg.data

    def batt_volts_callback(self, msg):
        self.batt_volts = float(msg.data)/10
        
           
    def timer_callback(self):

        ###########
        # handle charging states
        xc = Int16()
        if self.chrg_req == True:
            self.chrg_stats = "REQUESTED"
            if self.flag_chrg_align == True:
                xc.data = 1
                self.chrg_stats = "CHARGING"
            if self.flag_chrg_align == False:
                xc.data = 0

        if self.chrg_req == False:
            self.chrg_stats = "TERMINATED"
            xc.data = 0

        self.pub_cs_chrg.publish(xc)
        self.pub_ri_chrg.publish(xc)

        # handle flow states
        xf = Int16()
        if self.flow_req == True:
            self.flow_stats = "REQUESTED"
            if self.flag_chrg_align == True and self.flag_liquid_ful == False:
                xf.data = 1
                self.flow_stats = "FLOWING"
            elif self.flag_chrg_align == True or self.flag_liquid_ful == True:
                xf.data = 0
                self.flow_stats = "TANK_FULL"
            elif self.flag_chrg_align == False:
                xf.data = 0
                self.flow_stats = "UNALIGNED"

        if self.flow_req == False:
            self.flow_stats = "TERMINATED"
            xf.data = 0

        self.pub_cs_valv.publish(xf)
        self.pub_cs_pump.publish(xf)
        self.pub_ri_valv.publish(xf)
        self.pub_ri_pump.publish(xf)
    

        ###########

        msg_chrg = String()
        msg_flow = String()
        msg_chrg.data =  self.chrg_stats
        msg_flow.data =  self.flow_stats
        self.pub_chrg_stats.publish(msg_chrg)
        self.pub_flow_stats.publish(msg_flow)

     
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
