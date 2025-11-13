#!/usr/bin/env python3
import rclpy
from rclpy.node import Node # We need Node for the subscriber
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState
from std_msgs.msg import Bool # Import the Boolean message type 

class PerceptionController(BaseHeadingController): 
    def __init__(self):
        super().__init__('perception_controller') 
        
        # Task 2.2: Declare 'active' parameter 
        #self.declare_parameter("active", True)


        # Task 4.1: Boolean variable to track detection 
        self.image_detected = False

        # Task 4.1: Subscriber for the /detector_bool topic 
        self.detector_sub = self.create_subscription(
            Bool,
            '/detector_bool',
            self.detector_callback,
            10)
            
        self.get_logger().info("Perception Controller is running...")

    def detector_callback(self, msg: Bool):
        """
        Callback for the /detector_bool topic.
        Sets self.image_detected to True if the message is true.
        """
        if msg.data == True:
            self.get_logger().info("Target object detected!")
            self.image_detected = True  
        else :
            self.image_detected = False

    # @property
    # def active(self) -> bool: # [cite: 33]
    #     """Get the real-time value of the 'active' parameter."""
    #     return self.get_parameter("active").value

    def compute_control_with_goal(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        
        message = TurtleBotControl()
        
        # Task 2.3: Spin if active, stay static otherwise 
        #if self.active:
        if not self.image_detected:

            # Spin with a constant angular velocity
            message.omega = 0.2 
        else:
            # Be static (zero velocity)
            message.omega = 0.0
            
        message.v = 0.0 # Always 0.0 linear velocity
        return message

if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()