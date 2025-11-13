#!/usr/bin/env python3
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState

class PerceptionController(BaseHeadingController): 
    def __init__(self):
        super().__init__('perception_controller') 
        
        # Task 2.2: Declare 'active' parameter 
        self.declare_parameter("active", True)

    @property
    def active(self) -> bool: # [cite: 33]
        """Get the real-time value of the 'active' parameter."""
        return self.get_parameter("active").value

    def compute_control_with_goal(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        
        message = TurtleBotControl()
        
        # Task 2.3: Spin if active, stay static otherwise 
        if self.active:
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
