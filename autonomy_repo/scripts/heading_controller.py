#!/usr/bin/env python3
import numpy
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.declare_parameter("kp",2)
    
    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value

    def compute_control_with_goal(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        err = wrap_angle(goal.theta - state.theta)
        angular_velocity = self.kp * err
        message  = TurtleBotControl()
        message.omega = float(angular_velocity)
        return message

if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()
