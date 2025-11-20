#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.signal import convolve2d
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from asl_tb3_msgs.msg import TurtleBotState

class StochOccupancyGrid2D(object):
    """
    A stochastic occupancy grid derived from ROS2 map data.
    """
    def __init__(self, resolution, size_xy, origin_xy, window_size, probs, thresh=0.5):
        self.resolution = resolution
        self.size_xy = size_xy
        self.origin_xy = origin_xy
        self.probs = np.reshape(np.asarray(probs), (size_xy[1], size_xy[0]))
        self.window_size = window_size
        self.thresh = thresh

    def state2grid(self, state_xy):
        state_snapped_xy = self.resolution * np.round(state_xy / self.resolution)
        grid_xy = ((state_snapped_xy - self.origin_xy) / self.resolution).astype(int)
        return grid_xy

    def grid2state(self, grid_xy):
        return (grid_xy * self.resolution + self.origin_xy).astype(float)

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters from HW4 Section 7
        self.window_size = 13  # Window size for exploration heuristics
        self.stop_duration = 5.0 # Seconds to stop for a stop sign
        self.ignore_duration = 3.0 # Seconds to ignore detections after resuming
        
        # State variables
        self.map_data = None
        self.robot_pose = None # [x, y]
        self.robot_theta = 0.0 # Track theta for proper stopping
        self.is_exploring = False
        
        # Pause/Resume Logic variables
        self.is_paused = False
        self.last_resume_time = 0.0
        self.stop_timer = None
        
        # Publishers
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, '/cmd_nav', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(TurtleBotState, '/state', self.state_callback, 10)
        self.create_subscription(Bool, '/nav_success', self.nav_success_callback, 10)
        self.create_subscription(Bool, '/detector_bool', self.detector_callback, 10)

        self.get_logger().info("Frontier Explorer Node Started")

    def state_callback(self, msg):
        # Just update state. DO NOT PLAN HERE.
        self.robot_pose = np.array([msg.x, msg.y])
        self.robot_theta = msg.theta

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        # If we have a map and pose but haven't started, trigger the first plan
        if not self.is_exploring and not self.is_paused and self.robot_pose is not None:
             self.get_logger().info("Map received. Starting exploration...")
             self.plan_next_frontier()

    def nav_success_callback(self, msg):
        # If paused, ignore nav success (likely caused by our stop command)
        if self.is_paused:
            return

        # If navigation finished (success or fail), plan next
        if self.is_exploring:
            self.get_logger().info(f"Navigation finished (Success: {msg.data}). Planning next frontier...")
            self.plan_next_frontier()

    def detector_callback(self, msg):
        # Task 4.1: Handle stop sign detections.
        # If no stop sign (false) or already paused, do nothing
        if not msg.data or self.is_paused:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check if we are in the "ignore" window after recently resuming
        if (current_time - self.last_resume_time) < self.ignore_duration:
            return

        # Trigger Stop Sequence
        self.get_logger().info("Stop sign detected! Pausing exploration for 5 seconds.")
        self.is_paused = True
        self.stop_robot()
        
        # Schedule resume
        self.stop_timer = self.create_timer(self.stop_duration, self.resume_exploration)

    def stop_robot(self):
        """Stops the robot by sending a goal at its current location."""
        if self.robot_pose is not None:
            msg = TurtleBotState()
            msg.x = float(self.robot_pose[0])
            msg.y = float(self.robot_pose[1])
            # IMPORTANT: Use current theta so it doesn't spin away from the stop sign
            msg.theta = float(self.robot_theta)
            self.cmd_nav_pub.publish(msg)
            self.get_logger().info(f"Published stop goal at ({msg.x:.2f}, {msg.y:.2f})")

    def resume_exploration(self):
        """Resumes exploration after the pause duration."""
        self.get_logger().info("Resuming exploration...")
        self.is_paused = False
        self.last_resume_time = self.get_clock().now().nanoseconds / 1e9
        
        # Clean up timer
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
            
        # Immediately plan next frontier
        self.plan_next_frontier()

    def plan_next_frontier(self):
        if self.map_data is None or self.robot_pose is None or self.is_paused:
            return

        self.is_exploring = True

        # 1. Convert ROS OccupancyGrid to StochOccupancyGrid2D
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        raw_data = np.array(self.map_data.data)
        probs = np.full(raw_data.shape, -1.0)
        
        # Fix data mapping: 0..100 -> 0.0..1.0
        known_mask = (raw_data != -1)
        probs[known_mask] = raw_data[known_mask] / 100.0

        occupancy = StochOccupancyGrid2D(
            resolution=resolution,
            size_xy=np.array([width, height]),
            origin_xy=np.array([origin_x, origin_y]),
            window_size=self.window_size,
            probs=probs,
            thresh=0.5
        )

        # 2. Execute Exploration Heuristics
        frontier_states = self.explore(occupancy)

        # 3. Select and Publish Goal
        if len(frontier_states) > 0:
            distances = np.linalg.norm(frontier_states - self.robot_pose, axis=1)
            closest_idx = np.argmin(distances)
            target_state = frontier_states[closest_idx]
            self.publish_goal(target_state)
        else:
            self.get_logger().info("No valid frontiers found.")
            self.is_exploring = False

    def explore(self, occupancy):
        """Applies heuristics to find valid frontier states."""
        window_size = self.window_size
        window = np.ones((window_size, window_size))
        total_cells = window_size * window_size
        probs = occupancy.probs

        unknown_mask = (probs == -1).astype(int)
        occupied_mask = (probs >= 0.5).astype(int)
        unoccupied_mask = ((probs < 0.5) & (probs != -1)).astype(int)

        unknown_count = convolve2d(unknown_mask, window, mode='same')
        occupied_count = convolve2d(occupied_mask, window, mode='same')
        unoccupied_count = convolve2d(unoccupied_mask, window, mode='same')

        h1 = (unknown_count / total_cells) >= 0.20
        h2 = (occupied_count == 0)
        h3 = (unoccupied_count / total_cells) >= 0.30

        valid_mask = h1 & h2 & h3
        
        valid_indices_yx = np.argwhere(valid_mask)
        valid_indices_xy = valid_indices_yx[:, ::-1]
        
        return occupancy.grid2state(valid_indices_xy)

    def publish_goal(self, state_xy):
        msg = TurtleBotState()
        msg.x = float(state_xy[0])
        msg.y = float(state_xy[1])
        msg.theta = 0.0 # Orientation doesn't matter much for exploration, 0 is fine
        self.cmd_nav_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
