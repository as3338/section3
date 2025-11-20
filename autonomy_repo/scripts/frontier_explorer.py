#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from scipy.signal import convolve2d
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from asl_tb3_lib.msg import TurtlebotState 

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

        # Parameters
        self.window_size = 13  # Window size for exploration heuristics [cite: 235]
        self.stop_duration = 5.0 # Seconds to stop for a stop sign [cite: 42]
        self.ignore_duration = 3.0 # Seconds to ignore detections after resuming [cite: 45]
        
        # State variables
        self.map_data = None
        self.robot_pose = None # [x, y]
        self.is_exploring = False
        
        # Pause/Resume Logic variables
        self.is_paused = False
        self.last_resume_time = 0.0
        self.stop_timer = None
        
        # Publisher: Sending Goals to Navigator
        # CHANGED: Uses TurtlebotState instead of PoseStamped
        self.cmd_nav_pub = self.create_publisher(TurtlebotState, '/cmd_nav', 10)

        # Subscriber: Map
        # CHANGED: Added QoS to ensure we receive the map from map_server
        map_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # Subscriber: Robot State
        self.create_subscription(TurtlebotState, '/state', self.state_callback, 10)

        # Subscriber: Nav Success
        self.create_subscription(Bool, '/nav_success', self.nav_success_callback, 10)
        
        # Subscriber: Detector (Task 4.1)
        self.create_subscription(Bool, '/detector_bool', self.detector_callback, 10)

        self.get_logger().info("Frontier Explorer Node Started (TurtlebotState Edition)")

    def state_callback(self, msg):
        """Updates the robot's internal state representation[cite: 267]."""
        self.robot_pose = np.array([msg.x, msg.y])

    def map_callback(self, msg):
        self.map_data = msg
        # If we have a map and pose but haven't started, trigger the first plan
        if not self.is_exploring and not self.is_paused and self.robot_pose is not None:
            self.plan_next_frontier()

    def nav_success_callback(self, msg):
        """Triggered when the navigator reaches a goal or fails[cite: 262]."""
        if self.is_paused:
            return

        if self.is_exploring:
            self.get_logger().info(f"Navigation finished (Success: {msg.data}). Planning next frontier...")
            self.plan_next_frontier()

    def detector_callback(self, msg):
        """Handle stop sign detections[cite: 41]."""
        if not msg.data or self.is_paused:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check ignore window
        if (current_time - self.last_resume_time) < self.ignore_duration:
            return

        self.get_logger().info("Stop sign detected! Pausing exploration for 5 seconds.")
        self.is_paused = True
        self.stop_robot()
        
        self.stop_timer = self.create_timer(self.stop_duration, self.resume_exploration)

    def stop_robot(self):
        """Stops the robot by sending a goal at its current location."""
        if self.robot_pose is not None:
            # Send current pose as goal to stop
            self.publish_goal(self.robot_pose)

    def resume_exploration(self):
        self.get_logger().info("Resuming exploration...")
        self.is_paused = False
        self.last_resume_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
            
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

        # 2. Execute Exploration Heuristics [cite: 235]
        frontier_states = self.explore(occupancy)

        # 3. Select and Publish Goal
        if len(frontier_states) > 0:
            distances = np.linalg.norm(frontier_states - self.robot_pose, axis=1)
            closest_idx = np.argmin(distances)
            target_state = frontier_states[closest_idx]
            
            # Publish the new goal
            self.publish_goal(target_state)
        else:
            self.get_logger().info("No valid frontiers found. Retrying on next map update...")
            # CRITICAL FIX: Reset flag so we try again when map updates
            self.is_exploring = False

    def explore(self, occupancy):
        """Applies heuristics to find valid frontier states[cite: 200, 201, 202]."""
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
        """
        Publishes a goal using TurtlebotState msg (x, y, theta).
        """
        msg = TurtlebotState()
        msg.x = float(state_xy[0])
        msg.y = float(state_xy[1])
        msg.theta = 0.0 # Frontier exploration targets positions; theta is 0 by default
        self.cmd_nav_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
