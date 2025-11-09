#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.signal import convolve2d

from nav_msgs.msg import OccupancyGrid
from asl_tb3_msgs.msg import TurtleBotState
from std_msgs.msg import Bool

# Assumes this class is available in your environment as per the homework context [cite: 190]
from asl_tb3_lib.grids import StochOccupancyGrid2D

class ExplorerNode(Node):
    """
    A standalone ROS2 node for autonomous frontier exploration.
    It finds frontiers based on Problem 2 heuristics 
    and sends the closest one  to the navigator as a goal.
    """
    def __init__(self):
        super().__init__('explorer_node')

        # --- Parameters ---
        self.declare_parameter('window_size', 7)
        self.declare_parameter('unknown_thresh', 0.20)
        self.declare_parameter('occupied_thresh', 0)
        self.declare_parameter('unoccupied_thresh', 0.30)
        
        # --- State Variables ---
        self.current_map_msg = None
        self.current_state = None
        self.is_navigating = False
        self.map_resolution = 0.0
        self.map_origin = None

        # --- Publishers and Subscribers ---
        
        # Subscribe to topics needed to find frontiers 
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.state_sub = self.create_subscription(
            TurtleBotState,
            '/state',
            self.state_callback,
            10)
        
        # Subscribe to navigator status to send next goal 
        self.nav_success_sub = self.create_subscription(
            Bool,
            '/nav_success',
            self.nav_success_callback,
            10)
        
        # Publish goals to the navigator node
        self.goal_pub = self.create_publisher(
            TurtleBotState,
            '/cmd_nav',
            10)

        self.get_logger().info("Frontier Exploration Node is ready.")

    def state_callback(self, msg):
        """Stores the robot's current state."""
        self.current_state = msg

    def nav_success_callback(self, msg: Bool):
        """
        Callback for when the navigator finishes a goal.
        If successful (or failed), it releases the navigation lock
        and triggers a search for the next frontier.
        """
        self.is_navigating = False
        self.get_logger().info(f"Navigation complete. Success: {msg.data}. Finding next frontier...")
        # Trigger a new exploration cycle
        self.explore()

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback for new map data. Stores map info and triggers
        exploration if not already navigating.
        """
        self.current_map_msg = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        
        # If we get a new map and aren't busy, try to explore
        if not self.is_navigating:
            self.explore()

    def explore(self):
        """
        Main exploration logic. Finds all frontiers, selects the
        closest one, and sends it as a goal to the navigator.
        """
        # --- Guard Clauses ---
        if self.is_navigating:
            # self.get_logger().info("Already navigating to a goal.", throttle_duration_sec=5.0)
            return
        
        if self.current_map_msg is None or self.current_state is None:
            self.get_logger().info("Waiting for map and state data...", throttle_duration_sec=5.0)
            return
            
        self.get_logger().info("Running exploration logic...")

        # --- 1. Process Map using Problem 2 Heuristics ---
        
        # Use StochOccupancyGrid2D as suggested [cite: 190]
        # We assume it has a method to get a probability matrix
        # (e.g., -1 for unknown, 0-1 for known)
        try:
            # This is a hypothetical constructor based on[cite: 190].
            # If this fails, you may need to use msg.data directly.
            grid = StochOccupancyGrid2D.from_ros_msg(self.current_map_msg)
            grid_probs = grid.get_probs() # Assumes this returns np.array
        except Exception as e:
            # Fallback to manual processing if from_ros_msg or get_probs fails
            self.get_logger().warn(f"StochOccupancyGrid2D failed ({e}). Falling back to manual map parsing.")
            map_data = np.array(self.current_map_msg.data).reshape(
                (self.current_map_msg.info.height, self.current_map_msg.info.width)
            )
            # Convert ROS map data (-1 unknown, 0-100 occupied) to ( -1 unknown, 0-1 probability)
            grid_probs = np.full_like(map_data, -1.0, dtype=float)
            grid_probs[map_data != -1] = map_data[map_data != -1] / 100.0

        # Get window size and kernel for convolution
        w = self.get_parameter('window_size').value
        kernel = np.ones((w, w), dtype=np.uint8)
        total_cells_in_window = float(w * w)

        # Create binary masks for cell types [cite: 129]
        unknown_mask = (grid_probs == -1).astype(np.uint8)
        occupied_mask = (grid_probs >= 0.5).astype(np.uint8)
        unoccupied_mask = ((grid_probs < 0.5) & (grid_probs >= 0)).astype(np.uint8)

        # Run convolution to count neighbors
        unknown_count = convolve2d(unknown_mask, kernel, mode='same', boundary='fill', fillvalue=0)
        occupied_count = convolve2d(occupied_mask, kernel, mode='same', boundary='fill', fillvalue=0)
        unoccupied_count = convolve2d(unoccupied_mask, kernel, mode='same', boundary='fill', fillvalue=0)

        # Apply exploration heuristics from Problem 2 
        heuristic_1 = (unknown_count / total_cells_in_window) >= self.get_parameter('unknown_thresh').value
        heuristic_2 = (occupied_count == self.get_parameter('occupied_thresh').value)
        heuristic_3 = (unoccupied_count / total_cells_in_window) >= self.get_parameter('unoccupied_thresh').value

        # Final frontier mask: must satisfy all 3 heuristics AND be an unoccupied cell itself
        frontier_cells_mask = heuristic_1 & heuristic_2 & heuristic_3 & (unoccupied_mask == 1)

        # Get (row, col) indices of all valid frontier cells
        # These are in grid coordinates
        frontier_indices_grid = np.argwhere(frontier_cells_mask) # shape (N, 2)

        if frontier_indices_grid.shape[0] == 0:
            self.get_logger().info("No new frontiers found. Exploration may be complete.")
            # self.is_navigating = False (already False, leave it)
            return

        # --- 2. Find Closest Frontier (Problem 2.ii)  ---
        
        # Convert robot's world (x, y) to grid (row, col)
        robot_x_world = self.current_state.x
        robot_y_world = self.current_state.y
        origin_x_world = self.map_origin.position.x
        origin_y_world = self.map_origin.position.y
        
        robot_x_grid = int((robot_x_world - origin_x_world) / self.map_resolution)
        robot_y_grid = int((robot_y_world - origin_y_world) / self.map_resolution)
        
        # Note: np.argwhere gives (row, col), which is (y, x)
        robot_pos_grid = np.array([robot_y_grid, robot_x_grid])

        # Calculate Euclidean distance from robot to all frontiers
        distances = np.linalg.norm(frontier_indices_grid - robot_pos_grid, axis=1)
        
        # Find the index of the closest frontier
        closest_frontier_idx = np.argmin(distances)
        closest_frontier_grid = frontier_indices_grid[closest_frontier_idx] # [row, col]
        
        # Report distance [cite: 160]
        self.get_logger().info(f"Closest frontier found at grid {closest_frontier_grid}, "
                             f"distance: {distances[closest_frontier_idx]:.2f} grid units.")

        # --- 3. Send Goal to Navigator ---
        
        # Set lock to prevent sending more goals until this one is done
        self.is_navigating = True

        # Convert closest frontier's grid (row, col) back to world (x, y)
        # We target the center of the grid cell
        goal_y_grid, goal_x_grid = closest_frontier_grid
        goal_x_world = (goal_x_grid + 0.5) * self.map_resolution + origin_x_world
        goal_y_world = (goal_y_grid + 0.5) * self.map_resolution + origin_y_world

        # Calculate a simple heading (theta) pointing from robot to goal
        delta_x = goal_x_world - robot_x_world
        delta_y = goal_y_world - robot_y_world
        goal_theta = np.arctan2(delta_y, delta_x)

        # Create and publish the goal message
        goal_msg = TurtleBotState()
        goal_msg.x = float(goal_x_world)
        goal_msg.y = float(goal_y_world)
        goal_msg.theta = float(goal_theta)
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published new goal to navigator: (x={goal_x_world:.2f}, y={goal_y_world:.2f})")

def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()
    rclpy.spin(explorer_node)
    
    explorer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()