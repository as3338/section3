#!/usr/bin/env python3

import numpy as np
from asl_tb3__lib.navigation import BaseNavigator
import rclpy
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState
from scipy.interpolate import splev

class AStar(object):
   """Represents a motion planning problem to be solved using A*"""
   def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
       self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
       self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
       self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
       self.resolution = resolution               # resolution of the discretization of state space (cell/m)
       self.x_offset = x_init                    
       self.x_init = self.snap_to_grid(x_init)    # initial state
       self.x_goal = self.snap_to_grid(x_goal)    # goal state


       self.closed_set = set()    # the set containing the states that have been visited
       self.open_set = set()      # the set containing the states that are condidate for future expension


       self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
       self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
       self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path


       self.open_set.add(self.x_init)
       self.cost_to_arrive[self.x_init] = 0
       self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)


       self.path = None        # the final path as a list of states


   def is_free(self, x):
       """
       Checks if a give state x is free, meaning it is inside the bounds of the map and
       is not inside any obstacle.
       Inputs:
           x: state tuple
       Output:
           Boolean True/False
       Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
             useful here
       """
       ########## Code starts here ##########
       if x[0] < self.statespace_lo[0] or x[0] > self.statespace_hi[0] or x[1] < self.statespace_lo[1] or x[1] > self.statespace_hi[1]:
           return False


       return self.occupancy.is_free(x)
       ########## Code ends here ##########


   def distance(self, x1, x2):
       """
       Computes the Euclidean distance between two states.
       Inputs:
           x1: First state tuple
           x2: Second state tuple
       Output:
           Float Euclidean distance


       HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
       """
       ########## Code starts here ##########   
       #return float(np.sum(np.abs(np.array(x1) - np.array(x2)))) # L1
       return float(np.max(np.abs(np.array(x1) - np.array(x2)))) # Linf
       #return float(np.linalg.norm(np.abs(np.array(x1) - np.array(x2)))) # L2
       ########## Code ends here ##########


   def snap_to_grid(self, x):
       """ Returns the closest point on a discrete state grid
       Input:
           x: tuple state
       Output:
           A tuple that represents the closest point to x on the discrete state grid
       """
       return (
           self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
           self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
       )


   def get_neighbors(self, x):
       """
       Gets the FREE neighbor states of a given state x. Assumes a motion model
       where we can move up, down, left, right, or along the diagonals by an
       amount equal to self.resolution.
       Input:
           x: tuple state
       Ouput:
           List of neighbors that are free, as a list of TUPLES


       HINTS: Use self.is_free to check whether a given state is indeed free.
              Use self.snap_to_grid (see above) to ensure that the neighbors
              you compute are actually on the discrete grid, i.e., if you were
              to compute neighbors by adding/subtracting self.resolution from x,
              numerical errors could creep in over the course of many additions
              and cause grid point equality checks to fail. To remedy this, you
              should make sure that every neighbor is snapped to the grid as it
              is computed.
       """
       neighbors = []
       ########## Code starts here ##########
       offsets = []
       for mult_x in (-1, 0 , 1):
           for mult_y in (-1, 0 , 1):
               if not (mult_x == 0 and mult_y == 0):
                   offsets.append((mult_x * self.resolution, mult_y * self.resolution))


       for dx, dy in offsets:
           neighbor = self.snap_to_grid((x[0] + dx, x[1] + dy)) # ensure it is on grid
           if neighbor == x:
               continue
           if self.is_free(neighbor): # check if free
               neighbors.append(neighbor)
       ########## Code ends here ##########
       return neighbors


   def find_best_est_cost_through(self):
       """
       Gets the state in open_set that has the lowest est_cost_through
       Output: A tuple, the state found in open_set that has the lowest est_cost_through
       """
       return min(self.open_set, key=lambda x: self.est_cost_through[x])


   def reconstruct_path(self):
       """
       Use the came_from map to reconstruct a path from the initial location to
       the goal location
       Output:
           A list of tuples, which is a list of the states that go from start to goal
       """
       path = [self.x_goal]
       current = path[-1]
       while current != self.x_init:
           path.append(self.came_from[current])
           current = path[-1]
       return list(reversed(path))


   def plot_path(self, fig_num=0, show_init_label=True):
       """Plots the path found in self.path and the obstacles"""
       if not self.path:
           return


       self.occupancy.plot(fig_num)


       solution_path = np.asarray(self.path)
       plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
       plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
       if show_init_label:
           plt.annotate(r"$x_{init}$", np.array(self.x_init) + np.array([.2, .2]), fontsize=16)
       plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
       plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)


       plt.axis([0, self.occupancy.width, 0, self.occupancy.height])


   def plot_tree(self, point_size=15):
       plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
       plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
       px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
       py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
       plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)


   def solve(self):
       """
       Solves the planning problem using the A* search algorithm. It places
       the solution as a list of tuples (each representing a state) that go
       from self.x_init to self.x_goal inside the variable self.path
       Input:
           None
       Output:
           Boolean, True if a solution from x_init to x_goal was found


       HINTS:  We're representing the open and closed sets using python's built-in
               set() class. This allows easily adding and removing items using
               .add(item) and .remove(item) respectively, as well as checking for
               set membership efficiently using the syntax "if item in set".
       """
       ########## Code starts here ##########


       self.open_set = set([self.x_init])
       self.closed_set = set()
       self.cost_to_arrive = {self.x_init: 0.0}
       self.est_cost_through = {self.x_init: self.distance(self.x_init, self.x_goal)}
      
       while len(self.open_set) > 0:
           current = self.find_best_est_cost_through()


           if current == self.x_goal:
               self.path = self.reconstruct_path()
               return True


           self.open_set.remove(current)
           self.closed_set.add(current)


           for neighbor in self.get_neighbors(current):
               if neighbor in self.closed_set:
                   continue


               tentative_cost_to_arrive = self.cost_to_arrive[current] + self.distance(current, neighbor)


               if neighbor not in self.open_set:
                   self.open_set.add(neighbor)
               elif tentative_cost_to_arrive > self.cost_to_arrive.get(neighbor, float('inf')):
                   continue


               self.came_from[neighbor] = current
               self.cost_to_arrive[neighbor] = tentative_cost_to_arrive
               self.est_cost_through[neighbor] = tentative_cost_to_arrive + self.distance(neighbor, self.x_goal)


       return False
       ########## Code ends here ##########


class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__("navigator")
        self.om_prev = 0
        self.V_prev = 0
        self.t_prev = 0
        self.V_PREV_THRES = 0.0001
        
        self.kpx = 6.0
        self.kpy = 6.0
        self.kdx = 2.0
        self.kdy = 2.0

        self.V_max = 0.5
        self.om_max = 1

        self.coeffs = np.zeros(8)


    def compute_heading_control(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        err = wrap_angle(goal.theta - state.theta)
        angular_velocity = self.kp * err
        message  = TurtleBotControl()
        message.omega = float(angular_velocity)
        return message
    
    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        dt = t - self.t_prev
        x, y, th = state.x, state.y, state.theta
        x_d, xd_d, xdd_d = splev(t, plan.path_x_spline)
        y_d, yd_d, ydd_d = splev(t, plan.path_y_spline)
        
        ########## Code starts here ##########
        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        xd = self.V_prev*np.cos(th)
        yd = self.V_prev*np.sin(th)

        # compute virtual controls
        u = np.array([xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(th), -self.V_prev*np.sin(th)],
                          [np.sin(th), self.V_prev*np.cos(th)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a*dt
        ########## Code ends here ##########

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        return V, om

    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D, resolution: float, horizon: float,) -> T.Optional[TrajectoryPlan]:
        astar = AStar((-horizon, -horizon), (horizon, horizon), state.x, goal.x, occupancy, resolution)
        if not astar.solve() or len(astar.path) < 4:
            return None
        self.V_prev = 0
        self.t_prev = 0
        
        path = np.asarray(astar.path)
        deltas = np.linalg.norm(np.diff(path, axis=0), axis=1)
        segment_times = deltas / 0.15
        ts = np.concatenate([0.0], np.cumsum(segment_times))

        path_x_spline = scipy.interpolate.splrep(ts, path[:0], s=0.5, k=3)
        path_y_spline = scipy.inteprolate.splrep(ts, path[:1], s=0.5, k=3)

        return TrajectoryPlan(path=path, path_x_spline=path_x_spline, path_y_spline=path_y_spline, duration=ts[-1])


if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()
