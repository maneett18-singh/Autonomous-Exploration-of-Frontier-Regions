import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped,Twist, PoseArray
import numpy as np
import numpy as np
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import  Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy,DurabilityPolicy
import heapq
from geometry_msgs.msg import Twist
import math

"""This node use a Astar to calculate the path and follow the path untill goal probability 
    is more than 0.9 (obstacle) or less than 0.1 (free) then choose the next goal and goes on till no frontier centroids are left."""


INFLATION_RADIUS = 4.5
PRIOR_PROB = 50  # 0.5 * 100
OCC_PROB = 75    # 0.75 * 100
FREE_PROB = 45   # 0.45 * 100

class AStarGlobalPlanner(Node):
    def __init__(self):
        super().__init__('a_star_global_planner')

        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, '/global_path_viz', 10)  # Path visualization

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ROS2 Subscribers
        self.qos_policy = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            durability=DurabilityPolicy.VOLATILE,)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback,  qos_profile=self.qos_policy)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback,  qos_profile=self.qos_policy)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseArray,'/frontier_points', self.frontier_callback, qos_profile=self.qos_policy)


        # funtion for follow path running every 0.1 second
        self.timer = self.create_timer(0.1, self.follow_path)

        # Grid map
        self.grid = None
        self.map_resolution = 0.05 # Default resolution (meters per cell)
        self.map_origin = [0, 0]
        self.x=None 
        self.y=None
        self.theta=0.0
        self.global_path = None

        self.goal_active = False # If goal is not active then calculate astar again for the next goal
        self.costmap_data = None

        self.all_goals = set([]) 
        self.current_path = None
        self.path_end = None

    
    def publish_path(self, waypoints):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in waypoints:  # Assume astar_path is a list of (x, y) tuples
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            i,j = self.to_real(waypoint[0],waypoint[1])
            pose.pose.position.x = i
            pose.pose.position.y = j
            pose.pose.orientation.w = 1.0  # Assume no rotation needed

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def odom_callback(self, msg):
        """ Updates robot position and velocity from odometry """

        from_frame_rel = 'map'
        to_frame_rel = 'base_link'

        if self.tf_buffer.can_transform(from_frame_rel,to_frame_rel,rclpy.time.Time(seconds=0)):

            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())
            
            # Extract translation (x, y, z)
            self.x = t.transform.translation.x
            self.y = t.transform.translation.y
            z = t.transform.translation.z
        
            # Extract rotation as a quaternion (x, y, z, w)
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            self.theta = self.quarternion_to_yaw(qx, qy, qz, qw)

    def map_callback(self, msg):
        """ Receives the occupancy grid map and stores it as a numpy array """
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

    def goal_callback(self, msg):
        print('goal got it')
        """ Runs A* path planning when a goal is received """
        if self.grid is None:
            self.get_logger().warn("No map received yet!")
            return
        if self.x is None or self.y is None:
            print('no x,y from base_link')
            return
    
        #Convert world coordinates to grid indices
        start_x, start_y = self.map_to_grid(self.x, self.y)  
        goal_x, goal_y = self.map_to_grid(msg.pose.position.x, msg.pose.position.y)

        # Run A* search
        inflated_grid = self.inflate_obstacles(self.grid,self.grid.shape[0] ,self.grid.shape[1])
        self.get_logger().log('value of goal in grid', self.grid[goal_x,goal_y])
        path = self.astar(inflated_grid,(start_x, start_y), (goal_x, goal_y),self.grid.shape[0] ,self.grid.shape[1] )
        print(path)

        if path:
            self.get_logger().log('Path Found')
            self.global_path=path
            self.publish_path(path)


    def frontier_callback(self, msg):
        """Processes goals sequentially: calculates, follows, and only then moves to the next goal."""

        self.get_logger().info("Entering frontier callback")

        # Check if robot's position is available
        if self.x is None or self.y is None:
            self.get_logger().error("No x,y from base_link")
            return

        # Check if the map is available
        if self.grid is None:
            self.get_logger().warn("No map received yet!")
            return

        # Inflate obstacles once before path calculations
        inflated_grid = self.inflate_obstacles(self.grid, self.grid.shape[0], self.grid.shape[1])

        # If a goal is already active, do nothing
        if self.goal_active:
            self.get_logger().info("Goal in progress, waiting for completion...")
            return

        # Converting the goal from world to map corrdinates and Storings goals  
        for pose in msg.poses:
            if pose.position is None:
                self.get_logger().error("pose.position is None, skipping this goal.")
                continue
            goal_x, goal_y = self.map_to_grid(pose.position.x, pose.position.y)
            self.all_goals.add((goal_x, goal_y))

        # Start processing the first goal
        if  not self.goal_active:
            self.get_logger().info("Path found, publishing and following it.")
            if len(self.all_goals) > 0 :
                sx,sy = self.map_to_grid(self.x,self.y)
                gx,gy = self.all_goals.pop()
                print(gx,gy)
                self.current_path = self.astar(inflated_grid, (sx, sy), (gx, gy), self.grid.shape[0], self.grid.shape[1])
                print(self.current_path)
                if not self.current_path is None:
                    self.path_end = self.current_path[-1]
                    self.goal_active = True  # Mark goal as active
                    self.publish_path(self.current_path)


    def follow_path(self):

        """Follows the computed path by adjusting the robot's speed and orientation.  
            Checks goal status, computes yaw error, and publishes velocity commands."""

        if not self.current_path or self.path_end is None:
            self.get_logger().info("No valid path available")
            return
        # id the goal is alredy explored before move to next goal by calculating the astar again
        if len(self.current_path) > 1:
            last_index = self.current_path[-1]
            if self.grid[last_index] > 95 or self.grid[last_index] < 10:
                self.goal_active = False
                print('goal explored')

        # Get current robot position
        x, y = self.map_to_grid(self.x, self.y)
        current_x = x
        current_y = y
        current_yaw = self.theta  # Ensure this is updated from a subscriber

        # Check if the robot has reached the final goal
        reach_distance = math.sqrt((current_x - self.path_end[0]) ** 2 + (current_y - self.path_end[1]) ** 2)
        if reach_distance < 10.0:  
            self.get_logger().info("Reached the final goal")
            print(self.all_goals)
            self.goal_active = False
            return

        # Get the next waypoint
        target_x, target_y = self.current_path[0]

        # Compute distance and angle to the target
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        desired_yaw = math.atan2(target_y - current_y, target_x - current_x)
        yaw_error = desired_yaw - current_yaw

        # Normalize yaw error to [-pi, pi]
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        twist = Twist()

        # Speed and turning
        max_speed = 0.3  # Linear velocity
        min_speed = 0.05  # Avoid stopping abruptly
        turn_gain = 0.3  # Higher gain for quicker response

        # Rotate if yaw error is large, otherwise move forward smoothly
        if abs(yaw_error) > 0.2:
            twist.angular.z = turn_gain * yaw_error  # Smooth turning
            twist.linear.x = 0.0  # Avoid drifting while turning
        else:
            twist.linear.x = max(min_speed, min(max_speed, 0.5 * distance))  # Adaptive speed
            twist.angular.z = 0.1 * yaw_error  # Small correction while moving

        # If close to the waypoint, transition to the next one
        if distance < 5.0:
            self.current_path.pop(0)
            self.get_logger().info("Reached waypoint, moving to next")

        self.cmd_vel_publisher.publish(twist)
    ## Reference: This implementation is inhanced using chatgpt : https://chatgpt.com/c/67a87bda-df08-8000-83c6-7362a1007173



    # Inflating the obstacle so robot avoid the obstacle with safety
    def inflate_obstacles(self,grid, width, height):
        inflated_grid = np.copy(grid)
        for y in range(height):
            for x in range(width):
                if grid[y][x] >= OCC_PROB:
                    for dy in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                        for dx in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                inflated_grid[ny][nx] = max(inflated_grid[ny][nx], OCC_PROB)
        return inflated_grid
    

#Astar algorithm start here

    
    # Global path planner
    def astar(self,grid, start, goal, width, height):
    
        """Finds the shortest path from start to goal using the A* algorithm.  
            Uses Manhattan distance as the heuristic and considers 8-directional movement."""
        
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        directions = [(0, -1), (0, 1), (-1, 0), (1, 0),(-1, -1), (-1, 1), (1, -1), (1, 1)] #, 

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                    if grid[neighbor[1]][neighbor[0]] >= OCC_PROB:
                        continue
                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def is_valid(self, pos):
        """ Checks if the position is inside the map and not an obstacle """
        x, y = pos
        if 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]:
            if self.grid[x, y] == 100:  # 0 means free space
                return False
            else:
                return True

    def reconstruct_path(self, came_from, current):
        """ Reconstructs the path from the A* search """
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path[::-1]  # Reverse path
        ## Reference: This implementation is inspired by the code : https://www.geeksforgeeks.org/a-search-algorithm


#Astar algorithm ends here




    # convert the world corrdinates to the grid corrdinates
    def map_to_grid(self, x, y):
        """ Converts world coordinates (meters) to grid indices """
        gx = int((x - self.map_origin[0]) / self.map_resolution)
        gy = int((y - self.map_origin[1]) / self.map_resolution)
        return gx, gy
    
    # convert the grid corrdinates to the World corrdinates
    def to_real(self, x,y):
        x = x * self.map_resolution + self.map_origin[0]
        y = y * self.map_resolution + self.map_origin[1]
        return (x, y)

    def quarternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

    # for smoothing the astar path if required but not using it right now
    def smooth_path(self, path):
        """ Applies simple path smoothing by skipping unnecessary waypoints """
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            if not self.is_valid(path[i]):  # Remove noisy points
                continue
            smoothed.append(path[i])
        smoothed.append(path[-1])
        return smoothed

def main():
    rclpy.init()
    planner = AStarGlobalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
