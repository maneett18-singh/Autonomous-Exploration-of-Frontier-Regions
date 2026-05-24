
import rclpy
import rclpy.publisher
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Twist
import rclpy.time
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener,TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
import numpy as np
from turtlebot4_pkg.utils.wavefrontier_detection import wavefront_frontier_detection, get_centroid
from turtlebot4_pkg.utils.frontier_cells import ColoredMapPublisher




FLOOR_SIZE_X = 10 # meters, 
FLOOR_SIZE_Y = 10 # meters
RESOLUTION = 0.05 # meters per cell

WORLD_ORIGIN_X = -FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = -FLOOR_SIZE_Y / 2.0

MAP_SIZE_X = int(FLOOR_SIZE_X / RESOLUTION)
MAP_SIZE_Y = int(FLOOR_SIZE_Y / RESOLUTION)

PRIOR_PROB = 0.5
OCC_PROB   = 0.75
FREE_PROB  = 0.45

# p(x) = 1 - \frac{1}{1 + e^l(x)}
def l2p(l):
    return 1 - (1/(1+np.exp(l)))

# Reference: This implementation is inspired by the code from the GitHub repository: https://github.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py#L39


# l(x) = log(\frac{p(x)}{1 - p(x)})
def p2l(p):
    return np.log(p/(1-p))

# Reference: This implementation is inspired by the code from the GitHub repository: https://github.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py#L39


class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')

        self.qos_policy = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            durability=DurabilityPolicy.VOLATILE,
)

        # Buffer and listener for transformations
        self.target_frame = self.declare_parameter('target_frame', 'base_link').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the occupancy grid
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,  
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        self.points_publisher = self.create_publisher(PoseArray, '/frontier_points', 10)

        self.timer = self.create_timer(2.0, self.publish_points)

    
        self.centeroids = []

        self.cp = ColoredMapPublisher()
        

        #Static Transform Broadcaster (for map to odom)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_static = TransformStamped()
        self.tf_static.header.stamp = self.get_clock().now().to_msg()
        self.tf_static.header.frame_id = 'map'
        self.tf_static.child_frame_id = 'odom'
        self.tf_static.transform.translation.x = 0.0
        self.tf_static.transform.translation.y = 0.0
        self.tf_static.transform.translation.z = 0.0
        self.tf_static.transform.rotation.w = 1.0
        self.tf_static.transform.rotation.x = 0.0
        self.tf_static.transform.rotation.y = 0.0
        self.tf_static.transform.rotation.z = 0.0
        self.static_tf_broadcaster.sendTransform(self.tf_static)

        #reference: This implementation is inspired by the code from the : https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the robot's position and orientation
        self.robotX = 0.0
        self.robotY = 0.0
        self.robotPhi = 0.0  

        #scan metrics
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.range_max = 0.0

        self.prev_robot_x = -99999999
        self.prev_robot_y = -99999999
        self.update_movement = 0.1

        self.sensor_model_l_occ = p2l(OCC_PROB)
        self.sensor_model_l_free = p2l(FREE_PROB)
        self.sensor_model_l_prior = p2l(PRIOR_PROB)


        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=self.qos_policy)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=self.qos_policy)

        self.grid_init(RESOLUTION,MAP_SIZE_X,MAP_SIZE_Y) 
        
        self.frontiers = []
        
    def grid_init(self,resolution,width,height):

        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'map'   
        self.occupancy_grid_msg.info.resolution = resolution
        self.occupancy_grid_msg.info.width = width
        self.occupancy_grid_msg.info.height = height
        self.occupancy_grid_msg.info.origin.position.x = WORLD_ORIGIN_X
        self.occupancy_grid_msg.info.origin.position.y = WORLD_ORIGIN_Y
        self.occupancy_grid = self.sensor_model_l_prior * np.ones((width, height))
        #reference: This implementation is inspired by the code from the : https://github.com/noshluk2/ros2_learners/blob/main/navigation_tb3/src/pub_occupancy_grid.cpp

    
    def reset_occupancy_grid(self):
        self.occupancy_grid = np.zeros((MAP_SIZE_Y, MAP_SIZE_X), dtype=int)
     

    def pub_map(self, grid):
    
        """
        Publish an OccupancyGrid to the /map topic.
        """   
        gridmap_p = l2p(grid)
        gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_grid_msg.data = gridmap_int8.flatten().tolist()
        self.map_publisher.publish(self.occupancy_grid_msg)
    # Reference: This implementation is inspired by the code from the GitHub repository: https://github.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py#L157
    
    def publish_points(self):

        """Convert grid points to world coordinates and publish as PoseArray."""

        if len(self.centeroids) < 1:
            return
        
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"  # Set the frame to "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for i,j in self.centeroids:
            x,y = self.map_to_world(i,j)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose_array.poses.append(pose)

        self.points_publisher.publish(pose_array)
        self.get_logger().info(f"Published {len(self.centeroids)} points as PoseArray.")



    def odom_callback(self, msg_odom):
        """
        Reads odometry message, updates robot position, rotation and velocity and performs tf2-transformation between map and robot (base_link).

        Args:
            msg_odom: message received from topic /odom
        """

        pose = msg_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        t = TransformStamped()
        t.header.stamp = msg_odom.header.stamp  # timestamp from the odometry message
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)



    def quarternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

    def world_to_map (self, x, y):
        i = (y-WORLD_ORIGIN_Y) / RESOLUTION
        j = (x-WORLD_ORIGIN_X) / RESOLUTION
        return i, j
    def map_to_world(self, i, j):
        x = j * RESOLUTION + WORLD_ORIGIN_X
        y = i * RESOLUTION + WORLD_ORIGIN_Y
        return x, y
    # reference: https://github.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py#L38-L46

    def is_inside (self, i, j):
        return ( 0 <= i < self.occupancy_grid.shape[0] and 0 <= j < self.occupancy_grid.shape[1])
    # reference: https://github1s.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py#L48-L49

    
    def update_map(self,x,y, theta, scan):
        for i, distance in enumerate(scan.ranges):
            if distance < self.range_max:
                angle =  theta + self.angle_min + i * self.angle_increment
                x0 = x
                y0 = y
                x1 = x0 + distance * np.cos(angle + np.pi/2)  # Adjusting the angle
                y1 = y0 + distance * np.sin(angle + np.pi/2)
                i0, j0 = self.world_to_map(x0, y0)
                i1, j1 = self.world_to_map(x1, y1)
                d_cells = distance / RESOLUTION
                last_i, last_j,_ = self.bresenham(i0, j0, i1, j1, d_cells)
                
                if not np.isnan(distance) and distance != self.range_max and self.is_inside(int(last_i),int(last_j)):
          

                    
                    self.occupancy_grid[int(last_i),int(last_j)] += self.sensor_model_l_occ - self.sensor_model_l_prior
 
        return self.occupancy_grid
        


    def bresenham(self, x, y, i1, j1, d): 
        dx = np.absolute(j1-y)
        dy = -1 * np.absolute(i1-x)
        sx = -1
        if y<j1:
            sx = 1
        sy = -1
        if x<i1:
            sy = 1
        jp, ip = y, x
        err = dx+dy                    
        while True:                     
            if (jp == j1 and ip == i1) or (np.sqrt((jp-y)**2+(ip-x)**2) >= d) or not self.is_inside(ip, jp):
                return ip, jp, False
            elif self.occupancy_grid[int(ip),int(jp)]==100:
                return ip, jp, True

            if self.is_inside(ip, jp):
                # miss:
                self.occupancy_grid[int(ip),int(jp)] += self.sensor_model_l_free - self.sensor_model_l_prior

            e2 = 2*err
            if e2 >= dy:                
                err += dy
                jp += sx
            if e2 <= dx:                
                err += dx
                ip += sy
    #reference: https://github1s.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py#L73-L99
    
    def scan_callback(self, msg_scan):


        """
        Reads scan message and updates the occupancy grid based on detected obstacles.
        
        Args:
            msg_scan: message received from topic /scan
        """
        
        # Check if robot position is already known
        if self.robotX is None or self.robotY is None:
            return
            
        from_frame_rel = 'map'
        to_frame_rel = self.target_frame

        if self.tf_buffer.can_transform(from_frame_rel,to_frame_rel,rclpy.time.Time(seconds=0)):
            #self.grid_init(RESOLUTION,MAP_SIZE_X,MAP_SIZE_Y)
            
             
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())
                # Extract translation (x, y, z)
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            # Extract rotation as a quaternion (x, y, z, w)
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            theta = self.quarternion_to_yaw(qx, qy, qz, qw)
            
            self.angle_min = msg_scan.angle_min
            self.angle_max = msg_scan.angle_max
            self.angle_increment = msg_scan.angle_increment
            self.range_max = msg_scan.range_max

            if ( (x-self.prev_robot_x)**2 + (y-self.prev_robot_y)**2 >= self.update_movement**2 ):
                print('first',(x-self.prev_robot_x)**2 + (y-self.prev_robot_y)**2)
                print('second',self.update_movement**2 )

                updated_grid = self.update_map(x,y,theta,msg_scan)
        
                self.prev_robot_x = x
                self.prev_robot_y = y
                self.robotX = x
                self.robotY = y
                ir,yr = self.world_to_map(x,y)
                print(ir,yr)
        
                self.pub_map(updated_grid)
                grid_p = l2p(updated_grid)
                
                frontier = wavefront_frontier_detection(grid_p,int(ir),int(yr))
                self.cp.publish_colored_map(frontier)


                self.centeroids = get_centroid(frontier)
                self.cp.publish_centroid(self.centeroids)
                print(self.centeroids)

        else:
            self.get_logger().info(
                f'Could not transform {from_frame_rel} to {to_frame_rel}')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()