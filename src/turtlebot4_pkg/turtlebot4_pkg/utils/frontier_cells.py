import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import random


class ColoredMapPublisher(Node):
    def __init__(self):
        super().__init__('colored_map_publisher')
        self.publisher = self.create_publisher(MarkerArray, '/colored_map', 10)

        # Floor and map configuration
        self.floor_size_x = 10.0  # Meters
        self.floor_size_y = 10.0  # Meters
        self.resolution = 0.05 # Meters per cell
        self.thickness = 0.01 # Slightly thicker for visibility

        self.world_origin_x = -self.floor_size_x / 2.0
        self.world_origin_y = -self.floor_size_y / 2.0

        self.map_size_x = int(self.floor_size_x / self.resolution)
        self.map_size_y = int(self.floor_size_y / self.resolution)

        self.get_logger().info("Colored Map Publisher Node Initialized")

    def to_xy(self, i, j):
        x = j * self.resolution + self.world_origin_x
        y = i * self.resolution + self.world_origin_y
        return x, y

    def generate_random_color(self,cluster_id):
        """
        Generate a random color.
        Returns normalized RGB values.
        """
        random.seed(cluster_id)
        r = random.uniform(0.2, 1.0)  # Avoid too dark or too light colors
        g = random.uniform(0.2, 1.0)
        b = random.uniform(0.2, 1.0)
        return r, g, b, 1.0  # Fully opaque



    def publish_colored_map(self, frontier_list):
        """Publish colored grid based on frontier coordinates."""

        marker_array = MarkerArray()
        marker_id = 0  # Unique ID for each marker

        for cluster_id, cluster in enumerate(frontier_list):
            r, g, b, a = self.generate_random_color(cluster_id)
            for i, j in cluster:

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "colored_cells"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Convert grid coordinates to world position
                x, y = self.to_xy(i, j)
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # Flat grid

                # Set the cell's size
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = self.thickness

                # Assign the batch color
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = a

                # Add the marker to the array
                marker_array.markers.append(marker)
                marker_id += 1
                
        self.publisher.publish(marker_array)
        
    def publish_centroid(self, centeroids):

        """Publish center of the clusters in red."""

        marker_array = MarkerArray()
        marker_id = 0  # Unique ID for each marker
    
        for i,j in centeroids:
                # Create a marker for each cell
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "color_center"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                # Convert grid coordinates to world position
                x, y = self.to_xy(i, j)
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # Flat grid
                # Set the cell's size
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = self.thickness
                # Assign the point eith red color
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                # Add the marker to the array
                marker_array.markers.append(marker)
                marker_id += 1

        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ColoredMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Colored Map Publisher Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()