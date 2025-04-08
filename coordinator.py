import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import threading
import numpy as np
from rclpy.qos import qos_profile_sensor_data

from time import sleep

#List of commands on PC terminal
# python3 ~/Documents/eg2310/coordinator_node.py
# python3 ~/Documents/eg2310/r2auto_nav_modified.py
# ros2 launch auto_mapper auto_mapper.launch.py map_path:=~/map is_sim:=false
 

side_angles = range(-90,91,1)
test_num = 2
def spin_robot(self):
    self.get_logger().info('Spinning the robot 360 degrees...')
    # Create a Twist message
    twist_msg = Twist()
    twist_msg.angular.z = 2.8  # Set angular velocity (adjust as needed)
    # Calculate the duration to spin 360 degrees
    #spin_duration = 2 * 3.14159 / abs(twist_msg.angular.z)  # 360 degrees in radians
    spin_duration = 5
    # Publish the Twist message for the calculated duration
    start_time = self.get_clock().now()
    while (self.get_clock().now() - start_time).nanoseconds / 1e9 < spin_duration:
        self.cmd_vel_pub.publish(twist_msg)
        rclpy.spin_once(self, timeout_sec=0.1)
    # Stop the robot after spinning
    twist_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(twist_msg)
    self.get_logger().info('Robot spin completed!')    
heat_source_array = []
#Frontier Navigation
def start_frontier_navigation(self):
    self.nav_pub.publish(String(data='START'))
    self.get_logger().info('Published navigation start command.')
def stop_frontier_navigation(self):
    self.nav_pub.publish(String(data='STOP'))
    self.get_logger().info('Published navigation stop command.')
#Backup r2autonav
def start_r2autonav(self):
    self.r2autonav_pub.publish(String(data='START'))
    self.get_logger().info('Published r2autonav start command.')
def stop_r2autonav(self):
    self.r2autonav_pub.publish(String(data='STOP'))
    self.get_logger().info('Published r2autonav stop command.')
#Shooting    
def start_shooting(self):
    self.shooting_pub.publish(String(data='START'))
    self.get_logger().info('Published shooting start command.')
def stop_shooting(self):
    self.shooting_pub.publish(String(data='STOP'))
    self.get_logger().info('Published shooting stop command.')
#Thermal
def start_thermal(self):
    self.thermal_pub.publish(String(data='START'))
    self.get_logger().info('Published thermal start command.')
def stop_thermal(self):
    self.thermal_pub.publish(String(data='STOP'))
    self.get_logger().info('Published thermal stop command.')


class CoordinatorNode(Node): 
    def __init__(self):
        super().__init__('coordinator_node')
        
        # Publisher for controlling other nodes (e.g., navigation, shooting, etc.)
        self.r2autonav_pub = self.create_publisher(String, 'r2autonav_command_in', 10)
        self.nav_pub = self.create_publisher(String, 'navigation_command_in', 10)
        self.shooting_pub = self.create_publisher(String, 'shooting_command_in', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.thermal_pub = self.create_publisher(String, 'thermal_command_in', 10)
        # Subscriber to get feedback/status from other nodes
        self.nav_status_sub = self.create_subscription(String, 'navigation_status_out', self.nav_status_callback, 10)
        self.shooting_status_sub = self.create_subscription(String, 'shooting_status_out', self.shooting_status_callback, 10)
        self.r2autonav_status_sub = self.create_subscription(String, 'r2autonav_status_out', self.r2autonav_status_callback, 10)
        self.thermal_sub = self.create_subscription(String, 'thermal_status_out', self.thermal_status_callback, 10)

        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_topic_callback, 10)
        self.current_pose = None
        self.marker_array_pub = self.create_publisher(MarkerArray, '/heat_sources', 10)
        self.marker_array = MarkerArray()

        # Flag to track if navigation task is completed
        self.nav_completed = False
        self.shooting_completed = False
        """
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        # Subscriber to get occupancy grid data
        """
        """
        self.occ_subscription = self.create_subscription(
        OccupancyGrid,
        'map',
        self.occ_callback,
        qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        """

        input_thread = threading.Thread(target=self.wait_for_start_command)
        input_thread.daemon = True  # Ensure the thread exits when the program exits
        input_thread.start()

    def wait_for_start_command(self):
        while True:
            user_input = input("Enter 'r' to start navigation: ").strip().lower()
            if user_input == 'r':
                start_frontier_navigation(self)
                break
            if user_input == 'pass':
                break

#ros2 topic pub /navigation_command_in std_msgs/msg/String "{data: 'START'}"
    def nav_status_callback(self, msg):
        if msg.data == "GOAL_REACHED":
            self.get_logger().info('Frontier Navigation Goal completed! Starting Thermal Detection...')
            if len(heat_source_array) == 2: #if len(test_num) == 2:
                self.ramp_detection()
            else:
                sleep(5)
                start_thermal(self)
            #start_frontier_navigation(self)
        elif msg.data == "r2autonav":
            start_r2autonav(self) 
            #Add delay here to allow r2autonav to complete
            sleep(5)
            stop_r2autonav(self)
            start_thermal(self)
            #self.nav_completed = True
            #self.check_all_tasks_completed()

    def r2autonav_status_callback(self, msg):
        if msg.data == "GOAL_REACHED":
            self.get_logger().info('R2AutoNav Navigation Goal completed! Starting Thermal Detection...')
            start_thermal(self)
            #start_frontier_navigation(self)
            
    def odom_topic_callback(self, msg):
        #self.get_logger().info('Received odometry data.')
        self.current_pose = msg.pose.pose
        #self.get_logger().info(f"Current pose: {self.current_pose.position.x}, {self.current_pose.position.y}, {self.current_pose.position.z}")

    def thermal_status_callback(self,msg):
        if msg.data == "HEAT_SOURCE_DETECTED":
            self.get_logger().info('Heat source detected!')
            if self.current_pose:
                if not self.is_duplicate_heat_source(self.current_pose):
                    self.get_logger().info('Heat source seeking!')
                    self.thermal_pub.publish(String(data='HEAT_SOURCE_SEEK'))
                else:
                    self.get_logger().warn("Heat source already detected in this direction.")
                    stop_thermal(self)
        elif msg.data == "HEAT_SOURCE_REACHED":
            self.publish_marker(self.current_pose)
            heat_source_array.append(self.current_pose)
            self.get_logger().info(f"New heat source recorded at: {self.current_pose.position.x}, {self.current_pose.position.y}")
            start_shooting(self)
        
        elif msg.data == "NO_HEAT_SOURCE_FOUND":
            self.get_logger().info('Heat source not detected!')
            start_frontier_navigation(self)
                    
#ros2 topic pub /thermal_status_out std_msgs/msg/String "{data: "HEAT_SOURCE_DETECTED"}"  
    """      
    def thermal_status_callback(self, msg):
        if msg.data == "HEAT_SOURCE_DETECTED":
            self.get_logger().info('Heat source detected!')
            if self.current_pose:
                if not self.is_duplicate_heat_source(self.current_pose):
                    self.publish_marker(self.current_pose)
                    heat_source_array.append(self.current_pose)
                    self.get_logger().info(f"New heat source recorded at: {self.current_pose.position.x}, {self.current_pose.position.y}")
                else:
                    self.get_logger().info("Heat source already detected in this direction.")
            else:
                self.get_logger().warn("Current pose is not available.")
    """
    
    def is_duplicate_heat_source(self, current_pose):
        # Define thresholds
        radius_of_search = 0.5  # Meters
        distance_threshold = 0.3  # Meters
        #angular_threshold = 0.1  # Radians
        current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)
        # Project a point 0.5m ahead in the direction the robot is facing
        projected_x = current_pose.position.x + distance_threshold * math.cos(current_yaw)
        projected_y = current_pose.position.y + distance_threshold * math.sin(current_yaw)

        for heat_source in heat_source_array:
            # Calculate distance between the projected point and the heat source
            distance = math.sqrt(
                (projected_x - heat_source.position.x) ** 2 +
                (projected_y - heat_source.position.y) ** 2
            )

            # Check if the projected point is within the radius of the heat source
            if distance < radius_of_search:
                return True  # Heat source detected near the projected point

        return False  # No heat source detected near the projected point
    
    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to yaw (rotation around the Z-axis)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp) 
   
#ros2 topic pub /shooting_status_out std_msgs/msg/String "{data: 'SHOOTING_COMPLETED'}"
    def shooting_status_callback(self, msg):
        if msg.data == "SHOOTING_COMPLETED":
            self.get_logger().info('Shooting task completed!')
            #markers.append(self.current_pose)
            #self.publish_marker(self.current_pose)
            #heat_source_array.append(self.current_pose)
            start_frontier_navigation(self)


    def publish_marker(self, pose):
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "map"  # Replace with the appropriate frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "heat_source"
        marker.id = len(self.marker_array.markers)  # Unique ID for each marker
        marker.type = Marker.SPHERE  # Marker type (e.g., SPHERE, CUBE, etc.)
        marker.action = Marker.ADD

        # Set the marker's position from the pose
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation = pose.orientation

        # Set marker scale (size)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set marker color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Append the marker to the MarkerArray
        self.marker_array.markers.append(marker)

        # Publish the updated MarkerArray
        self.marker_array_pub.publish(self.marker_array)

        self.get_logger().info(f"Published marker at position: {pose.position.x}, {pose.position.y}, {pose.position.z}")

    def ramp_detection(self):
        self.get_logger().info('Ramp detection start')
        if self.laser_range.size != 0:
            #while
            if self.laser_range[side_angles]<float(0.35):
                if np.isnan(self.laser_range[0]):
                    self.get_logger().info('Ramp detected')
                    twist = Twist()
                    twist.linear.x = 2
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(Twist())
                    sleep(5)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info('Stopped robot')
                    #self.stop_robot()
                else:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.1
                    self.cmd_vel_pub.publish(Twist())
                    sleep(0.1)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(Twist())
                    self.ramp_detection()
            else:
                pass
                    
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
    """
    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i T

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)

    # Occupancy Grid too complicated

    def is_near_unexplored_region(self, current_pose):

        # Define the search radius in meters (e.g., 0.1m = 10cm)
        search_radius = 0.1  # 10 cm

        # Convert the robot's current position to grid coordinates
        robot_x, robot_y = current_pose.position.x, current_pose.position.y
        grid_x, grid_y = self.world_to_map(robot_x, robot_y)

        # Get the map's resolution and dimensions
        map_height, map_width = self.occdata.shape
        resolution = 0.03  # Replace with your map's resolution (meters per cell)

        # Calculate the number of cells to search in each direction
        cells_to_search = int(search_radius / resolution)

        # Iterate through the cells in the search radius
        for dx in range(-cells_to_search, cells_to_search + 1):
            for dy in range(-cells_to_search, cells_to_search + 1):
                neighbor_x = grid_x + dx
                neighbor_y = grid_y + dy

                # Check if the neighbor cell is within the map bounds
                if 0 <= neighbor_x < map_width and 0 <= neighbor_y < map_height:
                    # Check if the cell is free (0)
                    if self.occdata[neighbor_y, neighbor_x] == 0:
                        return True  # Found an free region nearby

        return False  # No unexplored regions nearby

    def world_to_map(self, x, y):

        map_origin_x = 0  # Replace with the map's origin x-coordinate
        map_origin_y = 0  # Replace with the map's origin y-coordinate
        resolution = 0.03  # Replace with your map's resolution (meters per cell)

        grid_x = int((x - map_origin_x) / resolution)
        grid_y = int((y - map_origin_y) / resolution)

        return grid_x, grid_y
    """

def main(args=None):
    rclpy.init(args=args)
    #Run initial commands here

    # Create the coordinator node
    coordinator_node = CoordinatorNode()
    # Start navigation and shooting tasks
    rclpy.spin(coordinator_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
