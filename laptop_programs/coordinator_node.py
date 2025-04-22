# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Mayukh Ghosh

from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
import math
import threading
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from time import sleep

# COORDINATOR PROGRAM WITH HEAT NAVIGATION GOALS
# List of commands to run on PC terminal (for testing)
# python3 ~/Documents/eg2310/coordinator_node.py
# python3 ~/Documents/eg2310/r2auto_nav_modified.py
# ros2 launch auto_mapper auto_mapper.launch.py map_path:=~/map is_sim:=false

# NOTE: The following code is the final code we have used for our final mission run. We have kept some parts such as the ramp detection which were not used in the final mission run but are still useful for future reference. 
# However, they still need to to properly implemented in the code so that is left for any future users of this code to do. 
 
#side_angles = range(-90,91,1) #For ramp detection

#Frontier Navigation
def start_frontier_navigation(self):
    self.nav_pub.publish(String(data='START'))
    self.get_logger().info('Published navigation start command.')
#Backup r2autonav
def start_r2autonav(self):
    self.r2autonav_pub.publish(String(data='START'))
    self.get_logger().info('Published r2autonav start command.')
#Shooting    
def start_shooting(self):
    self.shooting_pub.publish(String(data='START'))
    self.get_logger().info('Published shooting start command.')
#Thermal
def start_thermal(self):
    self.thermal_pub.publish(String(data='START'))
    self.get_logger().info('Published thermal start command.')


class CoordinatorNode(Node): 
    def __init__(self):
        super().__init__('coordinator_node')
        
        # Publishers for controlling other nodes (e.g., navigation, shooting, etc.)
        self.r2autonav_pub = self.create_publisher(String, 'r2autonav_command_in', 10)
        self.nav_pub = self.create_publisher(String, 'navigation_command_in', 10)
        self.shooting_pub = self.create_publisher(String, 'shooting_command_in', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.thermal_pub = self.create_publisher(String, 'thermal_command_in', 10)
        self.current_pose = None
        self.marker_array_pub = self.create_publisher(MarkerArray, '/heat_sources', 10)
        # Subscribers to get feedback/status from other nodes
        self.nav_status_sub = self.create_subscription(String, 'navigation_status_out', self.nav_status_callback, 10)
        self.shooting_status_sub = self.create_subscription(String, 'shooting_status_out', self.shooting_status_callback, 10)
        self.r2autonav_status_sub = self.create_subscription(String, 'r2autonav_status_out', self.r2autonav_status_callback, 10)
        self.thermal_sub = self.create_subscription(String, 'thermal_status_out', self.thermal_status_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_topic_callback, 10)

        #Initializations of other variables
        self.current_pose = None
        self.marker_array = MarkerArray()
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose') #Action client for heat navigation goal
        self.radius_of_search = 0.5  
        self.heat_distance_threshold = 0.4 #0.3-0.5 is a good range for the distance threshold
        self.heat_source_array = [] #Used for recording the location of heat sources
        # Uncomment the below if you want to test ramp detection code
        """
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        """
        input_thread = threading.Thread(target=self.wait_for_start_command) #For reading entered command in terminal
        input_thread.daemon = True  # Ensure the thread exits when the program exits
        input_thread.start()

    # Uncomment the below if you want to test ramp detection code
    """
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
    """
    def wait_for_start_command(self):
        while True:
            user_input = input("Enter 'r' to start navigation: ").strip().lower()
            if user_input == 'r':
                twist_msg = Twist()
                twist_msg.linear.x = 0.1
                self.cmd_vel_pub.publish(twist_msg)
                sleep(5) #Ensure occupancy grid is updated through robot movement before navigation starts
                twist_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                start_frontier_navigation(self)
                break
            if user_input == 'pass': # For testing individual subsections
                break



#ros2 topic pub /navigation_command_in std_msgs/msg/String "{data: 'START'}"
    def nav_status_callback(self, msg):
        if msg.data == "GOAL_REACHED":
            self.get_logger().info('Frontier Navigation Goal completed! Starting Thermal Detection...')
            sleep(1)
            start_thermal(self)
        elif msg.data == "r2autonav": #Backup requested if exploration failed multiple times
            start_r2autonav(self) 

    def r2autonav_status_callback(self, msg):
        if msg.data == "GOAL_REACHED":
            self.get_logger().info('R2AutoNav Navigation Goal completed! Starting Thermal Detection...')
            start_thermal(self)
            
    def odom_topic_callback(self, msg):
        #self.get_logger().info('Received odometry data.')
        self.current_pose = msg.pose.pose
        #self.get_logger().info(f"Current pose: {self.current_pose.position.x}, {self.current_pose.position.y}, {self.current_pose.position.z}")

#IThermal Subsection Test command: ros2 topic pub /thermal_status_out std_msgs/msg/String "{data: "HEAT_SOURCE_DETECTED"}"
 
    def thermal_status_callback(self,msg):
        if msg.data == "HEAT_SOURCE_DETECTED":
            self.get_logger().info('Heat source detected!')
            if self.current_pose:
                if not self.is_duplicate_heat_source(self.current_pose): #Check if heat source is already recorded 
                    self.get_logger().info('Heat source seeking!')
                    heat_goal_point = self.project_point(self.current_pose)
                    self.navgoal_pub(heat_goal_point, self.current_pose)
                else:
                    self.get_logger().warn("Heat source already detected in this direction.")
                    start_frontier_navigation(self)
        
        elif msg.data == "NO_HEAT_SOURCE_FOUND":
            self.get_logger().info('Heat source not detected!')
            start_frontier_navigation(self)
                     
    def is_duplicate_heat_source(self, current_pose):
        # Project a point 0.4m (or whatever the value of heat_distance_threshold is) ahead in the direction the robot is facing
        projected_x, projected_y = self.project_point(current_pose)
        for heat_source in self.heat_source_array:
            # Calculate distance between the projected point and the heat source
            distance = math.sqrt(
                (projected_x - heat_source.position.x) ** 2 +
                (projected_y - heat_source.position.y) ** 2
            )

            # Check if the projected point is within the radius of an already detectet heat source
            if distance < self.radius_of_search:
                return True  # Heat source detected near the projected point

        return False  # No heat source detected near the projected point
    
    def project_point(self, current_pose):
        # Calculate the projected point based on the current pose and distance
        current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)
        projected_x = current_pose.position.x + self.heat_distance_threshold * math.cos(current_yaw)
        projected_y = current_pose.position.y + self.heat_distance_threshold * math.sin(current_yaw)
        return projected_x, projected_y

    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to yaw (rotation around the Z-axis)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp) 
   
#Shooting subsection test command: ros2 topic pub /shooting_status_out std_msgs/msg/String "{data: 'SHOOTING_COMPLETED'}"
    def shooting_status_callback(self, msg):
        if msg.data == "SHOOTING_COMPLETED":
            self.get_logger().info('Shooting task completed!')
            self.publish_marker(self.current_pose) #Create rviz marker at the current pose
            self.heat_source_array.append(self.current_pose) #Store current pose in the heat source array
            self.get_logger().info(f"New heat source recorded at: {self.current_pose.position.x}, {self.current_pose.position.y}")
            sleep(1)
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

    #Untested ramp detection code (No longer in use as frontier navigation can scale ramp, but still kept for future reference)
    """
    def ramp_detection(self):
        self.get_logger().info('Ramp detection start')
        if self.laser_range.size != 0:
            # Check if both sides of the front part of the robot are close to walls (so a narrow space)
            # However, the flaw here is that the ramp itself cannot be detected as a wall
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
    """                
    def navgoal_pub(self, projected_pose, current_pose):
        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = projected_pose[0]
        goal_msg.pose.pose.position.y = projected_pose[1]
        goal_msg.pose.pose.orientation = current_pose.orientation
        
        # Send the goal to the action server
        heat_goal_status = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        heat_goal_status.add_done_callback(self.goal_response_callback) #Callback to goal acceptance

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback #Added for testing

    def goal_response_callback(self, heat_goal_status):
        goal_handle = heat_goal_status.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.heat_distance_threshold -= 0.1 
            if self.heat_distance_threshold >= 0.3: #If distance threshold is decreased too much, that implies heat source is very close to robot.
                start_thermal(self)
            else:
                start_shooting(self)
            return

        self.get_logger().info("Goal accepted by Nav2.")
        result = goal_handle.get_result_async()
        result.add_done_callback(self.goal_result_callback) #Callback to goal result
    
    def goal_result_callback(self, goal_status):
        try:
            result = goal_status.result().result #for testing
            status = goal_status.result().status


            if status == GoalStatus.STATUS_SUCCEEDED:  # Goal succeeded
                self.get_logger().info("Goal reached successfully!")
                self.heat_distance_threshold = 0.4
                sleep(1)
                start_shooting(self)

            else:
                self.get_logger().warn(f"Goal ended with failure status: {status}") #For troubleshooting using status codes
                start_thermal(self) #If goal ends with failure, this could mean that the robot might have moved somewhere else accidentally before failing so thermal is started again

        except Exception as e:
            self.get_logger().error(f"Exception in goal_result_callback: {e}")



def main(args=None):
    rclpy.init(args=args)
    # Create the coordinator node
    coordinator_node = CoordinatorNode()
    # Keep it spinning
    rclpy.spin(coordinator_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
