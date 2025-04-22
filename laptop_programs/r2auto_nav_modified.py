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


# Author of Modified Code: Mayukh Ghosh

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.5 
speedchange = 0.1
stop_distance = 0.4
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
max_runtime = 25

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('r2auto_nav')
        self.start_command = False # Start command used for starting this program when command is received from coordinator_node
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)  
        self.laser_range = np.array([]) 
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # initialize odom variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        #Subscriber for receiving commands from coordinator_node
        self.r2autonav_status_sub = self.create_subscription(
            String, 
            'r2autonav_command_in', 
            self.r2autocommand_in_callback, 
            10)
        #Publisher for sending status to coordinator_node
        self.r2autonav_status_pub = self.create_publisher(
            String,
            'r2autonav_status_out',
            10)

        self.get_logger().info("r2autonav node initialized") #For confirming node is initialized

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def scan_callback(self, msg):
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan for nanargmax later on
        self.laser_range[self.laser_range==0] = np.nan

    def r2autocommand_in_callback(self, msg):
        if msg.data == "START":
            self.get_logger().info('Received START command')
            self.start_command = True


    # Function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # create Twist object
        twist = Twist()
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self, timeout_sec=1)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0 to stop the rotation
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to the first furthest direction detected
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # Add delay to ensure there is reliability in the movement
        time.sleep(1)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    # Main function to control all movements of the robot
    def mover(self):
        while not self.start_command:
            rclpy.spin_once(self)
        twist_msg = Twist()
        # find direction with the largest distance from the Lidar, rotate to that direction, and start moving
        self.pick_direction()
        start_time = time.time() #Start timer to keep track of how long r2auto_nav movement is running   
        twist_msg = Twist()
        while True:
            elapsed_time = time.time() - start_time #Keep checking time to ensure r2auto_nav stops running once the max runtime has been exceeded
            if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less than stop_distance
                lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                if(len(lri[0])>0): #Check if there are any values less than stop_distance
                        twist_msg.linear.x = 0.0
                        self.publisher_.publish(twist_msg) #Immediately stop the robot to prevent collison
                        if any(index > 30 for index in lri[0]): #Firstly check for obstacles on the left that have an index greater than 30
                            self.get_logger().info("Obstacle detected on the left. Turning right")
                            #Check which obstacle on the left is closest to the robot (ie, closest to 30) and create a new index value based on that to determine how long to turn
                            min_index = 30 - (min([index for index in lri[0] if index > 30]) - 30) 
                            twist_msg.linear.x = 0.0  # Stop moving forward
                            twist_msg.angular.z = -1*rotatechange  # Turn right
                            self.get_logger().info(f"min_index: {min_index}")

                            spin_duration = (abs(min_index)*(3.14/180))/ abs(twist_msg.angular.z) #Time for spinning to face new angle
                            self.publisher_.publish(twist_msg)
                            time.sleep(spin_duration)
                            twist_msg.linear.x = speedchange
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                        elif any(index <= 30 for index in lri[0]): #Now check for obstacles on the right and the center that have an index less than 30
                            self.get_logger().info("Obstacle detected on the right. Turning left")
                            #Check which obstacle on the right is closest to the robot (ie, closest to index 30) for spin duration
                            max_index = max([index for index in lri[0] if index < 30]) 
                            twist_msg.linear.x = 0.0  
                            twist_msg.angular.z = rotatechange
                            self.get_logger().info(f"max_index: {max_index}")
                            spin_duration = (abs(max_index)*(3.14/180))/ abs(twist_msg.angular.z)
                            self.publisher_.publish(twist_msg)
                            time.sleep(spin_duration)
                            twist_msg.linear.x = speedchange
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
            if elapsed_time > max_runtime: #Check if the maximum runtime has been exceeded
                self.get_logger().info("Maximum runtime exceeded. Stopping the robot.")
                self.stopbot() #Stop the robot
                self.start_command = False #Reset start command to prevent r2autonav from restarting without a new command from coordinator_node
                self.r2autonav_status_pub.publish(String(data="GOAL_REACHED")) #Let coordinator_node know r2autonav is finished
                break
                    
            # allow the callback functions to run
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)

    r2auto_nav = AutoNav()
    while True:
        r2auto_nav.mover() #keep calling mover so r2autonav is always ready for start command
    rclpy.shutdown()


if __name__ == '__main__':
    main()
