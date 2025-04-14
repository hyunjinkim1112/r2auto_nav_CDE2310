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
rotatechange = 0.5 #0.1
speedchange = 0.1
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.4
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
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
        self.start_command = False
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.r2autonav_status_sub = self.create_subscription(
            String, 
            'r2autonav_command_in', 
            self.r2autocommand_in_callback, 
            10)
        
        self.r2autonav_status_pub = self.create_publisher(
            String,
            'r2autonav_status_out',
            10)


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def r2autocommand_in_callback(self, msg):
        if msg.data == "START":
            self.get_logger().info('Received START command')
            #self.start_r2autonav()
            #self.r2autonav_status_pub.publish(String(data="FINISHED"))
            self.start_command = True


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
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
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            #self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            rclpy.spin_once(self, timeout_sec=1)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            #self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def start_r2autonav(self):
        #self.pick_direction()
        self.mover()
        time.sleep(2)
        self.get_logger().info("GOAL_REACHED")
        #self.stopbot()



    def mover(self):
        #try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
        while not self.start_command:
            rclpy.spin_once(self)

        self.pick_direction()
        start_time = time.time()    
        twist_msg = Twist()
        #while rclpy.ok():
        while True:
            elapsed_time = time.time() - start_time
            #self.get_logger().error(f"{self.laser_range[front_angles]}")
            if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))
                #NEED TO CHECK LRI WHEN TURNING AGAIN
                    # if the list is not empty
                if(len(lri[0])>0):
                        # stop moving
                        twist_msg.linear.x = 0.0
                        self.publisher_.publish(twist_msg)
                        if any(index > 30 for index in lri[0]):
                            self.get_logger().info("Obstacle detected on the left. Turning right")
                            min_index = min(lri[0])
                            twist_msg.linear.x = 0.0  # Stop moving forward
                            twist_msg.angular.z = -1*rotatechange  # Turn left
                            spin_duration = (min_index*(3.14/180))/ abs(twist_msg.angular.z)
                            self.publisher_.publish(twist_msg)
                            time.sleep(spin_duration)
                            #twist_msg.linear.x = 0.2
                            #twist_msg.angular.z = 0.0
                            #self.publisher_.publish(twist_msg)
                            #time.sleep(1)
                            #twist_msg.linear.x = 0.0
                            #twist_msg.angular.z = 0.1
                            #self.publisher_.publish(twist_msg)
                            #time.sleep(spin_duration)
                            twist_msg.linear.x = speedchange
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                        elif any(index < 30 for index in lri[0]):
                            self.get_logger().info("Obstacle detected on the right. Turning left")
                            max_index = max(lri[0])
                            twist_msg.linear.x = 0.0  
                            twist_msg.angular.z = rotatechange
                            spin_duration = (max_index*(3.14/180))/ abs(twist_msg.angular.z)
                            self.publisher_.publish(twist_msg)
                            time.sleep(spin_duration)
                            #twist_msg.linear.x = 0.2
                            #twist_msg.angular.z = 0.0
                            #self.publisher_.publish(twist_msg)
                            #time.sleep(1)
                            #twist_msg.linear.x = 0.0
                            #twist_msg.angular.z = -0.1
                            #self.publisher_.publish(twist_msg)
                            #time.sleep(spin_duration)
                            twist_msg.linear.x = speedchange
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                    #self.stopbot()
                    #self.start_command = False
                    #break
            if elapsed_time > max_runtime:
                self.get_logger().info("Maximum runtime exceeded. Stopping the robot.")
                self.stopbot()
                self.start_command = False
                self.r2autonav_status_pub.publish(String(data="GOAL_REACHED"))
                break
                    
                # allow the callback functions to run
            rclpy.spin_once(self)

        #except Exception as e:
        #    print(e)
        
        # Ctrl-c detected
        #finally:
            # stop moving
        #self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    r2auto_nav = AutoNav()
    while True:
        r2auto_nav.mover()
    #r2auto_nav.odom_callback()

    # create matplotlib figure
    # plt.ion()
    # plt.show()
    #rclpy.spin(r2auto_nav)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #r2auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
