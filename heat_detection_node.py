import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import busio
import board
import adafruit_amg88xx
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import numpy as np
import math
import cmath
import time

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

rotatechange = 0.5
speedchange = 0.05

class HeatDetection(Node):
    def __init__(self):
        super().__init__('heat_detection_node')
        self.thermal_pub = self.create_publisher(String, 'thermal_status_out', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.thermal_sub = self.create_subscription(String, 'thermal_command_in', self.thermal_callback, 10)
        # Initialize I2C and thermal sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)

        # Log initialization
        self.get_logger().info("Heat Detection node initialized and thermal sensor ready.")

        # Flag to control publishing
        #self.publish_temp = False
        #self.timer = None
        self.thermal_array = []
        self.odom_subscription = self.create_subscription(
        Odometry,
        'odom',
        self.odom_callback,
        10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.start_command = False
    
    def odom_callback(self, msg):
    # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def thermal_callback(self, msg):
        #self.get_logger().info(f"Received thermal command: {msg.data}")
        if msg.data == "START":
            self.start_command = True
        elif msg.data == "HEAT_SOURCE_SEEK":
            self.move_toward_target()

    def get_temp(self):
        """
        Extract the 3x3 grid of middle cells from the thermal sensor and compute the mean temperature.
        """
        try:
            middle_cells = [
                self.amg.pixels[row][col]
                for row in range(2, 5)  # Rows 3, 4, 5 (index 2, 3, 4)
                for col in range(2, 5)  # Columns 3, 4, 5 (index 2, 3, 4)
            ]

            middle_mean_temp = sum(middle_cells) / len(middle_cells) if middle_cells else 0.0

            # Publish the mean temperature
            #msg_middle_mean = Float32()
            #msg_middle_mean.data = middle_mean_temp
            #self.publisher.publish(msg_middle_mean)

            #self.get_logger().info(f'Mean temperature of middle cells: {middle_mean_temp:.2f}')
            return middle_mean_temp
        except Exception as e:
            self.get_logger().error(f"Error reading thermal sensor: {e}")
            return 0.0
    #"""
    def spin_to_angle(self, rot_angle):
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
        self.cmd_vel_pub.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.cmd_vel_pub.publish(twist)
    """
    def spin_to_angle(self, target_angle):
        #Spin the robot to the given target angle.
        twist_msg = Twist()
        # Calculate the direction to turn
        current_angle = 0  # Assuming the robot starts facing 0 degrees (can be adjusted based on actual orientation)
        angle_diff = target_angle - current_angle
        # Ensure the robot turns in the shortest direction (clockwise or counterclockwise)
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5  # Turn in the appropriate direction
        # Spin the robot until the angle is reached
        start_time = self.get_clock().now()
        spin_duration = abs(angle_diff) / 180  # Duration to turn (based on angular velocity)
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < spin_duration:
            self.cmd_vel_pub.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        # Stop the robot after spinning
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Aligned to {target_angle} degrees.")
    """
    def start_detection(self):
        while not self.start_command:
            rclpy.spin_once(self)
        self.spin_robot()
        self.control_robot()
        self.thermal_pub.publish(String(data='HEAT_SOURCE_DETECTED'))
        self.start_command = False
        #self.thermal_pub.publish(String(data='HEAT_SOURCE_REACHED'))
        
        
    def spin_robot(self):
        """Spins the robot 360 degrees while recording temperature data from the sensor."""
        # if self.last_temp is None:
        #     self.get_logger().warn("No temperature data available, cannot spin robot.")
        #     return
        
        self.get_logger().info('Spinning the robot 360 degrees...')
        
        # Create a Twist message for spinning
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Angular velocity (adjust as needed)

        # Calculate spin duration for 360 degrees
        spin_duration = 2 *1* 3.14159 / abs(twist_msg.angular.z)  # radians / angular velocity

        # Record temperatures at intervals
        start_time = self.get_clock().now()
        measure_interval = spin_duration / 36  # Capture temperature every 10 degrees
        next_measure_time = measure_interval

        self.thermal_array = []

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < spin_duration:
            self.cmd_vel_pub.publish(twist_msg)

            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time >= next_measure_time:
                temp = self.get_temp()
                self.thermal_array.append(temp)
                #self.get_logger().info(f'Temperature recorded: {temp:.2f}')
                next_measure_time += measure_interval

            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot after spinning
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Robot spin completed!')
        time.sleep(5)

        # Log thermal array
        #self.get_logger().info(f"Thermal array: {self.thermal_array}")

    def control_robot(self):
        """Control robot movement based on the thermal array."""
        if not self.thermal_array:
            self.get_logger().warn("No temperature data available in thermal_array.")
            return

        # Find the direction with the highest temperature > 25
        highest_temp = None
        highest_temp_index = -1
        for idx, temp in enumerate(self.thermal_array):
            if temp > 25.0 and (highest_temp is None or temp > highest_temp):
                highest_temp = temp
                highest_temp_index = idx

        if highest_temp_index == -1:
            self.get_logger().info("No temperature reading higher than 25 found.")
            self.thermal_pub.publish(String(data='NO_HEAT_SOURCE_FOUND'))
            return

        self.get_logger().info(f"Highest temperature {highest_temp:.2f} found at index {highest_temp_index}.")

        # Convert the index to an angle
        # Assuming that the robot did a full 360-degree spin and we took 36 measurements (10-degree intervals)
        angle = highest_temp_index * 10  # 10 degrees between each measurement

        # Spin the robot to face the direction of the highest temperature
        self.spin_to_angle(angle)

        # Move the robot forward towards the target temperature area
        #self.move_toward_target()

    def move_toward_target(self):
        """Move the robot forward after aligning to the highest temperature direction."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Move forward with moderate speed
        # Move forward for a short time (or until the next goal is met)
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Moving toward the target area...")
        while self.get_temp() < 29:
            pass
        # Run for a short time (e.g., 3 seconds) or adjust this as needed
        #time.sleep(3)
        # Stop the robot after reaching close enough
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Stopped moving toward the target area.")
        self.thermal_pub.publish(String(data='HEAT_SOURCE_REACHED'))
    
def main(args=None):
    rclpy.init(args=args)
    node = HeatDetection()
    while True:
        node.start_detection()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
