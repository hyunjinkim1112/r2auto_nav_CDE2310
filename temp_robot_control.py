import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time
import math

class TempRobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        # Create subscriber for temperature
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temp_callback,
            10
        )

        # Create publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variable to track the temperature and control the movement
        self.last_temp = None

        # For tracking thermal data during robot spin
        self.thermal_array = []

    def temp_callback(self, msg):
        """Callback function for receiving temperature."""
        self.last_temp = msg.data
        self.get_logger().info(f"Received temperature: {self.last_temp:.2f}")

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
            return

        self.get_logger().info(f"Highest temperature {highest_temp:.2f} found at index {highest_temp_index}.")

        # Convert the index to an angle
        # Assuming that the robot did a full 360-degree spin and we took 36 measurements (10-degree intervals)
        angle = highest_temp_index * 10  # 10 degrees between each measurement

        # Spin the robot to face the direction of the highest temperature
        self.spin_to_angle(angle)

        # Move the robot forward towards the target temperature area
        self.move_toward_target()

    def spin_to_angle(self, target_angle):
        """Spin the robot to the given target angle."""
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

    def move_toward_target(self):
        """Move the robot forward after aligning to the highest temperature direction."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Move forward with moderate speed

        # Move forward for a short time (or until the next goal is met)
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Moving toward the target area...")

        # Run for a short time (e.g., 3 seconds) or adjust this as needed
        time.sleep(3)

        # Stop the robot after moving
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Stopped moving toward the target area.")

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
        spin_duration = 2 * 3.14159 / abs(twist_msg.angular.z)  # radians / angular velocity

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
                self.get_logger().info(f'Temperature recorded: {temp:.2f}')
                next_measure_time += measure_interval

            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot after spinning
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Robot spin completed!')

        # Log thermal array
        self.get_logger().info(f"Thermal array: {self.thermal_array}")

    def get_temp(self):
        """Simulate getting temperature from the thermal sensor."""
        if self.last_temp is not None:
            return self.last_temp
        self.get_logger().warn("No valid temperature data available.")
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = TempRobotControl()

    try:
        node.spin_robot()  # Call the spin_robot method to start spinning and recording temperature
        node.control_robot()  # Call the control_robot method to move the robot based on thermal data
        rclpy.spin(node)  # Keep the node alive for additional callbacks if needed
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RobotControl node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
