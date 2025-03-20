import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

def spin_robot(self):
    self.get_logger().info('Spinning the robot 360 degrees...')
    # Create a Twist message
    twist_msg = Twist()
    twist_msg.angular.z = 2.8  # Set angular velocity (adjust as needed)
    # Calculate the duration to spin 360 degrees
    spin_duration = 2 * 3.14159 / abs(twist_msg.angular.z)  # 360 degrees in radians
    # Publish the Twist message for the calculated duration
    start_time = self.get_clock().now()
    while (self.get_clock().now() - start_time).nanoseconds / 1e9 < spin_duration:
        self.cmd_vel_pub.publish(twist_msg)
        rclpy.spin_once(self, timeout_sec=0.1)
    # Stop the robot after spinning
    twist_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(twist_msg)
    self.get_logger().info('Robot spin completed!')    
def start_frontier_navigation(self):
    self.nav_pub.publish(String(data='START'))
    self.get_logger().info('Published navigation start command.')
def start_r2autonav(self):
    self.assisted_explore_pub.publish(String(data='START'))
    self.get_logger().info('Published r2autonav start command.')
def start_shooting(self):
    self.shooting_pub.publish(String(data='start_shooting'))
    self.get_logger().info('Published shooting start command.')

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')
        
        # Publisher for controlling other nodes (e.g., navigation, shooting, etc.)
        self.r2autonav_pub = self.create_publisher(String, 'assisted_exploration_command_in', 10)
        self.nav_pub = self.create_publisher(String, 'navigation_command_in', 10)
        self.shooting_pub = self.create_publisher(String, 'shooting_command_in', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to get feedback/status from other nodes
        self.nav_status_sub = self.create_subscription(String, 'navigation_status_out', self.nav_status_callback, 10)
        self.shooting_status_sub = self.create_subscription(Bool, 'shooting_status_out', self.shooting_status_callback, 10)
        self.r2autonav_status_sub = self.create_subscription(String, 'assisted_exploration_status_out', self.r2autonav_status_callback, 10)
        # Flag to track if navigation task is completed
        self.nav_completed = False
        self.shooting_completed = False

    def nav_status_callback(self, msg):
        if msg.data == "GOAL_REACHED":
            self.get_logger().info('Frontier Navigation task completed!')
            spin_robot(self)
            start_frontier_navigation(self)
        elif msg.data == "r2autonav":
            start_r2autonav(self)

            
            
            #self.nav_completed = True
            #self.check_all_tasks_completed()

    def r2autonav_status_callback(self, msg):
        if msg.data == "GOAL_REACHED":
            spin_robot(self)
            start_frontier_navigation(self)
            


    def shooting_status_callback(self, msg):
        if msg.data:
            self.get_logger().info('Shooting task completed!')
            self.shooting_completed = True
            self.check_all_tasks_completed()

    def check_all_tasks_completed(self):
        if self.nav_completed and self.shooting_completed:
            self.get_logger().info("All tasks completed! Proceeding to next phase.")


def main(args=None):
    rclpy.init(args=args)
    
    coordinator_node = CoordinatorNode()
    # Start navigation and shooting tasks
    rclpy.spin(coordinator_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
