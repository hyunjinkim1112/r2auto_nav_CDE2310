import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import busio
import board
import adafruit_amg88xx

class ThermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
        self.publisher_mean = self.create_publisher(Float32, 'mean_temperature', 10)

        # Initialize I2C and sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)

        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        msg_mean = Float32()

        # Collect all temperatures to calculate mean
        temp_values = [temp for row in self.amg.pixels for temp in row]

        # Compute mean temperature
        mean_temp = sum(temp_values) / len(temp_values) if temp_values else 0.0
        msg_mean.data = mean_temp

        # Publish the mean temperature
        self.publisher_mean.publish(msg_mean)

        self.get_logger().info(f'Published mean temperature: {msg_mean.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
