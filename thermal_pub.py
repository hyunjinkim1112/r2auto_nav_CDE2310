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
        self.publisher_middle_mean = self.create_publisher(Float32, 'middle_cells_mean_temperature', 10)

        # Initialize I2C and sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)

        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        msg_middle_mean = Float32()

        # Extract the 9 middle cells (3x3 grid)
        middle_cells = [
            self.amg.pixels[row][col]
            for row in range(2, 5)  # Rows 3, 4, 5 (index 2, 3, 4)
            for col in range(2, 5)  # Columns 3, 4, 5 (index 2, 3, 4)
        ]

        # Compute the mean temperature of the 9 cells
        middle_mean_temp = sum(middle_cells) / len(middle_cells) if middle_cells else 0.0
        msg_middle_mean.data = middle_mean_temp

        # Publish the mean temperature of the middle 9 cells
        self.publisher_middle_mean.publish(msg_middle_mean)

        self.get_logger().info(f'Published mean temperature of middle cells: {msg_middle_mean.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
