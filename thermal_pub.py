import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import busio
import board
import adafruit_amg88xx

# This 'thermal_publisher' node will be run on rpi.
# It will publish the temperature data from the thermal sensor to the 'temperature' topic.
# The temperature data will be published as a Float32 message type.
# The node will also have a method to start and stop publishing the temperature data.

class TempPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        
        # Initialize I2C and thermal sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)

        # Log initialization
        self.get_logger().info("ThermalCam node initialized and thermal sensor ready.")

        # Flag to control publishing
        self.publish_temp = False
        self.timer = None

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
            msg_middle_mean = Float32()
            msg_middle_mean.data = middle_mean_temp
            self.publisher.publish(msg_middle_mean)

            self.get_logger().info(f'Mean temperature of middle cells: {middle_mean_temp:.2f}')
            return middle_mean_temp
        except Exception as e:
            self.get_logger().error(f"Error reading thermal sensor: {e}")
            return 0.0

    def start_publishing(self):
        """Start publishing temperature data."""
        self.publish_temp = True
        self.timer = self.create_timer(1.0, self.publish_temperature)

    def stop_publishing(self):
        """Stop publishing temperature data."""
        if self.timer:
            self.timer.cancel()
        self.publish_temp = False

    def publish_temperature(self):
        """Periodically publish the temperature."""
        if self.publish_temp:
            self.get_temp()

def main(args=None):
    rclpy.init(args=args)
    node = TempPublisher()

    try:
        node.start_publishing()  # Start publishing temperature data
        rclpy.spin(node)  # Keep the node alive for additional callbacks if needed
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TempPublisher node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()