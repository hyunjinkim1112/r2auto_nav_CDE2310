import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from time import sleep
import atexit

# Define GPIO pin numbers
in1 = 24
in2 = 23
in3 = 22
in4 = 27
en1 = 16
en2 = 17
servo = 18

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(servo, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

# PWM setup
p1 = GPIO.PWM(en1, 1000)
p1.start(25)
p2 = GPIO.PWM(en2, 1000)
p2.start(25)
p3 = GPIO.PWM(servo, 50)
p3.start(5.5)

# Ensure GPIO cleanup on exit
def cleanup_gpio():
    GPIO.cleanup()

atexit.register(cleanup_gpio)

class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.shooting_pub = self.create_publisher(String, 'shooting_status_out', 10)
        self.shooting_command_sub = self.create_subscription(String, 'shooting_command_in', self.shooting_command_callback, 10)

        self.state = "r"
        self.start_command = False
        self.get_logger().info("Motor Driver node initialized")


    def shooting_command_callback(self, msg):
        if msg.data == "START":
            self.start_command = True
            self.get_logger().info("Received START command")

    def start_shooting(self):
        while not self.start_command:
            rclpy.spin_once(self)
        self.test()
        self.start_command = False
        self.shooting_pub.publish(String(data='SHOOTING_COMPLETED'))
        
    def test(self):
        temp1 = 1
        try: 
            while True:
                # On laptop: ros2 topic pub /motor_driver std_msgs/msg/String "{data: r}"       
                if self.state == "r":
                    self.get_logger().info("Run motor command received")
                    if temp1 == 1:
                        self.get_logger().info("Run flywheel motor")
                        GPIO.output(in1, GPIO.HIGH)
                        GPIO.output(in2, GPIO.LOW)
                        GPIO.output(in3, GPIO.HIGH)
                        GPIO.output(in4, GPIO.LOW)
                        sleep(3)
                        self.get_logger().info("Motor running forward")
                        self.get_logger().info("Run servo motor")
                        # first launch
                        self.get_logger().info("first launch")
                        move_servo()
                        sleep(2)
                        # second launch 
                        self.get_logger().info("second launch")
                        move_servo()
                        sleep(4)
                        # third launch 
                        self.get_logger().info("third launch")
                        move_servo()
                        sleep(2)
                        sleep(0.5)
                        return None
                        
                    else:
                        GPIO.output(in1, GPIO.LOW)
                        GPIO.output(in2, GPIO.HIGH)
                        GPIO.output(in3, GPIO.LOW)
                        GPIO.output(in4, GPIO.HIGH)
                        self.get_logger().info("Motor running backward")
                
                elif self.state == "fly":
                    self.get_logger().info("Flywheel motor command received")
                    if temp1 == 1:
                        self.get_logger().info("Run flywheel motor")
                        GPIO.output(in1, GPIO.HIGH)
                        GPIO.output(in2, GPIO.LOW)
                        GPIO.output(in3, GPIO.HIGH)
                        GPIO.output(in4, GPIO.LOW)
                        self.get_logger().info("Flywheel motor running forward")
                    else:
                        GPIO.output(in1, GPIO.LOW)
                        GPIO.output(in2, GPIO.HIGH)
                        GPIO.output(in3, GPIO.LOW)
                        GPIO.output(in4, GPIO.HIGH)
                        self.get_logger().info("Flywheel motor running backward")
                        
                # On laptop: ros2 topic pub /motor_driver std_msgs/msg/String "{data: servo}"       
                elif self.state == "servo":
                    self.get_logger().info("Servo motor command received")
                    move_servo()
                    self.get_logger().info("Servo motor running")
                    # p3.ChangeDutyCycle(6.8)
                    # sleep(0.5)
                    # p3.ChangeDutyCycle(2.6)
                    # sleep(0.5)
                    self.get_logger().info("Servo motor command completed")

                elif self.state == "s":
                    self.get_logger().info("Stop motor")
                    GPIO.output(in1, GPIO.LOW)
                    GPIO.output(in2, GPIO.LOW)
                    GPIO.output(in3, GPIO.LOW)
                    GPIO.output(in4, GPIO.LOW)

                elif self.state == "f":
                    self.get_logger().info("Motor forward")
                    GPIO.output(in1, GPIO.HIGH)
                    GPIO.output(in2, GPIO.LOW)
                    GPIO.output(in3, GPIO.HIGH)
                    GPIO.output(in4, GPIO.LOW)
                    temp1 = 1
                elif self.state == "b":
                    self.get_logger().info("Motor backward")
                    GPIO.output(in1, GPIO.LOW)
                    GPIO.output(in2, GPIO.HIGH)
                    GPIO.output(in3, GPIO.LOW)
                    GPIO.output(in4, GPIO.HIGH)
                    temp1 = 0

                elif self.state == "l":
                    self.get_logger().info("Set speed: low")
                    p1.ChangeDutyCycle(25)
                    p2.ChangeDutyCycle(25)

                elif self.state == "m":
                    self.get_logger().info("Set speed: medium")
                    p1.ChangeDutyCycle(50)
                    p2.ChangeDutyCycle(50)

                elif self.state == "h":
                    self.get_logger().info("Set speed: high")
                    p1.ChangeDutyCycle(75)
                    p2.ChangeDutyCycle(75)

                elif self.state == "e":
                    self.get_logger().info("Exiting and cleaning up GPIO")
                    GPIO.cleanup()
                    break

                else:
                    self.get_logger().warn("Invalid command received") 
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt detected. Cleaning up...")
            GPIO.cleanup()

def move_servo():
    p3.ChangeDutyCycle(9.8)
    sleep(0.5)
    p3.ChangeDutyCycle(5.5)
    sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    #motor_driver.test()
    while True:
        motor_driver.start_shooting()
    #rclpy.spin(motor_driver)
    #motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
