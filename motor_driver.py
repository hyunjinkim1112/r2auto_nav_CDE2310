import RPi.GPIO as GPIO   
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy


from time import sleep


GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,1000)
p.start(25)

class Motor_Driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            durability=DurabilityPolicy.VOLATILE,   
            history=HistoryPolicy.KEEP_LAST,        
            depth=10                                
        )
        self.subscription = self.create_subscription(String, "motor
        self.state = "Stopped"

    def listener_callback(self, msg):
        self.state = msg.data

    def test(self):
        in1 = 24
        in2 = 23
        en = 25
        temp1=1
        while rclpy.ok():
            while True:
                if self.state == "r":
                    print("run")
                    if temp1 == 1:
                        GPIO.output(in1,GPIO.HIGH)
                        GPIO.output(in2,GPIO.LOW)
                        print("forward")
                    else:
                        GPIO.output(in1,GPIO.LOW)
                        GPIO.output(in2,GPIO.HIGH)
                        print("backward")
                elif self.state == "s":
                    print("stop")
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.LOW)
                elif self.state == "f":
                    print("forward")
                    GPIO.output(in1,GPIO.HIGH)
                    GPIO.output(in2,GPIO.LOW)
                    temp1=1
                elif self.state == "b":
                    print("backward")
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.HIGH)
                    temp1=0
                elif self.state == "l":
                    print("low")
                    p.ChangeDutyCycle(25)
                elif self.state == "m":
                    print("medium")
                    p.ChangeDutyCycle(50)
                elif self.state == "h":
                    print("high")
                    p.ChangeDutyCycle(75)
                elif self.state == "e":
                    GPIO.cleanup()
                    break
                else:
                    print("<<<  wrong data  >>>")
                break
     
            rclpy.spin_once(self)

            
def main(args=None):
    rclpy.init(args=args)
    motor_driver = Motor_Driver()
    motor_driver.test()
    #rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

