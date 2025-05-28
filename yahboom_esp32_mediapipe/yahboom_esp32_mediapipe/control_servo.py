import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int32


class Servo_Cotrol(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub_Servo1 = self.create_publisher(Int32, "servo_s1", 10)
        self.pub_Servo2 = self.create_publisher(Int32, "servo_s2", 10)

        self.PWMServo_X = 0
        self.PWMServo_Y = 0
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y

        self.pub_Servo2.publish(self.s2_init_angle)
        self.pub_Servo1.publish(self.s1_init_angle)

    def Send_servo_angle(self):
        self.pub_Servo2.publish(self.s2_init_angle)
        self.pub_Servo1.publish(self.s1_init_angle)
        time.sleep(0.1)


def main():
    rclpy.init()
    Servo1 = Servo_Cotrol("Servo_Cotrol")

    print("When the servo runs to the middle!")
    print("Please Kill the program according to Ctrl+C !")

    try:
        while True:
            Servo1.Send_servo_angle()
            

    except KeyboardInterrupt:
        Servo1.destroy_node()
        rclpy.shutdown()
    finally:
        Servo1.destroy_node()
        rclpy.shutdown()
