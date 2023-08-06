import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class IntPublisher(Node):
    def __init__(self):
        super().__init__("int_publisher")
        self.int_publisher = self.create_publisher(Int32, "int_msg", 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.send_msg)

        self.msg_pub = Int32()

    # ↓0.01秒ごとに実行
    def send_msg(self):
        number = 1
        self.msg_pub.data = number
        self.int_publisher.publish(self.msg_pub)

def main(args=None):
    rclpy.init(args=args)
    pub = IntPublisher()
    rclpy.spin(pub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()