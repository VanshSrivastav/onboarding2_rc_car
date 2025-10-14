import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json

'''
This receives a remote input.
ie. controller input is made on laptop, then WSL machine receives input 
    now once WSL receives input we publish to the topic
    rc car should in theory have access to the published data
    

'''
class JoystickReceiverNode(Node):
    def __init__(self):
        super().__init__('joystick_receiver_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tcp_ip = "0.0.0.0"
        self.tcp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.tcp_ip, self.tcp_port))
        self.sock.listen(1)
        self.get_logger().info(f"Listening for joystick data on TCP port {self.tcp_port}")

    def run(self):

        conn, addr = self.sock.accept()
        self.get_logger().info(f"Connection from {addr}") # logs the received connection

        buffer = "" # initialize the buffer

        while rclpy.ok():
            data = conn.recv(1024)
            if not data:
                break
            buffer += data.decode('utf-8')
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
            try:
                joy = json.loads(data.decode('utf-8'))
                twist = Twist()
                twist.linear.x = joy.get('y_axis', 0.0)
                twist.angular.z = joy.get('x_axis', 0.0)
                self.pub.publish(twist)
                self.get_logger().info(f'Recevied input {joy}')

            except Exception as e:
                self.get_logger().warn(f"Failed to parse joystick data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickReceiverNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
