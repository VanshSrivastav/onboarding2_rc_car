import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import can

class JoystickNode(Node):
# we are creating a new node called the JoystickNode, this node will inherit from the built-in Node constructor
# we name it joystick_node

    def __init__(self):
        super().__init__('joystick_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) # 10 is the queue size, so how many messages will "buffer" if the subscriber is slow
        # now we create a publisher that will publish a "twist" message to the topic "cmd_vel"
        # the car will run the listener which will "subscribe" to the cmd_vel topic, and then will run
        # the published instruction

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
            return

        # check to see if there is a joystick detected 

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        # init the first joystick, self.joystick = pygame.joystick.Joystick(0) detects the first connected joystick
        # self.joystick.init() inits it for use
        self.get_logger().info(f"Joystick {self.joystick.get_name()} initialized")
        # just prints the joystick name, confirms that it is connected


        self.timer = self.create_timer(0.05, self.timer_callback) # reads joystick inputs every  5ms

        
    def timer_callback(self):
        pygame.event.pump()

        x_axis = self.joystick.get_axis(0) # get the axis input from the controller
        y_axis = self.joystick.get_axis(1) # get the y axis input from the controller

        msg = Twist() # init the message object to send back

        msg.linear.x = -y_axis # y is up down, but up is considered negative so we invert
        msg.angular.z = x_axis # left or right turn

        # call the create publish and then publish the msg
        self.pub.publish(msg)

        # log the input and published output
        self.get_logger().info(f"Publishing: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")




def main(args=None):

    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()