import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct 


class RC_Controller_Node(Node):
    def __init__(self):
        super().__init__('rc_car_controller')
        self.subscriber = self.create_subscription(
            Twist, # msg type
            'cmd_vel', # topic we are pulling from
            self.cmd_vel_callback, # method to do received action based on input
            10 # buffer
        )
        self.subscriber  # prevent unused variable warning
        self.get_logger().info("RC Car Controller Node started. Listening to /cmd_vel")

        # Example: initialize CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

        # Example CAN IDs for steering and drive
        self.steer_can_id = 0x100
        self.drive_can_id = 0x101


    def cmd_vel_callback(self, msg: Twist):

        throttle_mmps = int(msg.linear.x*1000)   # scale velocity to mm/s
        steering_mrad = int(msg.angular.z * 1000)


        drive_msg = can.Message(arbitration_id=self.drive_can_id,
                                data=struct.pack('<h6x', throttle_mmps),
                                is_extended_id=False)
        steer_msg = can.Message(arbitration_id=self.steer_can_id,
                                data=struct.pack('<h6x', steering_mrad),
                                is_extended_id=False)
        try:
            self.bus.send(drive_msg)
            self.bus.send(steer_msg)
            self.get_logger().info(f"Drive: {throttle_mmps} mm/s | Steer: {steering_mrad} mrad")
        except can.CanError as e:
            self.get_logger().error(f"CAN send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RC_Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
