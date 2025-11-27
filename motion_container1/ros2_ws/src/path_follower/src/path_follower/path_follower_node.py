import rclpy
from rclpy.node import Node
from extract_step_msgs.msg import VehicleCommand
import threading
from Rosmaster_Lib import Rosmaster
from path_follower.motion_controller import control_loop

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        self.current_command = 0
        self.is_running = True

        
        self.bot = Rosmaster()
        self.bot.create_receive_threading()

        self.subscription = self.create_subscription(
            VehicleCommand,
            'vehicle_command',  
            self.command_callback,
            10
        )

        self.control_thread = threading.Thread(
            target=control_loop,
            args=(self.bot, self.get_command, self.is_running_func)
        )
        self.control_thread.start()

        self.get_logger().info("Path follower node 已启动，等待指令...")

    def command_callback(self, msg):
        self.current_command = msg.command
        self.get_logger().info(f"收到指令: {self.current_command}")

    def get_command(self):
        return self.current_command

    def is_running_func(self):
        return self.is_running

    def stop(self):
        self.is_running = False
        self.control_thread.join()
        self.bot.set_car_motion(0, 0, 0)
        self.bot.set_pwm_servo(1, 90)
        self.get_logger().info("Path follower 停止")

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[退出] Ctrl+C 收到")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
