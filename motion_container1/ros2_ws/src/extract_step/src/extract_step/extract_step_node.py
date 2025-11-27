import os
import json
import threading
import time

import rclpy
from rclpy.node import Node
from extract_step_msgs.msg import VehicleCommand
from ament_index_python.packages import get_package_share_directory

# 导入 step_receiver 中的 TCP 接收函数
from extract_step.steps_receiver import start_file_server
# 导入 tripstatus_mqtt
from extract_step.tripstatus_mqtt import TripStatusMQTT

class StepPublisher(Node):
    def __init__(self):
        super().__init__('step_publisher')
        self.publisher_ = self.create_publisher(VehicleCommand, 'vehicle_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.active = True  # 控制是否继续发布
        self.last_json_file = None
        
        # 获取保存路径
        self.save_dir = os.path.join(
            get_package_share_directory('extract_step'), 'saved_steps'
        )
        os.makedirs(self.save_dir, exist_ok=True)

        # 启动 TCP 接收线程（后台）
        self.get_logger().info("启动文件接收服务...")
        self.server_thread = threading.Thread(
            target=start_file_server,
            args=('0.0.0.0', 9000),
            daemon=True
        )
        self.server_thread.start()

        # 启动 MQTT 监听
        self.get_logger().info("启动 MQTT 监听 /tripstatus ...")
        self.mqtt_listener = TripStatusMQTT(
            broker_ip='192.168.1.185',  # 替换为 broker IP
            cancel_callback=self.stop
        )
        self.mqtt_listener.start()

    def stop(self):
        self.get_logger().info("收到取消指令，停止路径发布")
        msg = VehicleCommand()
        msg.command = VehicleCommand.STOP
        self.publisher_.publish(msg)
        self.active = False  # 停止发布，但不取消 timer

    def find_latest_json(self):
        """查找最近的步骤文件"""
        files = [
            f for f in os.listdir(self.save_dir)
            if f.startswith("steps_") and f.endswith(".json")
        ]
        if not files:
            return None
        files.sort(key=lambda f: os.path.getmtime(os.path.join(self.save_dir, f)), reverse=True)
        return os.path.join(self.save_dir, files[0])

    def timer_callback(self):

        filename = self.find_latest_json()
        if not filename:
            self.get_logger().info("等待步骤 JSON 文件中...")
            return

        # 如果收到新 JSON 文件，重启路径发布
        if filename != self.last_json_file:
            self.get_logger().info(f"检测到新 JSON 文件 {filename}，恢复路径发布")
            self.active = True
            self.last_json_file = filename

        if not self.active:
            self.get_logger().info("已暂停路径发布，等待新 JSON")
            return

        try:
            with open(filename, 'r', encoding='utf-8') as f:
                steps_data = json.load(f)
            if steps_data:
                self.get_logger().info(f"读取到 {filename}，持续发布 forward 指令")
                msg = VehicleCommand()
                msg.command = VehicleCommand.FORWARD
                self.publisher_.publish(msg)

                # 只要能成功读取一次，就恢复 active
                self.active = True
        except Exception as e:
            self.get_logger().error(f"读取 JSON 文件失败: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = StepPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
