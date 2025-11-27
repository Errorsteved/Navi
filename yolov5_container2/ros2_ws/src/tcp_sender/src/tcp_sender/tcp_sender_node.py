import rclpy
from rclpy.node import Node
from yolov5_vehicle_detector_msgs.msg import DetectedObjects
import socket
import json
import time

class TcpSender(Node):
    def __init__(self):
        super().__init__('tcp_sender_node')
        self.subscription = self.create_subscription(
            DetectedObjects,
            '/yolov5/detected_objects',
            self.listener_callback,
            10)

        self.tcp_ip = "192.168.1.185"
        self.tcp_port = 8000
        self.sock = None
        self.connected = False
        self.reconnect_interval = 3.0  # 秒

        self.try_connect()

    def try_connect(self):
        """尝试连接TCP服务器"""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.connected = True
            self.get_logger().info(f"Connected to TCP server at {self.tcp_ip}:{self.tcp_port}")
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f"Connection failed: {e}. Retrying in {self.reconnect_interval} seconds...")
            time.sleep(self.reconnect_interval)
            self.try_connect()  # 递归重连（也可以用计时器异步做）

    def listener_callback(self, msg):
        if not self.connected:
            self.try_connect()

        objects = []
        for obj in msg.objects:
            object_data = {
                "class_name": obj.class_name,
                "distance_m": obj.distance_m,
                "speed_kmh": obj.speed_kmh,
                "angle_deg": obj.angle_deg,
            }
            objects.append(object_data)

        json_data = json.dumps({"objects": objects})

        try:
            self.sock.sendall(json_data.encode() + b'\n')
            self.get_logger().info(f'Sent TCP message: {json_data}')
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}, reconnecting...")
            self.connected = False
            self.try_connect()  # 发送失败也要重连

def main(args=None):
    rclpy.init(args=args)
    tcp_sender = TcpSender()
    try:
        rclpy.spin(tcp_sender)
    except KeyboardInterrupt:
        pass
    finally:
        if tcp_sender.sock:
            tcp_sender.sock.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
