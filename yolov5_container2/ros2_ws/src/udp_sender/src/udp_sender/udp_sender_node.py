import rclpy
from rclpy.node import Node
from yolov5_vehicle_detector_msgs.msg import DetectedObjects
import socket
import json

class UdpSender(Node):
    def __init__(self):
        super().__init__('udp_sender_node')
        self.subscription = self.create_subscription(
            DetectedObjects,
            '/yolov5/detected_objects',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.udp_ip = "192.168.1.185"
        self.udp_port = 8000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def listener_callback(self, msg):
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

        self.sock.sendto(json_data.encode(), (self.udp_ip, self.udp_port))
        self.get_logger().info(f'Sent UDP message: {json_data}')

def main(args=None):
    rclpy.init(args=args)
    udp_sender = UdpSender()
    rclpy.spin(udp_sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
