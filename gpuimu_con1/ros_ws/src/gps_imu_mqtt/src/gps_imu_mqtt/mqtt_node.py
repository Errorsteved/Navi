import rclpy
from rclpy.node import Node
from gps_imu_msgs.msg import GpsImuData
import json
import paho.mqtt.client as mqtt
import threading
import time

class GpsImuMqttBridge(Node):
    def __init__(self):
        super().__init__('gps_imu_mqtt_bridge')

        self.mqtt_broker = '192.168.1.185'  # 改为PC IP
        self.mqtt_port = 1883
        self.mqtt_topic = 'vehicle/gps_imu'

        self.mqtt_client = mqtt.Client()
        self.mqtt_connected = False

        # 启动连接线程
        self.connect_thread = threading.Thread(target=self.connect_mqtt_loop, daemon=True)
        self.connect_thread.start()

        self.subscription = self.create_subscription(
            GpsImuData,
            'gps_imu_data',
            self.listener_callback,
            10
        )

        self.get_logger().info('GPS-IMU MQTT Bridge Node initialized.')

    def connect_mqtt_loop(self):
        while not self.mqtt_connected:
            try:
                self.get_logger().info(f'尝试连接 MQTT Broker {self.mqtt_broker}:{self.mqtt_port} ...')
                self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
                self.mqtt_client.loop_start()  # 启动后台处理线程
                self.mqtt_connected = True
                self.get_logger().info('MQTT 连接成功')
            except Exception as e:
                self.get_logger().warn(f'MQTT连接失败: {e}，5秒后重试')
                time.sleep(5)

    def listener_callback(self, msg: GpsImuData):
        if not self.mqtt_connected:
            self.get_logger().warn('MQTT 未连接，跳过本次发布')
            return

        payload = {
            "utc_time": msg.utc_time,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "num_satellites": msg.num_satellites,
            "altitude": msg.altitude,
            "heading_true": msg.heading_true,
            "heading_magnetic": msg.heading_magnetic,
            "speed_knots": msg.speed_knots,
            "speed_kmph": msg.speed_kmph,
            "accel_x": msg.accel_x,
            "accel_y": msg.accel_y,
            "accel_z": msg.accel_z,
            "gyro_x": msg.gyro_x,
            "gyro_y": msg.gyro_y,
            "gyro_z": msg.gyro_z,
            "roll": msg.roll,
            "pitch": msg.pitch,
            "yaw": msg.yaw
        }

        try:
            self.mqtt_client.publish(self.mqtt_topic, json.dumps(payload))
            self.get_logger().info('发布GPS-IMU数据到MQTT成功')
        except Exception as e:
            self.get_logger().warn(f'发布失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsImuMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()