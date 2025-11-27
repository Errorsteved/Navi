import paho.mqtt.client as mqtt
import json
import threading

class TripStatusMQTT:
    def __init__(self, broker_ip, cancel_callback):
        self.broker_ip = broker_ip
        self.cancel_callback = cancel_callback
        # MQTT 客户端设置
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        try:
            self.client.connect(self.broker_ip, 1883, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"[MQTT] 连接失败: {e}")

    def on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] 已连接，返回码: {rc}")
        client.subscribe("/tripstatus")  # 订阅 /tripstatus 主题

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            print(f"[MQTT] 收到消息: {payload}")
            parsed = json.loads(payload)
            if parsed.get("command") == "CANCEL":
                print("[MQTT] 收到取消指令")
                self.cancel_callback()
        except Exception as e:
            print(f"[MQTT] 消息处理失败: {e}")
