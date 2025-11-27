import paho.mqtt.client as mqtt
import json
import threading
import time

latest_data = None
data_lock = threading.Lock()
client = None
canceling_trip = False

def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] 已连接，返回码: {rc}")
    client.subscribe("#")

def on_message(client, userdata, msg):
    global latest_data
    try:
        payload = msg.payload.decode('utf-8')
        # print(f"[收到消息] 主题: {msg.topic} | 内容: {payload}")
        parsed = json.loads(payload)

        with data_lock:
            latest_data = parsed

    except Exception as e:
        print(f"[错误] 处理 MQTT 消息异常: {e}")

def get_latest_data():
    with data_lock:
        return latest_data

def start_mqtt():
    global client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.reconnect_delay_set(min_delay=1, max_delay=120)

    try:
        client.connect("localhost", 1883, 60)
        print("[MQTT] 尝试连接 MQTT Broker（localhost:1883）...")
    except Exception as e:
        print(f"[错误] 无法连接 MQTT Broker: {e}")
        return
    thread = threading.Thread(
        target=lambda: client.loop_forever(retry_first_connection=True),
        daemon=True
    )
    thread.start()

def stop_mqtt():
    global client
    if client is not None:
        client.loop_stop()
        client.disconnect()
        print("[MQTT] 已停止并断开连接")
        client = None
    else:
        print("[MQTT] 客户端未运行")

def get_current_location(max_retries=5, timeout_per_try=5):
    for attempt in range(1, max_retries + 1):
        print(f"[尝试] 第 {attempt} 次等待 MQTT 数据...")

        start_mqtt()
        for _ in range(timeout_per_try * 10):  # 每次尝试最多等待 timeout_per_try 秒
            if get_latest_data() is not None:
                print("[成功] 已获取第一条 MQTT 数据")
                return True
            time.sleep(0.1)

        stop_mqtt()

    print("[失败] 多次尝试后仍未获取 MQTT 数据")
    return False

def cancel_trip():
    global canceling_trip
    if not canceling_trip:
        print("[MQTT] 收到取消行程指令，开始发送取消指令...")
        canceling_trip = True
        start_sending_cancel()

def start_sending_cancel():
    global canceling_trip
    start_time = time.time()
    while canceling_trip:
        message = {
            "command": "CANCEL"
        }
        client.publish("/tripstatus", json.dumps(message))
        print("[MQTT] 发送取消指令...")
        time.sleep(1)

        # 如果已经发送了 5 秒，则停止发送
        if time.time() - start_time >= 5:
            stop_sending_cancel()
            break

def stop_sending_cancel():
    """停止发送取消指令"""
    global canceling_trip
    canceling_trip = False
    print("[MQTT] 停止发送取消指令")
