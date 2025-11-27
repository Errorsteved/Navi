import socket
import os
import time

def send_file_to_jetson(file_path, jetson_ip='192.168.1.190', jetson_port=9000, max_retries=9):
    filename = os.path.basename(file_path)
    filesize = os.path.getsize(file_path)

    for attempt in range(max_retries):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((jetson_ip, jetson_port))
                print(f"[连接成功] 第 {attempt+1} 次尝试")

                header = f"{filename}|{filesize}".encode()
                s.sendall(header.ljust(256, b' '))

                with open(file_path, 'rb') as f:
                    while chunk := f.read(4096):
                        s.sendall(chunk)

                print("[发送完成]")
                return True

        except Exception as e:
            print(f"[连接失败] 第 {attempt+1} 次尝试: {e}")
            time.sleep(1)  # 等待再试

    raise ConnectionError(f"无法连接 Jetson（尝试了 {max_retries} 次）")

