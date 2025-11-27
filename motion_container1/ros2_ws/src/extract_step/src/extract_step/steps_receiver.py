#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ament_index_python.packages import get_package_share_directory
import socket
import os
import glob

SAVE_DIR = os.path.join(get_package_share_directory('extract_step'), 'saved_steps')
os.makedirs(SAVE_DIR, exist_ok=True)

def start_file_server(host='0.0.0.0', port=9000): 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((host, port))
        server_socket.listen(1)
        print(f"[监听] 等待文件连接... {host}:{port}")

        while True:
            conn, addr = server_socket.accept()
            with conn:
                print(f"[连接成功] 来自 {addr}")
                
                
                header = conn.recv(256).decode().strip()
                filename, filesize = header.split('|')
                filesize = int(filesize)

                tmp_path = os.path.join(SAVE_DIR, filename + ".tmp")
                final_path = os.path.join(SAVE_DIR, filename)

                
                with open(tmp_path, 'wb') as f:
                    remaining = filesize
                    while remaining > 0:
                        data = conn.recv(min(4096, remaining))
                        if not data:
                            break
                        f.write(data)
                        remaining -= len(data)

                
                if remaining == 0:
                    print(f"[接收完成] 临时保存为: {tmp_path}")
                    
                    old_files = glob.glob(os.path.join(SAVE_DIR, "steps_*.json"))
                    for old_file in old_files:
                        if old_file != final_path:  
                            try:
                                os.remove(old_file)
                                print(f"[清理] 删除旧文件: {old_file}")
                            except Exception as e:
                                print(f"[警告] 删除失败: {e}")

                    
                    os.rename(tmp_path, final_path)
                    print(f"[重命名完成] {tmp_path} → {final_path}")
                else:
                    print(f"[接收失败] 仅收到部分数据，已丢弃 {tmp_path}")
                    os.remove(tmp_path)
