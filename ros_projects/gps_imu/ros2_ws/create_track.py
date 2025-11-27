import time
import requests

# ===== 配置信息 =====
KEY = '高德Key'
SID = 'Service ID'
TID = '设备ID'
TRNAME = 'test_line_track'

# ===== 创建轨迹 =====
def create_trajectory():
    url = "https://tsapi.amap.com/v1/track/trace/add"
    payload = {
        "key": "9a83cbeedf6922956225",
        "sid": "1048245",
        "tid": "1279393184",
        "trname": "test_track"
    }
    response = requests.post(url, data=payload).json()
    if response.get("errcode") == 10000:
        print(f"轨迹创建成功，trid: {response['data']['trid']}")
        return response['data']['trid']
    else:
        print(f"创建失败: {response}")
        return None


if __name__ == "__main__":
    create_trajectory()
