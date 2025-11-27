import requests
import time

def create_terminal(key, sid, name, desc=None, props=None):
    url = "https://tsapi.amap.com/v1/track/terminal/add"
    payload = {
        "key": key,
        "sid": sid,
        "name": name
    }

    if desc:
        payload["desc"] = desc
    if props:
        payload["props"] = props

    response = requests.post(url, data=payload)
    result = response.json()

    if result.get("errcode") == 10000:
        tid = result["data"]["tid"]
        print(f"终端创建成功，tid: {tid}")
        print("\n")
        print(result)
        return tid
    else:
        print(f"创建失败: {result.get('errmsg')}，详细: {result.get('errdetail')}")
        return None


if __name__ == "__main__":
    your_key = "9a83cbeedf6922956225"
    your_sid = 1048245
    terminal_name = f"jetson_001"
    terminal_desc = "jetson终端设备"
    terminal_props = None  # json.dumps({"plate_number": "ABC123"})

    create_terminal(your_key, your_sid, terminal_name, terminal_desc, terminal_props)
