import requests

def create_service():
    url = "https://tsapi.amap.com/v1/track/service/add"
    payload = {
        "key": "9a83cbeedf6922956225",
        "name": "jetson_tracker",
        "desc": "jetson轨迹追踪"           
    }

    response = requests.post(url, data=payload)
    result = response.json()

    if result.get("errcode") == 10000:
        print(f"服务创建成功，sid: {result['data']['sid']}")
        return result['data']['sid']
    else:
        print(f"创建失败: {result.get('errmsg')}，详细: {result.get('errdetail')}")
        return None


create_service()
