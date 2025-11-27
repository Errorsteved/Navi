import time
import requests

KEY = '9a83cbeedf6922956225'
SID = '1048245'
TID = '1279393184'
TRNAME = 'test_line_track'

def generate_line_points(start_lng, start_lat, count=10, step=0.0001):
    points = []
    for i in range(count):
        lng = start_lng + i * step
        lat = start_lat
        timestamp = int(time.time() * 1000) + i * 1000  # 每秒一个点

        point = {
            "location": f"{lng:.6f},{lat:.6f}",
            "locatetime": timestamp,
            "speed": 5.0 + i * 0.1,
            "direction": 90,
            "height": 2.0,
            "accuracy": 3.0
        }
        points.append(point)
    return points

def upload_points(trid, points):
    key = "9a83cbeedf6922956225"
    url = f"https://tsapi.amap.com/v1/track/point/upload?key={key}"

    payload = {
        "sid": "1048245",
        "tid": "1279393184",
        "trid": trid,
        "points": points
    }

    response = requests.post(url, json=payload).json()
    print(response)

    if response.get("errcode") == 10000:
        print("轨迹点上传成功")
    else:
        print(f"上传失败: {response.get('errmsg')} | 详细: {response.get('errdetail')}")


if __name__ == "__main__":
    START_LNG = 116.397428
    START_LAT = 39.90923

    points = generate_line_points(START_LNG, START_LAT)
    upload_points("20", points)
