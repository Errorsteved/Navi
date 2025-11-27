import time
import requests

# ===== 配置信息 =====
KEY = '9a83cbeedf6922956225'
SID = '1048245'
TID = '1279393184'
TRNAME = 'test_line_track'

def query_trajectory(key, sid, tid, trid):
    key = "9a83cbeedf6922956225"
    url = f"https://tsapi.amap.com/v1/track/terminal/trsearch?key={key}"
    params = {
        "sid": "1048245",
        "tid": "1279393184",
        "trid": trid,
        "correction": "denoise=1,mapmatch=1,attribute=0,threshold=0,mode=driving",
        "ispoints": 1,
        "page": 1,
        "pagesize": 100
    }

    response = requests.get(url, params=params)
    result = response.json()
    print(result)

    if result.get("errcode") == 10000:
        tracks = result["data"]["tracks"]
        if not tracks:
            print("没有轨迹数据")
            return

        track = tracks[0]
        print(f"轨迹 ID: {track['trid']}")
        print(f"总距离: {track['distance']} 米")
        print(f"总时长: {track['time']} 毫秒")
        print(f"轨迹点数量: {track['counts']}")

        for i, point in enumerate(track.get("points", [])):
            print(f"{i+1}. {point['location']} @ {point['locatetime']}")
    else:
        print(f"查询失败: {result.get('errmsg')} | 详细: {result.get('errdetail')}")



if __name__ == "__main__":
    query_trajectory("9a83cbeedf6922956225", "1048245", "1279393184", "20")
