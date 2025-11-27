from flask import Flask, jsonify, render_template, request
import requests
import os
import json
from datetime import datetime
from pathlib import Path
from scripts import mqtt_listener, steps_publisher

app = Flask(__name__)

KEY = '9a83cbeedf6922956225'
SID = '1048245'
TID = '1279393184'
TRID = '20'

SAVE_DIR = 'saves'
os.makedirs(SAVE_DIR, exist_ok=True)

mqtt_started = False

@app.route('/')
def index():
    return render_template('map.html')

@app.route('/get_trajectory')
def get_trajectory():
    url = f"https://tsapi.amap.com/v1/track/terminal/trsearch?key={KEY}"
    params = {
        "sid": SID,
        "tid": TID,
        "trid": TRID,
        "correction": "denoise=1,mapmatch=1,attribute=0,threshold=0,mode=driving",
        "ispoints": 1,
        "page": 1,
        "pagesize": 100
    }

    response = requests.get(url, params=params)
    result = response.json()

    if result.get("errcode") == 10000 and result["data"]["tracks"]:
        points = result["data"]["tracks"][0].get("points", [])
        path = [[float(p["location"].split(',')[0]), float(p["location"].split(',')[1])] for p in points]
        return jsonify({"path": path})
    else:
        return jsonify({"error": result.get("errmsg", "轨迹获取失败")})


@app.route('/save_route', methods=['POST'])
def save_route():
    content = request.get_json()
    timestamp = content.get("timestamp")
    data = content.get("data", {})
    for f in os.listdir(SAVE_DIR):
        if f.startswith("route_") and f.endswith(".json"):
            os.remove(os.path.join(SAVE_DIR, f))
        if f.startswith("steps_") and f.endswith(".json"):
            os.remove(os.path.join(SAVE_DIR, f))

    filename = f"route_{timestamp.replace(':', '-').replace('.', '-')}.json"
    filepath = os.path.join(SAVE_DIR, filename)

    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        with open(os.path.join(SAVE_DIR, "current_route.txt"), "w") as f:
            f.write(filename)

        return jsonify({
            "status": "ok",
            "filename": os.path.abspath(filepath)
        })
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)})
    


@app.route("/extract_steps", methods=["POST"])
def extract_steps():
    data = request.json
    route_file_path = data.get("route_path")

    if not route_file_path:
        return jsonify({"status": "error", "message": "Missing 'route_path' in request"}), 400

    try:
        route_path = Path(route_file_path)
    except Exception as e:
        return jsonify({"status": "error", "message": f"路径格式错误: {e}"}), 400

    if not route_path.exists():
        return jsonify({"status": "error", "message": f"Route file not found: {route_path}"}), 404

    output_dir = route_path.parent
    route_suffix = route_path.stem.replace("route_", "")
    output_filename = f"steps_{route_suffix}.json"
    output_path = output_dir / output_filename

    try:
        with route_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        steps = data["routes"][0]["steps"]
        step_paths = []

        for i, step in enumerate(steps):
            entry = {
                "step_index": i,
                "instruction": step.get("instruction", ""),
                "road": step.get("road", ""),
                "action": step.get("action", ""),
                "distance_m": step.get("distance", 0),
                "time_s": step.get("time", 0),
                "path": step.get("path", [])
            }
            step_paths.append(entry)

        with output_path.open("w", encoding="utf-8") as f:
            json.dump(step_paths, f, ensure_ascii=False, indent=2)
        
        try:
            steps_publisher.send_file_to_jetson(str(output_path))
        except Exception as e:
            print(f"[错误] 文件发送失败: {e}")
            return jsonify({
                "status": "error",
                "message": f"步骤提取成功，但发送失败: {str(e)}",
                "output_file": str(output_path.resolve())
            }), 500


        return jsonify({
            "status": "ok",
            "message": "路径提取成功",
            "output_file": str(output_path.resolve()),
            "step_count": len(step_paths)
        })

    except Exception as e:
        return jsonify({"status": "error", "message": f"处理失败: {str(e)}"}), 500

@app.route("/start_mqtt", methods=["POST"])
def start_mqtt():
    global mqtt_started
    if not mqtt_started:
        mqtt_listener.start_mqtt()
        mqtt_started = True
        return jsonify({"status": "started"})
    else:
        return jsonify({"status": "already running"})

@app.route("/latest")
def latest():
    data = mqtt_listener.get_latest_data()
    if data:
        return jsonify(data)
    else:
        return jsonify({"status": "no data"}), 204
    
@app.route('/stop_mqtt', methods=['POST'])
def api_stop_mqtt():
    mqtt_listener.stop_mqtt()
    return jsonify({"status": "stopped"})

@app.route("/getlocation")
def getlocation():
    data = mqtt_listener.get_current_location()
    if data:
        return jsonify(data)
    else:
        return jsonify({"status": "no data"}), 204
    
@app.route('/canceltrip', methods=['POST'])
def cancelTrip():
    print("canceling")
    mqtt_listener.cancel_trip()
    return jsonify({"status": "stopped"})

if __name__ == '__main__':
    app.run(debug=True)
