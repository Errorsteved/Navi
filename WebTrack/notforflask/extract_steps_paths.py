import json
from pathlib import Path

# 设置输入路径（修改为路径）
route_path = Path(r"E:\Navi毕设\WebTrack\saves\route_2025-05-05T14-42-39-498Z.json")
output_path = Path(r"E:\Navi毕设\WebTrack")  # 输出路径

# 确保输出目录存在
output_path.parent.mkdir(parents=True, exist_ok=True)

# 🚀 提取输入文件名后缀用于命名输出文件
route_suffix = route_path.stem.replace("route_", "")
output_filename = f"steps_{route_suffix}.json"
output_path = output_path / output_filename

# 确保输出目录存在
output_path.parent.mkdir(parents=True, exist_ok=True)

# 读取高德路径规划 JSON 文件
with route_path.open("r", encoding="utf-8") as f:
    data = json.load(f)

# 提取 steps（默认只取第一条路线）
steps = data["routes"][0]["steps"]

# 构造每段路径的结构
step_paths = []
for i, step in enumerate(steps):
    entry = {
        "step_index": i,
        "instruction": step.get("instruction", ""),
        "road": step.get("road", ""),
        "action": step.get("action", ""),
        "distance_m": step.get("distance", 0),
        "time_s": step.get("time", 0),
        "path": step.get("path", [])  # List of [lon, lat]
    }
    step_paths.append(entry)

# 保存为 JSON 文件
with output_path.open("w", encoding="utf-8") as f:
    json.dump(step_paths, f, ensure_ascii=False, indent=2)

print(f"提取完成，保存至: {output_path.resolve()}")
