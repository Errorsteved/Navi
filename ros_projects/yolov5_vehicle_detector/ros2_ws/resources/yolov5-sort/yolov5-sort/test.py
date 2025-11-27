from models.common import DetectMultiBackend
import torch

model = DetectMultiBackend('yolov5s.pt', device='cpu')
stride = model.stride  # 模型下采样步长（通常是 32）
print(f"Model stride: {stride}")
print(f"Recommended input size: {stride * 20} = {stride * 20} (e.g. 640x640)")
dummy_input = torch.zeros(1, 3, 640, 640)
output = model(dummy_input)
print("ONNX-compatible input shape accepted.")
