import argparse
import os
import numpy as np
import cv2
import torch
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.augmentations import letterbox
from utils.torch_utils import select_device
import warnings
from sort_tracker.sort import Sort
from collections import deque
import math
#import serial  # 添加串口通信库

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

warnings.filterwarnings("ignore")

parser = argparse.ArgumentParser()
# 检测参数
parser.add_argument('--weights', default=r"yolov5s.pt", type=str, help='weights path')
parser.add_argument('--source', default=1, type=int, help='camera source')  #调用相机
#parser.add_argument('--source', default=r"001.mp4", type=str, help='img or video(.mp4)path') #调用内置视频
parser.add_argument('--save', default=r"./data/save", type=str, help='save img or video path')
parser.add_argument('--vis', default=True, action='store_true', help='visualize image')
parser.add_argument('--device', type=str, default="cpu", help='use gpu or cpu')
parser.add_argument('--imgsz', type=tuple, default=(640, 640), help='image size')
parser.add_argument('--merge_nms', default=False, action='store_true', help='merge class')
parser.add_argument('--conf_thre', type=int, default=0.5, help='conf_thre')
parser.add_argument('--iou_thre', type=int, default=0.5, help='iou_thre')

# 跟踪参数
parser.add_argument('--det_thresh', type=int, default=0, help='sort thresh')
parser.add_argument('--max_age', type=int, default=30, help='keep frame')
parser.add_argument('--min_hits', type=int, default=3, help='minimum match count')
parser.add_argument('--iou_threshold', type=int, default=0.3, help='hungarian algorithm threshold')
opt = parser.parse_args()

# 高度
H = 6.5
alpha = 8  # 角度a
# 相机内参
calib = np.array([[2900, 0.0, 640],
                  [0.0, 700, 360],
                  [0.0, 0.0, 1.0]])

# 初始化串口
#ser = serial.Serial('COM5', 115200)  # 根据具体情况修改串口号和波特率

# # 权重参数
# Ws = 0.5
# Wd = 1.0
# alert_threshold = 5

def draw_measure_line(H, calib, u, v, alpha):
    alpha = alpha  # 角度a

    # 相机焦距
    fy = calib[1][1]
    # 相机光心
    u0 = calib[0][2]
    v0 = calib[1][2]

    pi = math.pi

    Q_pie = [u - u0, v - v0]
    gamma_pie = math.atan(Q_pie[1] / fy) * 180 / np.pi

    beta_pie = alpha + gamma_pie

    if beta_pie == 0:
        beta_pie = 1e-2

    z_in_cam = (H / math.sin(beta_pie / 180 * pi)) * math.cos(gamma_pie * pi / 180)

    return z_in_cam


def get_color(idx):
    idx = idx * 3
    color = ((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255)
    return color


class Detector:
    def __init__(self, device, model_path=r'./best_dist_model.pt', imgsz=(640, 640), conf=0.5, iou=0.0625,
                 merge_nms=False):

        self.pointlist = [(6, 410), (1278, 410)]

        self.device = device
        self.model = DetectMultiBackend(model_path, device=self.device, dnn=False)
        self.names = self.model.names

        self.stride = self.model.stride

        self.imgsz = check_img_size(imgsz, s=self.stride)

        self.conf = conf

        self.iou = iou

        self.merge_nms = merge_nms

        self.tracker = Sort(det_thresh=opt.det_thresh, max_age=opt.max_age, min_hits=opt.min_hits,
                            iou_threshold=opt.iou_threshold)

        self.trajectories = {}

        self.max_trajectory_length = 5

        self.id_in = 0
        self.id_in_list = []

        self.id_out = 0
        self.id_out_list = []

    @torch.no_grad()
    def __call__(self, image: np.ndarray):
        img_vis = image.copy()
        img = letterbox(image, self.imgsz, stride=self.stride)[0]
        # print(img.shape)
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        im = img.float()  # uint8 to fp16/32
        im /= 255.0
        im = im[None]
        # inference
        pred = self.model(im, augment=False, visualize=False)

        # Apply NMS
        pred = non_max_suppression(pred, conf_thres=self.conf, iou_thres=self.iou, classes=None,
                                   agnostic=self.merge_nms, max_det=1000)

        #cv2.line(img_vis, self.pointlist[0], self.pointlist[1], (0, 255, 0), 2)
        for i, det in enumerate(pred):  # detections per image
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], image.shape).round()
            online_targets = self.tracker.update(det[:, :6].cpu(), [image.shape[0], image.shape[1]], self.imgsz)
            for t in online_targets:
                speed_km_per_h = 0
                tlwh = [t[0], t[1], t[2] - t[0], t[3] - t[1]]
                tid = t[4]
                cls = t[5]
                color = get_color(int(cls) + 2)
                bottom_center = (int(tlwh[0] + tlwh[2] / 2), int(tlwh[1] + tlwh[3]))

                # 计算距离
                zc_cam_other = draw_measure_line(H, calib, int(tlwh[0] + tlwh[2] / 2), int(tlwh[1] + tlwh[3]), alpha)

                # 调试：检查标签和距离
                print(f"标签: {self.names[int(cls)]}, 距离: {zc_cam_other:.2f} m")

                # 设置颜色
                # if self.names[int(cls)] == "car":
                #     if zc_cam_other > 10:
                #         #print("距离大于10米，设置绿色")
                #         color = (0, 255, 0)  # 绿色
                #     else:
                #         #print("距离小于等于10米，设置红色")
                #         color = (0, 0, 255)  # 红色
                # # 绘制检测框
                cv2.rectangle(img_vis, (int(tlwh[0]), int(tlwh[1])),
                              (int(tlwh[0] + tlwh[2]), int(tlwh[1] + tlwh[3])), color, 2)

                # 在检测框内显示距离
                cv2.putText(img_vis, f"{zc_cam_other:.2f} m", (int(tlwh[0]), int(tlwh[1]) - 10),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 2)

                zc_cam_other = draw_measure_line(H, calib, int(tlwh[0] + tlwh[2] / 2), int(tlwh[1] + tlwh[3]), alpha)

                # if self.names[int(cls)] == "person":
                if tid not in self.trajectories:
                    self.trajectories[tid] = deque(maxlen=self.max_trajectory_length)

                trajectory_point = {
                    'bottom_center': bottom_center,
                    'zc_cam_other': zc_cam_other
                }
                self.trajectories[tid].appendleft(trajectory_point)

                # 截断轨迹长度
                if len(self.trajectories[tid]) > self.max_trajectory_length:
                    self.trajectories[tid] = self.trajectories[tid][:self.max_trajectory_length]

                for i in range(1, len(self.trajectories[tid])):

                    if self.trajectories[tid][i - 1] is None or self.trajectories[tid][i] is None:
                        continue

                    thickness = int(np.sqrt(self.max_trajectory_length / float(i + i)) * 2)
                    cv2.line(img_vis, (self.trajectories[tid][i - 1]['bottom_center']),
                             (self.trajectories[tid][i]['bottom_center']), color, thickness)

                # 调试：检查轨迹信息
                #print(f"轨迹: {self.trajectories}")

                if len(self.trajectories[tid]) >= self.max_trajectory_length:
                    dist_last = self.trajectories[tid][0]['zc_cam_other']
                    dist_now = self.trajectories[tid][-1]['zc_cam_other']
                    t = 0.04 * (self.max_trajectory_length - 1)

                    # 计算速度
                    speed = (dist_last - dist_now) / t
                    speed_km_per_h = abs(speed * 3.6)
                    # 调试：检查速度
                    print(f"速度: {speed_km_per_h:.2f} km/h")

                # 显示ID、类别、速度
                cv2.putText(img_vis, f'ID: {tid} {self.names[int(cls)]} {speed_km_per_h:.2f}km/h', (int(tlwh[0]), int(tlwh[1]) - 30),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 2)

                # # 计算Alert_Score
                # Alert_Score = Ws * speed_km_per_h + Wd * (1.0 / zc_cam_other)
                # print(f"Alert_Score: {Alert_Score:.2f}")

                # # 判断是否触发警报
                # if Alert_Score > alert_threshold:
                #     # 触发警报并发送串口消息
                #     print("警报触发!")
                #     ser.write(b'\x01')  # 发送十六进制的1
                # else:
                #     # 未触发警报发送0
                #     ser.write(b'\x00')  # 发送十六进制的0

        return img_vis


def main():
    device = select_device(opt.device)
    cap = cv2.VideoCapture(opt.source)
    save_dir = opt.save

    if not cap.isOpened():
        print("Error: Unable to open camera source.")
        return

    detector = Detector(device, opt.weights, opt.imgsz, opt.conf_thre, opt.iou_thre, opt.merge_nms)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    save_path = os.path.join(save_dir, 'output.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(save_path, fourcc, fps, (frame_width, frame_height))

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Unable to read frame from camera source.")
            break

        result_frame = detector(frame)

        if opt.vis:
            cv2.imshow('Detection', result_frame)

        out.write(result_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
