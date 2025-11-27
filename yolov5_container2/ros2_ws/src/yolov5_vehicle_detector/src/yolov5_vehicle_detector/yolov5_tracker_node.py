import os
import sys
import torch
import math
import warnings
from collections import deque
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from yolov5_vehicle_detector.models.common import DetectMultiBackend
from yolov5_vehicle_detector.utils.general import check_img_size, non_max_suppression, scale_coords
from yolov5_vehicle_detector.utils.augmentations import letterbox
from yolov5_vehicle_detector.utils.torch_utils import select_device
from yolov5_vehicle_detector.sort_tracker.sort import Sort

from yolov5_vehicle_detector_msgs.msg import DetectedObject, DetectedObjects
from std_msgs.msg import Header

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore")

# 相机/测距参数
H = 1
alpha = 0
calib = np.array([[833, 0.0, 304],
                  [0.0, 838, 403],
                  [0.0, 0.0, 1.0]])


def draw_measure_line(H, calib, u, v, alpha):
    fy = calib[1,1]
    u0, v0 = calib[0,2], calib[1,2]
    gamma = math.degrees(math.atan((v - v0) / fy))
    beta = alpha + gamma or 1e-2
    pi = math.pi
    z = (H / math.sin(math.radians(beta))) * math.cos(math.radians(gamma))
    return z


def get_color(idx):
    idx *= 3
    return ((37 * idx) % 255,
            (17 * idx) % 255,
            (29 * idx) % 255)
# 计算水平角度
def compute_relative_angle(calib, u):
    fx = calib[0, 0]
    u0 = calib[0, 2]
    angle_rad = math.atan((u - u0) / fx)
    return math.degrees(angle_rad)  # 转换为角度

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolov5_detector_node')
        
        # 在 ROS 2 中创建一个话题发布者
        self.publisher_ = self.create_publisher(DetectedObjects, "/yolov5/detected_objects", 10)
        
        # 保存每个目标的历史轨迹信息，用于速度估计和轨迹可视化
        self.trajectories = {}
        # 轨迹长度上限（单位是帧），可根据帧率调整（例如 25FPS × 0.6s ≈ 15）
        self.max_trajectory_length = 15

        # 声明参数
        self.declare_parameter('weights', 'yolov5s.engine')
        self.declare_parameter('source', 0)  # 相机
        self.declare_parameter('save_dir', './data/save')
        self.declare_parameter('vis', True)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', [640, 640])
        self.declare_parameter('conf_thre', 0.5)
        self.declare_parameter('iou_thre', 0.5)
        self.declare_parameter('merge_nms', False)
        self.declare_parameter('det_thresh', 0)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('iou_threshold', 0.3)

        # 取出参数
        w = self.get_parameter('weights').value
        src = self.get_parameter('source').value
        save_dir = self.get_parameter('save_dir').value
        self.vis = self.get_parameter('vis').value
        device = select_device(self.get_parameter('device').value)
        imgsz = tuple(self.get_parameter('imgsz').value)
        conf = self.get_parameter('conf_thre').value
        iou = self.get_parameter('iou_thre').value
        merge_nms = self.get_parameter('merge_nms').value

        det_thresh = self.get_parameter('det_thresh').value
        max_age = self.get_parameter('max_age').value
        min_hits = self.get_parameter('min_hits').value
        iou_th = self.get_parameter('iou_threshold').value

        # 初始化视频读写
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)  # 强制使用 V4L2 后端
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开视频源')
            rclpy.shutdown()
            return

        fw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        fh = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0

        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, 'output.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(save_path, fourcc, fps, (fw, fh))

        # Detector 实例
        self.detector = DetectMultiBackend(w, device=device, dnn=False)
        self.names = self.detector.names
        self.stride = self.detector.stride
        self.imgsz = check_img_size(imgsz, s=self.stride)
        self.conf, self.iou, self.merge_nms = conf, iou, merge_nms, 
        self.tracker = Sort(det_thresh=det_thresh,
                            max_age=max_age,
                            min_hits=min_hits,
                            iou_threshold=iou_th)
        self.trajectories = {}
        self.max_trajectory_length = 5
        
        # 初始化帧率计时器
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # 定时器驱动处理
        self.timer = self.create_timer(1.0/fps, self.timer_callback)

    @torch.no_grad()
    def timer_callback(self):
        ret, frame_origin = self.cap.read()
        if not ret:
            self.get_logger().warn('读帧失败，退出')
            rclpy.shutdown()
            return

        #裁剪成640x640
        frame_h, frame_w, _ = frame_origin.shape  # h=768, w=1024
        # 计算裁剪区域左上角坐标
        x_start = (frame_w - 640) // 2
        y_start = (frame_h - 640) // 2
        # 裁剪成正方形
        frame = frame_origin[y_start:y_start + 640, x_start:x_start + 640]
        # print(f"[DEBUG] Original frame_crop shape: {frame.shape}")
        # print(f"[DEBUG] Original frame shape: {frame_origin.shape}")

        # 前处理
        img = letterbox(frame, self.imgsz, stride=self.stride)[0]
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.detector.device)
        img = img.float() / 255.0
        img = img.unsqueeze(0)
        # print(f"[DEBUG] Model input shape: {img.shape}")
        # print(f"[DEBUG] self.imgsz: {self.imgsz}")

        # 预测 + NMS
        pred = self.detector(img, augment=False, visualize=False)
        pred = non_max_suppression(pred,
                                   conf_thres=self.conf,
                                   iou_thres=self.iou,
                                   agnostic=self.merge_nms)

        # 遍历检测
        vis_img = frame.copy()
        for det in pred:
            # 坐标缩放
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()

            # 跟踪目标
            online = self.tracker.update(det[:, :6].cpu(), [frame.shape[0], frame.shape[1]], self.imgsz)

            # ROS 消息初始化
            objects_msg = DetectedObjects()
            objects_msg.header = Header()
            objects_msg.header.stamp = self.get_clock().now().to_msg()
            objects_msg.header.frame_id = "camera_frame"

            for *xyxy, tid, cls in online:
                x1, y1, x2, y2 = map(int, xyxy)
                color = get_color(int(cls) + 2)
                bottom_center = (x1 + x2) // 2, y2
                z = draw_measure_line(H, calib, bottom_center[0], bottom_center[1], alpha)
                
                # 角度计算
                angle = compute_relative_angle(calib, bottom_center[0])

                # 画框 + 类别 + 距离
                cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(vis_img, f'ID:{int(tid)} {self.names[int(cls)]} {z:.2f}m {angle:+.1f}deg',
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # 初始化轨迹
                if tid not in self.trajectories:
                    self.trajectories[tid] = deque(maxlen=self.max_trajectory_length)

                trajectory_point = {
                    'bottom_center': bottom_center,
                    'zc_cam_other': z
                }
                self.trajectories[tid].appendleft(trajectory_point)

                # 画轨迹线
                for i in range(1, len(self.trajectories[tid])):
                    if self.trajectories[tid][i - 1] is None or self.trajectories[tid][i] is None:
                        continue
                    thickness = int(np.sqrt(self.max_trajectory_length / float(i + i)) * 2)
                    cv2.line(vis_img,
                            self.trajectories[tid][i - 1]['bottom_center'],
                            self.trajectories[tid][i]['bottom_center'],
                            color, thickness)

                # 计算速度
                speed_kmh = 0.0
                if len(self.trajectories[tid]) >= self.max_trajectory_length:
                    dist_last = self.trajectories[tid][0]['zc_cam_other']
                    dist_now = self.trajectories[tid][-1]['zc_cam_other']
                    t = 0.04 * (self.max_trajectory_length - 1)
                    speed = (dist_last - dist_now) / t
                    speed_kmh = abs(speed * 3.6)
                    # print(f"[速度估计] ID:{tid} 速度: {speed_kmh:.2f} km/h")

                # 显示速度
                cv2.putText(vis_img,
                            f'ID: {tid} {self.names[int(cls)]} {speed_kmh:.2f}km/h',
                            (x1, y1 - 30),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 2)

                # ROS 消息填充
                obj = DetectedObject()
                obj.id = int(tid)
                obj.class_name = self.names[int(cls)]
                obj.bbox_x = x1
                obj.bbox_y = y1
                obj.bbox_width = x2 - x1
                obj.bbox_height = y2 - y1
                obj.distance_m = z
                obj.speed_kmh = speed_kmh
                obj.angle_deg = angle
                objects_msg.objects.append(obj)

            # 发布
            self.publisher_.publish(objects_msg)
            self.get_logger().info(f"Published {len(objects_msg.objects)} detected objects.")

        # 实时帧率计算与显示
        now = self.get_clock().now().nanoseconds / 1e9
        fps = 1.0 / max(now - self.last_time, 1e-6)
        self.last_time = now
        cv2.putText(vis_img,
                    f'FPS: {fps:.2f}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2)
        
        # 可视化 & 写文件
        if self.vis:
            cv2.imshow('Detection', vis_img)
            cv2.waitKey(1)
        self.out.write(vis_img)

    def destroy_node(self):
        # 关闭资源
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
