#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import message_filters
import numpy as np
from ultralytics import YOLO
import time

class RgbdYoloNode(Node):
    def __init__(self):
        super().__init__('rgbd_yolo_node')
        self.bridge = CvBridge()
        
        # Declare parameter for model path
        self.declare_parameter('model_path', '/home/u22/turtlebot/yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('target_class_id', 32)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('sync_slop', 0.12)
        self.declare_parameter('inference_imgsz', 416)
        self.declare_parameter('inference_iou', 0.45)
        self.declare_parameter('max_det', 20)
        self.declare_parameter('inference_device', 'cpu')
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('publish_empty', True)
        self.declare_parameter('log_timing_interval_sec', 5.0)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.target_class_id = self.get_parameter('target_class_id').get_parameter_value().integer_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value
        self.inference_imgsz = self.get_parameter('inference_imgsz').get_parameter_value().integer_value
        self.inference_iou = self.get_parameter('inference_iou').get_parameter_value().double_value
        self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
        self.inference_device = self.get_parameter('inference_device').get_parameter_value().string_value
        self.process_every_n_frames = max(1, self.get_parameter('process_every_n_frames').get_parameter_value().integer_value)
        self.publish_empty = self.get_parameter('publish_empty').get_parameter_value().bool_value
        self.log_timing_interval_sec = self.get_parameter('log_timing_interval_sec').get_parameter_value().double_value

        self._frame_count = 0
        self._timing_window_count = 0
        self._timing_window_cb_ms = 0.0
        self._timing_window_inf_ms = 0.0
        self._last_timing_log_t = time.time()
        
        # 加载 YOLO 模型
        self.model = YOLO(model_path) 
        
        #相机内参 (通过订阅 /camera/camera_info 动态获取)
        self.camera_info_received = False

        #订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        #创建两个独立的订阅器
        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        #对齐彩色流与深度流 
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=sync_queue_size, slop=sync_slop)
        self.ts.registerCallback(self.sync_callback)

        self.target_pose_pub = self.create_publisher(PoseArray, '/vision/target_poses', 10)
        self.get_logger().info(
            f"RGB-D YOLO 时间对齐节点已启动: color={color_topic}, depth={depth_topic}, "
            f"conf={self.confidence_threshold}, class_id={self.target_class_id}, "
            f"slop={sync_slop}, queue={sync_queue_size}, imgsz={self.inference_imgsz}, "
            f"device={self.inference_device}, process_every_n_frames={self.process_every_n_frames}")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]  # fx
            self.fy = msg.k[4]  # fy
            self.cx = msg.k[2]  # cx
            self.cy = msg.k[5]  # cy
            self.camera_info_received = True
            self.get_logger().info(f"相机内参已更新: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def sync_callback(self, color_msg, depth_msg):
        cb_t0 = time.perf_counter()

        # Validate camera info received before processing
        if not self.camera_info_received:
            self.get_logger().warn("Camera info not received yet, skipping frame")
            return

        self._frame_count += 1
        if (self._frame_count % self.process_every_n_frames) != 0:
            return
        
        # Convert ROS images to numpy matrices
        cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        # Gazebo depth output may be 16UC1 (mm) or 32FC1 (m), so keep original encoding.
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Perform YOLO inference on the color image
        inf_t0 = time.perf_counter()
        results = self.model(
            cv_color,
            verbose=False,
            conf=self.confidence_threshold,
            iou=self.inference_iou,
            imgsz=self.inference_imgsz,
            max_det=self.max_det,
            device=self.inference_device,
            classes=[int(self.target_class_id)]
        )
        inf_ms = (time.perf_counter() - inf_t0) * 1000.0

        points = []
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) != int(self.target_class_id):
                    continue

                # Obtain the center pixel in RGB image coordinates.
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_rgb = int((x1 + x2) / 2.0)
                v_rgb = int((y1 + y2) / 2.0)

                # Map RGB pixel coordinates to depth image coordinates.
                color_h, color_w = cv_color.shape[:2]
                depth_h, depth_w = cv_depth.shape[:2]
                sx = float(depth_w) / float(color_w)
                sy = float(depth_h) / float(color_h)

                u = int(round(u_rgb * sx))
                v = int(round(v_rgb * sy))

                # Clip to depth image bounds
                u = int(np.clip(u, 0, depth_w - 1))
                v = int(np.clip(v, 0, depth_h - 1))
                
                depth_raw = cv_depth[v, u]

                if depth_msg.encoding == '16UC1':
                    depth_m = float(depth_raw) / 1000.0
                elif depth_msg.encoding == '32FC1':
                    depth_m = float(depth_raw)
                else:
                    self.get_logger().warn(f"Unsupported depth encoding: {depth_msg.encoding}")
                    continue

                if not np.isfinite(depth_m) or depth_m <= 0.0:
                    continue

                Z_c = depth_m

                # Reproject with scaled intrinsics that match depth resolution.
                fx_eff = self.fx * sx
                fy_eff = self.fy * sy
                cx_eff = self.cx * sx
                cy_eff = self.cy * sy

                # 针孔相机模型反投影
                X_c = (u - cx_eff) * Z_c / fx_eff
                Y_c = (v - cy_eff) * Z_c / fy_eff

                points.append([X_c, Y_c, Z_c])

        if points:
            self.publish_target_poses(points, color_msg.header)
        else:
            self.get_logger().debug("No valid ball detections in this frame")
            if self.publish_empty:
                self.publish_target_poses([], color_msg.header)

        cb_ms = (time.perf_counter() - cb_t0) * 1000.0
        self._timing_window_count += 1
        self._timing_window_cb_ms += cb_ms
        self._timing_window_inf_ms += inf_ms
        now_t = time.time()
        if (now_t - self._last_timing_log_t) >= self.log_timing_interval_sec and self._timing_window_count > 0:
            avg_cb = self._timing_window_cb_ms / self._timing_window_count
            avg_inf = self._timing_window_inf_ms / self._timing_window_count
            self.get_logger().info(
                f"Timing window: samples={self._timing_window_count}, avg_cb={avg_cb:.1f}ms, avg_inf={avg_inf:.1f}ms")
            self._timing_window_count = 0
            self._timing_window_cb_ms = 0.0
            self._timing_window_inf_ms = 0.0
            self._last_timing_log_t = now_t

    def publish_target_poses(self, points, header):
        pose_array = PoseArray()
        pose_array.header = header

        for p in points:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.target_pose_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(points)} target poses")

def main(args=None):
    rclpy.init(args=args)
    node = RgbdYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()