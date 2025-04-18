#!/home/totoro/miniconda3/envs/graspness/bin/python
import os
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
# GN_DIR = "/home/totoro/workspaces/grasp/graspness_unofficial"
# sys.path.append(GN_DIR)
# from stream_infer import stream_process, stream_inference, stream_visualize

class GraspNetProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        # 订阅同步的RGB和Depth图像
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        
        # 订阅相机内参
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        self.camera_params = None

    def image_callback(self, rgb_msg, depth_msg):
        # 转换为OpenCV格式
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        
        if self.camera_params is not None:
            # 调用GraspNet推理
            self.run_graspnet(rgb_image, depth_image, self.camera_params)

    def cam_info_callback(self, msg):
        # 提取内参矩阵 [fx, fy, cx, cy]
        self.camera_params = {
            'fx': msg.K[0],
            'fy': msg.K[4],
            'cx': msg.K[2],
            'cy': msg.K[5]
        }

    def run_graspnet(self, rgb, depth, intrinsics):
        # 对齐深度图到RGB图像
        aligned_depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

        # 确保对齐后的深度图与RGB图像尺寸一致
        assert aligned_depth.shape[:2] == rgb.shape[:2], "Aligned depth and RGB image dimensions do not match."

        # 使用对齐后的深度图
        depth = aligned_depth

        print('+++++++++++++++++++++++++++++')
        print('RGB Image Type: , Shape:', rgb.dtype, rgb.shape)
        print('Depth Image Type: , Shape:', depth.dtype, depth.shape)
        print('Camera Intrinsics:', intrinsics)
        '''
        RGB Image Type: , Shape: uint8 (480, 640, 3)
        Depth Image Type: , Shape: uint16 (720, 1280)
        '''

        # data_dict = stream_process(depth, rgb, 1280.0, 720.0, intrinsics)
        # stream_visualize(data_dict, stream_inference(data_dict), False)
        # rospy.signal_shutdown('GraspNet processing complete.')
        # 保存RGB和深度图像
        rgb_filename = os.path.join('/home/totoro/workspaces/grasp/graspness_unofficial/doc/gazebo', 'color_image.png')
        depth_filename = os.path.join('/home/totoro/workspaces/grasp/graspness_unofficial/doc/gazebo', 'depth_image.png')

        # 保存RGB图像
        cv2.imwrite(rgb_filename, rgb)
        cv2.imwrite(depth_filename, depth)

        print(f"Saved RGB image to {rgb_filename}")
        print(f"Saved Depth image to {depth_filename}")
        rospy.signal_shutdown('GraspNet processing complete.')

if __name__ == '__main__':
    rospy.init_node('graspnet_processor')
    processor = GraspNetProcessor()
    rospy.spin()
