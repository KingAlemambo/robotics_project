#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from second_coursework.srv import YOLODetectionFrame, YOLODetectionFrameResponse
from second_coursework.msg import YOLODetection
from yolov4 import Detector

import numpy as np
import random

class YOLOv4ROSITR:

    def __init__(self):
        self.cv_image = None
        rospy.loginfo("inside init of yolo detction service")
        self.bridge = CvBridge()

        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.image_pub =rospy.Publisher('/yolo/inpainted_image',Image,queue_size=10)
        self.yolo_detection_srv = rospy.Service('detect_frame', YOLODetectionFrame, self.yolo_detection_service)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                weights_path='/opt/darknet/yolov4.weights',
                                lib_darknet_path='/opt/darknet/libdarknet.so',
                                meta_path='/home/k22047909/ROS_Y2/ros_ws/src/second_coursework/cfg/coco.data')


    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


    def yolo_detection_service(self, request):
        rospy.loginfo("inside yolo_detection_service")
        res = YOLODetectionFrameResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            img_arr = cv2.resize(cv_copy, (self.detector.network_width(), self.detector.network_height()))
            cv_height, cv_width, _ = self.cv_image.shape
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')

                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)

                d.bbox_x = int((d.bbox_x / self.detector.network_width()) * cv_width)
                d.bbox_y = int((d.bbox_y / self.detector.network_height()) * cv_height)
                d.width = int((d.width / self.detector.network_width()) * cv_width)
                d.height = int((d.height / self.detector.network_height()) * cv_height)
                res.detections.append(d)


                image_message = self.bridge.cv2_to_imgmsg(cv_copy, encoding="bgr8")
                self.image_pub.publish(image_message)



        else:
            rospy.logwarn("No image frame detected")

        return res



if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()