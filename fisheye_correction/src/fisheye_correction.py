#!/usr/bin/env python2.7
import rospy
import numpy as np
import pandas as pd
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import os
import pickle
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

class Fisheye():
    def __init__(self):
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.callback)  
        self.pub_CompressedImage = rospy.Publisher('/fisheye_correction/image/compressed', CompressedImage, queue_size = 2)
        self.pub_Image = rospy.Publisher('/fisheye_correction/image', Image, queue_size = 2)
        #self.my_msg = Image()  

    def undistort(self, img, balance=0.0, dim2=None, dim3=None):
        DIM=(1280, 960)
        K=np.array([[602.5428139496016, 0.0, 623.3515082490676], [0.0, 604.5859595163365, 460.6940080876433], [0.0, 0.0, 1.0]])
        D=np.array([[-0.01497899917215119], [0.094666419429798], [-0.17655778620914206], [0.091519661716245]])
        #img = cv2.imread(img)
        dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
        assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if not dim2:
            dim2 = dim1
        if not dim3:
            dim3 = dim1
        scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        new_K = np.array([[472.16234807, 0.0, 606.93132879], [0.0, 473.76339017, 451.38771026],[0.0, 0.0, 1.0]])
        #cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
        #print(new_K)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return undistorted_img

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        img_undistorsion = self.undistort(image_np)
        self.publisher(img_undistorsion)
    
    def publisher(self, img_undistorsion):
        image_compressed = CompressedImage()  
        image_compressed.header.stamp = rospy.Time.now()
        image_compressed.format = "jpeg"
        image_compressed.data = np.array(cv2.imencode('.jpg', img_undistorsion)[1]).tostring()
        self.pub_CompressedImage.publish(image_compressed)
        image = CvBridge().cv2_to_imgmsg(img_undistorsion, encoding="bgr8")
        self.pub_Image.publish(image)


def main():
    rospy.init_node("fisheye_correction")
    #rospy.spin()
    ri = Fisheye()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()