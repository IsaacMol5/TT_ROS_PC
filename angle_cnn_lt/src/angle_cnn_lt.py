#!/usr/bin/env python3
import tensorflow as tf
from tensorflow.keras.models import load_model
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8MultiArray
import numpy as np
import cv2
import time

class Angle_Acc_CNN():
    angle_acc_cnn = '/home/isaac/pesos_redes/steer.h5'
    angle_cnn = '/home/isaac/pesos_redes/lane_navigation.h5'

    def img_preprocess(self, image):
        height, _, _ = image.shape
        image = image[int(height/2):,:,:]  # remove top half of the image, as it is not relevant for lane following
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)  # Nvidia model said it is best to use YUV color space
        image = cv2.GaussianBlur(image, (5,5), 0)
        image = cv2.resize(image, (200,66)) # input image size (200,66) Nvidia model
        image = image / 255 # normalizing
        return image

    def __init__(self):
        self.session = tf.compat.v1.keras.backend.get_session()
        self.model = load_model(self.angle_cnn, compile=False)
        rospy.Subscriber('/fisheye_correction/image/compressed', CompressedImage, self.callback)  
        self.pub = rospy.Publisher('/sf_angle_control_picar/angle_command', Int8MultiArray, queue_size = 2)
        self.my_msg = Int8MultiArray()  

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        preprocessed = self.img_preprocess(image_np)
        X = np.asarray([preprocessed])
        with self.session.graph.as_default():
            tf.compat.v1.keras.backend.set_session(self.session)
            steering_angle_predict = self.model.predict(X)[0]
            print(steering_angle_predict)
            #time.sleep(0.4)
            self.publisher(steering_angle_predict)
        
    def publisher(self, steering_angle_predict):
        self.my_msg.data = [2, int(steering_angle_predict[0])]
        self.pub.publish(self.my_msg)


def init_node():
    rospy.init_node("ros_tf_angle_acc")
    #rospy.spin()
    ri = Angle_Acc_CNN()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    init_node()