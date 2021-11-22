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
    def __init__(self):
        self.session = tf.compat.v1.keras.backend.get_session()
        self.model = load_model('/home/isaac/pesos_redes/steer.h5', compile=False)
        rospy.Subscriber('/fisheye_correction/image/compressed', CompressedImage, self.callback)  
        self.pub = rospy.Publisher('/control_angle_cnn', Int8MultiArray, queue_size = 2)
        self.my_msg = Int8MultiArray()  

    def convert_angle_and_acceleration(self, cnn_predict):
        CNN_angle = cnn_predict[0]
        CNN_acceleration = cnn_predict[1]
        #Convert angle predict to angle for servomotor
        if(CNN_angle <= 0):
            angle = CNN_angle * 35
            angle = int(abs(angle)) #turn left
        else:
            angle = CNN_angle * -35
            angle = int(angle) #turn right

        #Convert accelaration predict to acceleration for motor
        acceleration = 0
        if(CNN_acceleration < -0.99):
            acceleration = 0
        else:
            acceleration = CNN_acceleration + 1
            acceleration = int(((acceleration * 30) / 2)+70)
        return angle, acceleration

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        with self.session.graph.as_default():
            tf.compat.v1.keras.backend.set_session(self.session)
            predict = self.model.predict([image.reshape(-1,160,120,1)])[0]
            print(predict)
            motor_predict = self.convert_angle_and_acceleration(predict)
            print(motor_predict)
            #time.sleep(0.4)
            self.publisher(motor_predict)
        
    def publisher(self, motor_predict):
        self.my_msg.data = [motor_predict[0],motor_predict[1]]
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