#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String
import numpy as np
import cv2
import time


class SelfDriving_Car():
    angle = 0
    acc = 90
    dist_ultrasonic = 0.0
    traffic_signals = ""
    stop_signal = False
    green_traffic_light = False
    yellow_traffic_light = False
    red_traffic_light = False
    speed_limit_100 = False
    speed_limit_80 = False
    speed_limit_70 = False
    speed_limit_50 = False
    speed_limit_30 = False
    speed_limit_20 = False
    no_right_turn_sign = False 
    no_left_turn_sign = False
    drift_sign = False
    no_parking = False
    pedestrian_crossing = False
    priority_to_oncoming_traffic_sign = False
    dangerous_curve_right_P = False
    dangerous_curve_left_P = False
    person = False
    cars = False

    def __init__(self):
        #rospy.Subscriber('/control_angle_cnn', Int8, self.callback_angle_cnn)  
        #rospy.Subscriber('/ros_yolo_sf/traffic_signals', String, self.callback_traffic_signals)  
        #rospy.Subscriber('/sonar_dist', Float32, self.callback_ultrasonic_sensor) 
        self.commands = rospy.Publisher('/sf_control_picar', Int8MultiArray, queue_size = 2)
        self.my_msg = Int8MultiArray()  
        self.publisher(self.acc, 35)

    def callback_angle_cnn(self, data_angle):
        aux_angle = int(data_angle.data)
        if (self.angle != aux_angle):
            self.angle = aux_angle
            self.rules()
        

    def callback_traffic_signals(self, signals_detected):
        aux_traffic_signals = str(signals_detected.data)
        if(self.traffic_signals != aux_traffic_signals):
            self.traffic_signals = aux_traffic_signals
            self.rules()
    
    def callback_ultrasonic_sensor(self, data_angle):
        aux_dist_ultrasonic = data_angle.data
        if(self.dist_ultrasonic != aux_dist_ultrasonic):
            self.dist_ultrasonic = aux_dist_ultrasonic
            self.rules()

    def calulate_pwm(self, speed_limit):
        pwm = int(((30/100)*speed_limit)+70)
        if(pwm < 70):
            pwm = 0
        return pwm

    def get_traffic_signals(self):
        aux_traffic_signals_iterable = self.traffic_signals.split("-")
        for traffic_sign in aux_traffic_signals_iterable:
            print(traffic_sign)
            if(traffic_sign == "Alto_R"):
                self.stop_signal = True
            else:
                self.stop_signal = False

            if(traffic_sign == "Semaforo_verde"):
                self.green_traffic_light = True
            else:
                self.green_traffic_light = False
                
            if(traffic_sign == "Semaforo_amarillo"):
                self.yellow_traffic_light = True
            else:
                self.yellow_traffic_light = False

            if(traffic_sign == "Semaforo_rojo"):
                self.red_traffic_light = True
            else:
                self.red_traffic_light = False

            if(traffic_sign == "Vel_max_100_R"):
                self.speed_limit_100 = True
            else:
                self.speed_limit_100 = False

            if(traffic_sign == "Vel_max_80Km_R"):
                self.speed_limit_80 = True
            else:
                self.speed_limit_80 = False

            if(traffic_sign == "Vel_max_70Km_R"):
                self.speed_limit_70 = True
            else:
                self.speed_limit_70 = False

            if(traffic_sign == "Vel_max_50Km_R"):
                self.speed_limit_50 = True
            else:
                self.speed_limit_50 = False

            if(traffic_sign == "Vel_max_30Km_R"):
                self.speed_limit_30 = True
            else:
                self.speed_limit_30 = False

            if(traffic_sign == "Vel_max_20Km_R"):
                self.speed_limit_20 = True
            else:
                self.speed_limit_20 = False

            if(traffic_sign == "Vuelta_prohibida_der_R"):
                self.no_right_turn_sign = True
            else:
                self.no_right_turn_sign = False 

            if(traffic_sign == "Vuelta_prohibida_izq_R"):
                self.no_left_turn_sign = True
            else:
                self.no_left_turn_sign = False

            if(traffic_sign == "Derrape_R"):
                self.drift_sign = True
            else:
                self.drift_sign = False

            if(traffic_sign == "No_estacionar_R"):
                self.no_parking = True
            else:
                self.no_parking = False

            if(traffic_sign == "Paso_peatonal_R"):
                self.pedestrian_crossing = True
            else:
                self.pedestrian_crossing = False

            #if(traffic_sign ==):
            #else:
            #priority_to_oncoming_traffic_sign = False

            if(traffic_sign == "Curva_peligrosa_der_P"):
                self.dangerous_curve_right_P = True
            else:
                self.dangerous_curve_right_P = False

            if(traffic_sign == "Curva_peligrosa_izq_P"):
                self.dangerous_curve_left_P = True
            else:
                self.dangerous_curve_left_P = False

            if(traffic_sign == "Persona"):
                self.dangerous_curve_right_P = True
            else:
                self.dangerous_curve_right_P = False

            if(traffic_sign == "Carro"):
                self.dangerous_curve_left_P = True
            else:
                self.dangerous_curve_left_P = False

    def rules(self):
        print("Conduction Rules")
        self.get_traffic_signals()
        if(self.dist_ultrasonic < 20.0):
            if (self.cars):
                self.publisher(0, 0)
        elif(self.person):
            self.publisher(0, 0)
        elif(self.stop_signal):
            self.publisher(0, 0)
        elif(self.no_right_turn_sign):
            self.publisher(0, -35)
        elif(self.no_left_turn_sign):
            self.publisher(0, 35)
        elif(self.red_traffic_light):
            self.publisher(0, 0)
        elif(self.yellow_traffic_light):
            self.publisher(85, self.angle)
        elif(self.green_traffic_light):
            self.publisher(self.acc, angle)
        else:
            self.publisher(self.acc, angle)
        

    def publisher(self, acc, angle):
        print("Hay msg")
        self.my_msg.data = [1, int(acc), int(angle)]
        print(self.my_msg.data)
        self.commands.publish(self.my_msg)
    

def init_node():
    rospy.init_node("ros_rules_angle_acc")
    ri = SelfDriving_Car()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    init_node()
        