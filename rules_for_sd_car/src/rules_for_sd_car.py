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
    last_acc = 0
    acc = 90
    dist_ultrasonic = 30.0
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
    on_callback_traffic_signals = True
    on_callback_ultrasonic_sensor = True

    def __init__(self):
        self.commands_sf = rospy.Publisher('/sf_acc_control_picar/acc_command', Int8MultiArray, queue_size = 2)
        self.my_msg = Int8MultiArray()
        time.sleep(2)  
        #self.publisher(self.acc)
        rospy.Subscriber('/ros_yolo_sf/traffic_signals', String, self.callback_traffic_signals)  
        #rospy.Subscriber('/sonar_dist', Float32, self.callback_ultrasonic_sensor) 

    def callback_traffic_signals(self, signals_detected):
        aux_traffic_signals = str(signals_detected.data)
        if(self.traffic_signals != aux_traffic_signals and self.on_callback_traffic_signals == True):
            self.traffic_signals = aux_traffic_signals
            self.get_traffic_signals()
            self.rules()
    
    def callback_ultrasonic_sensor(self, distance):
        aux_dist_ultrasonic = distance.data
        if(self.dist_ultrasonic != aux_dist_ultrasonic and self.on_callback_ultrasonic_sensor == True):
            self.dist_ultrasonic = aux_dist_ultrasonic
            self.rules()

    def calulate_pwm(self, speed_limit):
        pwm = int(((30/100)*speed_limit)+70)
        if(pwm < 60):
            pwm = 0
            print("Need for speed :)")
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
        if(self.dist_ultrasonic < 20.0):
            if (self.cars):
                self.publisher(0)
        elif(self.person):
            self.publisher(0)
        elif(self.stop_signal):
            self.last_acc = self.acc
            self.on_callback_traffic_signals = False
            self.on_callback_ultrasonic_sensor = False
            self.publisher(0)
            time.sleep(3)
            self.acc = self.last_acc
            self.publisher(self.acc)
            self.on_callback_ultrasonic_sensor = True
            time.sleep(1.5)
            self.on_callback_traffic_signals = True
        elif(self.no_right_turn_sign):
            self.publisher(0)
        elif(self.no_left_turn_sign):
            self.publisher(0)
        elif(self.red_traffic_light):
            self.last_acc = self.acc
            self.on_callback_traffic_signals = False
            self.on_callback_ultrasonic_sensor = False
            self.publisher(0)
        elif(self.yellow_traffic_light):
            self.last_acc = self.acc
            if(self.acc - 10 >= 60):
                self.acc = self.acc -10
                self.publisher(self.acc)
            else:
                self.acc = 70
                self.publisher(self.acc)
        elif(self.green_traffic_light):
            self.acc = self.last_acc
            self.publisher(self.acc)
        elif(self.speed_limit_100):
            self.acc = self.calulate_pwm(100)
            self.last_acc = self.calulate_pwm(100)
            self.publisher(self.acc)
        elif(self.speed_limit_50):
            self.acc = self.calulate_pwm(50)
            self.last_acc = self.calulate_pwm(50)
            self.publisher(self.acc)
        else:
            self.publisher(self.acc)
        

    def publisher(self, acc):
        self.my_msg.data = [1, acc]
        print(self.my_msg.data)
        self.commands_sf.publish(self.my_msg)
    

def init_node():
    rospy.init_node("ros_rules_acc")
    ri = SelfDriving_Car()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        

if __name__ == "__main__":
    init_node()
        