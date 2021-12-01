#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 
from PIL import Image as Img
from PIL import ImageTk
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8MultiArray
import sys
import time
import os
import threading, time 
import tkinter as tk
from tkinter import *

os.system('xset r off')

class GUI_Car():
	label_pwm_vel_gui = 0
	pwm_vel_gui = 0
	forward_bt = 0
	backward_bt = 0
	turnRight = 0
	turnRight = 0
	acc_car = 0
	angle_car = 0
	DS_gui = 0
	ST_gui = 0
	FB_gui = 0
	comm = 0
	angle_default = 35
	my_msg = Int8MultiArray()
	def __init__(self):
		rospy.init_node("GUI_car")
		rospy.Subscriber('/fisheye_correction/image/compressed', CompressedImage, self.callback)  
		self.comm = rospy.Publisher('/car/commands', Int8MultiArray, queue_size = 2)
		self.my_msg = Int8MultiArray()
        #self.t = threading.Thread(name = "2.0", target = self.callback)
		self.on = True

		global root
		root = tk.Tk()	
		root.title('Selfdriving Car ESCOM GUI')	  
		root.geometry('1000x800') 
	
	def callback(self, data):
		global frame, angle_for_training, acc_for_training, num_video, num_frame
		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		#frame = imutils.resize(frame, width=640)
		cv2.imwrite("/home/isaac/training_angle/"+"Video"+str(num_video)+"_"+str(num_frame)+"_"+str(angle_for_training)+"_"+str(acc_for_training)+".jpg",frame)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		im = Img.fromarray(frame)
		img = ImageTk.PhotoImage(image=im)
		self.lblVideo.configure(image=img)
		self.lblVideo.image = img
        #print("Llego un frame")
		num_frame = num_frame + 1

	def publisher(self, commands):
		self.my_msg.data = [commands[0], commands[1], commands[2], commands[3]]
		self.comm.publish(self.my_msg)

	def call_forward(self, e):
		global angle_for_training, acc_for_training 
		global node
		print("Forward")
		acc = int(self.pwm_vel_gui.get())#PWM
		print(acc)
		if(self.DS_gui == 0):
			acc_for_training = acc
			self.acc_car = acc
			self.FB_gui = 0
			self.publisher([0, self.acc_car, self.angle_car, self.FB_gui])#[RC car, acc_car, angle_car, forward]
			self.DS_gui == 1

	def call_backward(self, e):
		global angle_for_training, acc_for_training 
		global node
		print("Backward")
		acc =  int(self.pwm_vel_gui.get())
		print(acc)
		if(self.DS_gui == 0):
			self.acc_car = acc
			self.FB_gui = 1
			self.publisher([0, self.acc_car, self.angle_car, self.FB_gui])#[RC car, acc_car, angle_car, backward]
			self.DS_gui == 1

	def call_up_fb(self, e): #Stop motor for back wheels
		global angle_for_training, acc_for_training 
		global node
		print("DS")
		acc_for_training = 0
		self.acc_car = 0
		self.publisher([0, 0, self.angle_car, self.FB_gui])
		self.DS_gui = 0

	def call_right(self, e):
		global angle_for_training, acc_for_training 
		global node
		print("Right")
		if(self.ST_gui == 0):
			angle_for_training = -(self.angle_default)
			self.angle_car = -(self.angle_default)
			self.publisher([0, self.acc_car, self.angle_car, self.FB_gui])#[RC car, acc_car, angle_car, backward]
			self.ST_gui == 1

	def call_left(self, e):
		global angle_for_training, acc_for_training 
		global node
		print("Left")
		if(self.ST_gui == 0):
			angle_for_training = self.angle_default
			self.angle_car = self.angle_default
			self.publisher([0, self.acc_car, self.angle_car, self.FB_gui])#[RC car, acc_car, angle_car, backward]
			self.ST_gui == 1

	def call_up_rl(self, e): #Stop servo for front wheels
		global angle_for_training, acc_for_training 
		global node
		print("ST")
		self.angle_car = 0
		angle_for_training = 0
		self.publisher([0, self.acc_car, self.angle_car, self.FB_gui])
		self.DS_gui = 0

	def loop(self):
		global root		 

		self.label_pwm_vel_gui = Label(text="Velocidad (PWM):")
		self.label_pwm_vel_gui.place(x = 450, y = 30, width=130)
		self.pwm_vel_gui = Spinbox(root, from_= 60, to = 100, increment = 10)
		#pwm_vel_gui.place_configure(x = 450,y = 150, width = 70)
		self.pwm_vel_gui.pack(pady = 50)	

		self.forward_bt = tk.Button(root, text="Adelante")
		self.forward_bt.place(x = 150, y = 50)
		self.forward_bt.bind('<ButtonPress-1>', self.call_forward)
		self.forward_bt.bind('<ButtonRelease-1>', self.call_up_fb)
		root.bind('<KeyPress-Up>', self.call_forward)
		root.bind('<KeyRelease-Up>', self.call_up_fb)

		self.backward_bt = tk.Button(root, text="Reversa")
		self.backward_bt.place(x = 150, y = 130)
		self.backward_bt.bind('<ButtonPress-1>', self.call_backward)
		self.backward_bt.bind('<ButtonRelease-1>', self.call_up_fb)
		root.bind('<KeyPress-Down>', self.call_backward)
		root.bind('<KeyRelease-Down>', self.call_up_fb)

		self.right_bt = tk.Button(root, text="Derecha")
		self.right_bt.place(x = 225, y = 90)
		self.right_bt.bind('<ButtonPress-1>', self.call_right)
		self.right_bt.bind('<ButtonRelease-1>', self.call_up_rl)
		root.bind('<KeyPress-Right>', self.call_right)
		root.bind('<KeyRelease-Right>', self.call_up_rl)

		self.left_bt = tk.Button(root, text="Izquierda")
		self.left_bt.place(x = 75, y = 90)
		self.left_bt.bind('<ButtonPress-1>', self.call_left)
		self.left_bt.bind('<ButtonRelease-1>', self.call_up_rl)
		root.bind('<KeyPress-Left>', self.call_left)
		root.bind('<KeyRelease-Left>', self.call_up_rl)

		self.lblVideo = Label(root)
		self.lblVideo.place(x = 255, y = 150)

		root.mainloop()



root = 1 
command = 1 
node = 1 
frame = 1
pwm_vel = 0
angle_for_training = 0
acc_for_training = 0
num_frame = 0
num_video = 3 #Cambiar por cada prueba de entrenamiento -------------------------------------------------------    

if __name__ == '__main__':
	root_aux = GUI_Car()
	root_aux.loop()

os.system('xset r on')
