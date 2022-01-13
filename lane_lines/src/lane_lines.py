#!/usr/bin/env python3
import rospy
import math
import numpy as np
import pandas as pd
import cv2 
import os
import pickle
import time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

class LaneLines():
    enabled_selfdriving = True

    def __init__(self):
        rospy.Subscriber('/fisheye_correction/image/compressed', CompressedImage, self.callback)  
        self.pub_CompressedImage = rospy.Publisher('/lane_lines/image/compressed', CompressedImage, queue_size = 2)
        self.pub_Enabled_SD = rospy.Publisher('/lane_lines/enabled_sd', Bool, queue_size = 2)
    
    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        edges = self.detect_edges(img)
        cropped_edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        #Put if
        line_segment_image = self.display_lines(img, line_segments)
        lane_lines = self.average_slope_intercept(img, line_segments)
        if(lane_lines == []):
            self.enabled_selfdriving = False
            self.publisher_en_sd()
        else:
            self.enabled_selfdriving = True
            lane_lines_image = self.display_lines(img, lane_lines)
            self.publisher(lane_lines_image)

    def publisher_en_sd(self):
        time.sleep(0.3)
        if(self.enabled_selfdriving == False):
            msg = Bool()  
            msg = False
            self.pub_Enabled_SD.publish(msg)
    
    def publisher(self, img_lane_lines):
        image_compressed = CompressedImage()  
        image_compressed.header.stamp = rospy.Time.now()
        image_compressed.format = "jpeg"
        image_compressed.data = np.array(cv2.imencode('.jpg', img_lane_lines)[1]).tostring()
        self.pub_CompressedImage.publish(image_compressed)
    
    def detect_edges(self, frame):
        # filter for yellow lane lines
        frame = cv2.GaussianBlur(frame,(5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([30, 60, 60])
        upper_yellow = np.array([60, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # detect edges
        edges = cv2.Canny(mask, 200, 400)

        return edges

    def region_of_interest(self, canny):
        height, width = canny.shape
        mask = np.zeros_like(canny)
        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(canny, mask)

        return masked_image

    def length_of_line_segment(self, line):
        x1, y1, x2, y2 = line
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def detect_line_segments(self, cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                        maxLineGap=4)

        if line_segments is not None:
            for line_segment in line_segments:
                print('detected line_segment:')
                print("%s of length %s" % (line_segment, self.length_of_line_segment(line_segment[0])))

        return line_segments
    
    def display_lines(self, frame, lines, line_color=(255, 0, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            print('No line_segment segments detected')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    print('skipping vertical line segment (slope=inf): %s' % line_segment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        print('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

        return lane_lines

    def steer(lane_lines):
            logging.debug('steering...')
            if len(lane_lines) == 0:
                print("No se detecta carril...")
            elif len(lane_lines) == 1:
                print("Tomando curva...")
            elif len(lane_lines) == 2:
                print("Linea recta")
    
    

def main():
    rospy.init_node("lane_lines")
    #rospy.spin()
    ri = LaneLines()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()