#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
# camera image topic callback
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    #cv2.imshow('img', image)
# publish xycar_motor msg
######################################################################################
def hough(image):
    import cv2
    import numpy as np
    img = cv2.imread('line_pic.png', cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 70)

    # ROI Area
    roi_edge_img = edge_img[240:480, 0:640]
    #cv2.imshow("roi_edge_img", roi_edge_img)
    # HoughLinesP
    all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,30,30,10)
    print("Number of lines : %d" % len(all_lines))
    # draw lines in ROI area
    line_img = image.copy()
    for line in all_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 255, 0), 2)
    # cv2.imshow("roi_lines", line_img)
    # cv2.waitKey()
    # calculate slope and do filtering
    slopes = []
    new_lines = []
    for line in all_lines:
        x1, y1, x2, y2 = line[0]
        if (x2 - x1) == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        if 0.1 < abs(slope) < 10:
            slopes.append(slope)
            new_lines.append(line[0])
    print("Number of lines after slope filtering : %d" % len(new_lines))

    left_lines = []
    right_lines = []
    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]
        x1, y1, x2, y2 = Line
        if (slope < 0) and (x2 < 320):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > 320):
            right_lines.append([Line.tolist()])
    print("Number of left lines : %d" % len(left_lines))
    print("Number of right lines : %d" % len(right_lines))

    # draw right&left lines in different color
    line_img = image.copy()
    for line in left_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 0, 255), 2)
    for line in right_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 255, 255), 2)
    # cv2.imshow("left & right lines", line_img)
    # cv2.waitKey()

    # get average left-line
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
    size = len(left_lines)
    for line in left_lines:
        x1, y1, x2, y2 = line[0]
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)
        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_left = m_sum / size
        b_left = y_avg - m_left * x_avg
    if size != 0:
        x1 = int((0.0 - b_left) / m_left)
        x2 = int((240.0 - b_left) / m_left)
        x_l = (x1 + x2)//2
    else:
        x_l = 0
    cv2.line(line_img, (x1, 0+240), (x2, 240+240), (255, 0, 0), 2)

    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
    size = len(right_lines)
    for line in right_lines:
        x1, y1, x2, y2 = line[0]
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)
        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_right = m_sum / size
        b_right = y_avg - m_right * x_avg
    if size != 0:
        x1 = int((0.0 - b_right) / m_right)
        x2 = int((240.0 - b_right) / m_right)
        x_r = (x1 + x2)//2
    else:
        x_r = 640
    cv2.line(line_img, (x1, 0+240), (x2, 240+240), (255, 0, 0), 2)
    x_center = (x_l + x_r) // 2

    #cv2.imshow("detected lines", line_img)
    #cv2.waitKey()
    return x_center - 320

#########################################################################################
#################### S T O P L I N E ####################################################

# def detect_stopline_contour(cal_image, low_threshold_value):
#     blur = cv2.GaussianBlur(cal_image, (5, 5), 0)
#     m_roi, _, _ =  set_roi(blur, 400, 240, 240)
#     L, _, B = cv2.split(cv2.cvtColor(m_roi, cv2.COLOR_BGR2LAB))
#     _, lane = cv2.threshold(B, low_threshold_value, 255, cv2.THRESH_BINARY)
#     _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     for cont in contours:
#         length = cv2.arcLength(cont, True)
#         area = cv2.contourArea(cont)
#         if not ((area > 3000) and (length > 500)):
#             continue
#         if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
#             continue
#         (x, y, w, h) = cv2.boundingRect(cont)
#         center = (x + int(w/2), y + int(h/2))
#         _, width, _ = m_roi.shape
#         if 200 <= center[0] <= (width - 200):
#             cv2.rectangle(B, (x, y), (x + w, y + h), (0, 255, 0), 2)
#             print("stopline")
#             cv2.imshow("lines from B", B)
#             cv2.waitKey()
#             return 1

#     _, lane = cv2.threshold(L, 200, 255, cv2.THRESH_BINARY)
#     _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     for cont in contours: 
#         length = cv2.arcLength(cont, True)
#         area = cv2.contourArea(cont)
#         if not ((area > 3000) and (length > 500)):
#             continue
#         if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
#             continue
#         (x, y, w, h) = cv2.boundingRect(cont)
#         center = (x + int(w/2), y + int(h/2))
#         _, width, _ = m_roi.shape
#         if 300 <= center[0] <= (width - 300):
#             cv2.rectangle(L, (x, y), (x + w, y + h), (0, 255, 0), 2)
#             print("stopline")
#             cv2.imshow("lines from L", L)
#             cv2.waitKey()
#             return 1
    

#     return 0
def set_roi(frame, x_len, start_y, offset_y):
    _, width, _ = frame.shape
    start_x = int(width/2 - (x_len/2))
    end_x = int(width - start_x)
    return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

def detect_stopline(image, mid_x, low_threshold_value):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    roi = gray[400:420, mid_x-100:mid_x+100]
    _, roi_mask = cv2.threshold(roi, low_threshold_value, 255, cv2.THRESH_BINARY)
    if cv2.countNonZero(roi_mask) > 1500:
        print("stopline")
        #cv2.imshow('stop', image)
        #cv2.waitKey()
        #cv2.destroyAllWindows()
        return 1
    return 0

def calibrate_image(frame, mtx, dist, cal_mtx, cal_roi):
    height, width, _ = frame.shape
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]
    return cv2.resize(tf_image, (width, height))


Width, Height = 640, 480
mtx = np.array( [ [422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0] ])
dist = np.array( [-0.289296, 0.061035, 0.001786, 0.015238, 0.0] )



def traffic(image):
    roi = image[150:330, 200:440]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    r_mask = cv2.inRange(hsv, (170, 0, 70), (179, 255, 255))
    r_mask2 = cv2.inRange(hsv, (0, 0, 70), (5, 255, 255))
    r_mask = cv2.bitwise_or(r_mask, r_mask2)
    g_mask = cv2.inRange(hsv, (40, 0, 70), (80, 255, 255))
    if cv2.countNonZero(r_mask) > 1000:
        print("RED")
        return -1
    elif cv2.countNonZero(g_mask) > 1000:
        print("GREEN")
        return 1
    else:
        return 0




#########################################################################################

def drive(Angle, Speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed
    motor.publish(motor_msg)

def start():
    global motor
    global image
    global Width, Height
    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print "---------- Xycar ----------"
    time.sleep(3)
    is_sl = 1
    is_trf = 1
    while not rospy.is_shutdown():
        while not image.size == (Width*Height*3):
            continue
        img = image.copy()
        xx = hough(img)
        angle = xx /4
        angle = angle -10 if angle <0 else angle
        angle = 49 if angle > 40 else (-49 if angle < -49 else angle)
        speed = 5 #20 - (10 if abs(angle)>25 else 0)
        if is_sl*(detect_stopline(image, xx+320, 80)):
            drive(0, 0)
            time.sleep(3)
            is_sl = 0
        if not is_sl*is_trf:    
            trf = traffic(image)
            if trf != 0 and is_trf == 1:
                drive(trf*49, 5)
                time.sleep(2)
                is_trf = 0
            elif is_trf == 0:
                drive(0, 0)

        drive(is_sl*is_trf*angle, is_sl*is_trf*speed)

if __name__ == '__main__':
    start()