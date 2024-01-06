#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 윈도우에 조건 설정하여 변수를 적게 만듦
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2
import rospy, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

# 슬라이딩 윈도우의 크기 설정을 위한 변수
# 슬라이딩 윈도우의 크기는 (x, y) : (window_margin * 2, window_margin)
window_margin = 40

# src, dst => ROI 내에서 bird_eye_view를 적용시킬 좌표 설정
src = np.array([
    [50, 0],
    [600, 0],
    [639, 70],
    [0, 70]
], dtype = np.float32)

dst = np.array([
    [0, 0],
    [639, 0],
    [639, 479],
    [0, 479]
], dtype = np.float32)

#=============================================
# 시점 변환 함수
#=============================================
def bird_eye_view(roi, zero_roi):
    global Height, Width
    bird_mat = cv2.getPerspectiveTransform(src, dst)
    roi_bird = cv2.warpPerspective(roi, bird_mat, (640, 480))
    zero_roi_bird = cv2.warpPerspective(zero_roi, bird_mat, (640, 480))
    return roi_bird, zero_roi_bird

#=============================================
# 관심 영역 설정
#=============================================
def ROI_Process(img):
    roi = img[350 : 460, 0 : 639]
    roi_y, roi_x, _ = roi.shape
    
    roi_blur = cv2.GaussianBlur(roi, (0, 0), 2)
    roi_hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    low_bound = (0, 0, 150)
    up_bound  = (255, 255, 255)

    zero_roi = cv2.inRange(roi_hsv, low_bound, up_bound)
    return roi, zero_roi

#=============================================
# 슬라이딩 윈도우 차선 인식
#=============================================
def sliding_window(roi, zero_roi):
    # nonzero 개수(차선일 수 있는 부분)의 최소값 설정
    lower_threshold = 300

    roi_width  = zero_roi.shape[1]
    roi_height = zero_roi.shape[0]

    # 왼쪽, 오른쪽 차선의 디폴트 값 0, roi_width
    left_center = 0
    right_center = roi_width - 1

    # 슬라이딩 윈도우에서 차선으로 인식될 수 있는 왼쪽, 오른쪽 차선의 x배열
    left_x = []
    right_x = []

    # ROI 내에서 y좌표를 달리하여 슬라이딩 윈도우 적용
    for idx in range(0, roi_height - window_margin, window_margin):
        left_idx, right_idx = -1, -1
        
        #=============================================
        # 왼쪽 구역 슬라이딩 윈도우 적용
        # 왼쪽 끝에서 ROI의 가운데로 진행하되, 차선을 제외한 중앙 요소들을 배제하기 위하여
        # roi_width // 2 - 200으로 설정 (약 x좌표 120정도)
        #=============================================
        for left in range(0, roi_width // 2 - 200):
            left_area = zero_roi[idx:idx + window_margin, left:left + window_margin * 2]
           
            #=============================================
            # nonzero 즉, 차선 요소들의 개수가 최소 임계치보다 높다면
            # 차선일 가능성이 높기 때문에, 왼쪽 차선 x좌표 배열인 left_x에 좌표인
            # left를 추가해주고 함수 종료
            #=============================================
            if cv2.countNonZero(left_area) > lower_threshold:
                left_idx = left
                break
        # left와 동일
        # 오른쪽 끝에서 ROI의 끝 - 120, 520정도로 x좌표를 축소하여 슬라이딩 윈도우 적용
        for right in range(roi_width - 1, roi_width - 120, -1):
            right_area = zero_roi[idx:idx + window_margin, right - window_margin * 2:right]
            if cv2.countNonZero(right_area) > lower_threshold:
                right_idx = right
                break
        
        #=============================================
        # left_idx와 right_idx가 존재하는 경우
        # 최소 임계치보다 nonzero값이 높은 좌표가 인식된 경우
        # 그 좌표에는 슬라이딩 윈도우를 적용시킴
        #=============================================
        if(left_idx != -1):
            cv2.rectangle(roi, (left_idx, idx), 
            (left_idx + window_margin * 2, idx + window_margin), (0, 255, 0))
            left_x.append(left_idx)
        if(right_idx != -1):
            cv2.rectangle(roi, (right_idx - window_margin * 2, idx),
            (right_idx, idx + window_margin), (0, 255, 0))
            right_x.append(right_idx)
    #=============================================
    # 왼쪽과 오른쪽에서 차선일 가능성이 높은 좌표가 인식되는 경우
    # left_center와 right_center는 해당 배열의 평균값으로 함
    #=============================================
    if(left_x):
        left_center = sum(left_x) // len(left_x) 
    if(right_x):
        right_center = sum(right_x) // len(right_x)

    mid_center = (left_center + right_center) // 2

    #=============================================
    # 직선이지만 어떠한 이유로 다른 직선이 인식이 되지 않는 경우를 가정
    # left_center가 직선으로 인식되는 경우, right_center가 직선으로 인식되는 경우
    # 두 경우 모두 직선으로 인식하여 mid_center를 중앙값으로 정의함
    #=============================================
    if (left_center < 10 and left_center > 0) or (abs(right_center - roi_width) < 10 and right_center < roi_width - 1):
        mid_center = roi_width // 2
   
    return mid_center

#=============================================
# 터미널에서 Ctrl-c 키입력이로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge()
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
#=============================================
def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image, img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    #=========================================
    # 메인 루프
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서
    # "이미지처리 + 차선위치찾기 + 조향각 결정 + 모터토픽 발행"
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():

        # 이미지 처리를 위해 카메라 원본 이미지를 img에 복사 저장한다.
        img = image.copy()
        Height = img.shape[0]
        Width  = img.shape[1]

        # 전송받은 이미지에서 관심 영역 설정
        roi, zero_roi = ROI_Process(img)
        # 관심 영역 이미지에 버드아이뷰를 적용시켜 슬라이딩 윈도우를 적용시키기 용이한 형태로 변경
        roi_bird, zero_bird = bird_eye_view(roi, zero_roi)
        # ROI + 버드아이뷰 이미지에 슬라이딩 윈도우를 적용시켜 차선의 중앙값 받아옴
        x_center = sliding_window(roi_bird, zero_bird)
        
        # 이미지의 중앙과 슬라이딩 윈도우에서 받은 center값이 큰 차이가 나지 않은 경우는 
        # angle값을 0으로 설정하여 직선처럼 움직이도록 함
        if abs(Width // 2 - x_center) < 3:
            angle = 0
        else:
            angle = x_center - Width // 2

        cv2.circle(img, ((x_center),roi.shape[0]//2), 5, (255, 0,0), -1,cv2.LINE_AA)
        cv2.circle(img, (roi.shape[1] // 2,roi.shape[0]//2),5,(0,0,255),-1,cv2.LINE_AA)

        cv2.imshow("CAM View", img)
        cv2.waitKey(1)

        # 주행 속도를 결정
        speed = 20

        #=============================================
        # 조향각 결정
        # x_center - Width // 2와 -50 ~ 50의 조향각은 차이가 있기 때문에 
        # 기존에 구한 angle값을 1.7로 나누어 적절한 조향각으로 보정
        # 또한, 과한 angle값은 적절하지 않을 가능성이 높기 때문에 angle값을 0으로 설정
        #=============================================
        angle = int(angle / 1.7)
        if angle > 50:
            angle = 0
        
        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함
# start() 함수가 실질적인 메인 함수임.
#=============================================
if __name__ == '__main__':
    start()