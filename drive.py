#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultrasonicData = [0.0, 0.0, 0.0, 0.0, 0.0]

# 회전하게 되는 두 대각선 길이의 차
offset = 25

def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data 
    
# 회전 상수
def rotateConst():
    #지정한 offset 값보다 두 대각선 거리 차가 크면 회전할 수 있도록 회전 상수 부여
    if abs(diffdiagonal) > offset:
        return int(diffdiagonal*0.3)
    else: # offset보다 작으면 회전 안 함
        return 0
    
rospy.init_node('driver')
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

xycar_msg = xycar_motor()

# 초기값
angle = 0
speed = 50
while not rospy.is_shutdown():
    xycar_msg.angle = angle
    xycar_msg.speed = speed
    motor_pub.publish(xycar_msg)
    
    # 대각선 길이의 차가 일정 수준 이상이면 angle값 조정
    diffdiagonal = ultrasonicData[3]-ultrasonicData[1] # 두 대각선 거리 차
    # print(ultrasonicData[0], ultrasonicData[4])
    # print(ultrasonicData[2])
    
    
    #산술기하평균 느낌으로 코너링하고 장애물 지점 구분해 봄
    if(ultrasonicData[1]+ultrasonicData[3] > 350 and ultrasonicData[1]*ultrasonicData[3] >= 25000 ): #코너링 
        angle = int(rotateConst()*0.3)
        # int(rotateConst()*0.5)
        speed = 45
    else:
        if abs(diffdiagonal) <= offset: # 직진
            angle = 0
            speed = 50
        else: #장애물 지점
            angle += rotateConst()
            speed = 35
    