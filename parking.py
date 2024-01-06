#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
# 하늘색 점 tracking 경로 표시해주는 리스트
rx, ry = [0], [0]

addRlist = 0 #Trakcing경로 리스트 추가할 개수 초기화
nextroot = 0 #다음 Tracking 할 리스트 index
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================

def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry , addRlist , nextroot , checkTooClose
    rx.clear()
    ry.clear()
    #초기화
    rx, ry = [0,0], [0,0]
    checkTooClose = 0 #sx시작하는 x값이 P_ENTRY값하고 가까우면 CheckTooClose값을 통해 후진
    nextroot = 0 #Tracking에 사용되는 변수인데, planning을 누르게되면 0으로 바뀌어야 함
    
    #처음 sx값이 P_Entry값하고 비슷하다면 후진해서 값 차이를 벌린 후에 planning
    ## - 주차장하고 sx값 차이가 100이하이면 조향각(+-20)이슈 때문에 정상적인 작동이 잘 안됨.
    if abs(P_ENTRY[0]-sx < 100):
        checkTooClose = 800 # x값이 가까울 때 CheckTooClose값에 800을 넣어서 R list에 800개의 좌표로 후진할 것임 
        for j in range(checkTooClose):
            rx[0]=sx
            ry[0]=sy
            ######좌표 표시되는 값과 각도 표시가 다름! r값은 좌표로 찍히니까 각도를 좌표로 변환하려면 +pi/2해줘야함
            rx[1]=sx - math.cos(math.radians(syaw+90))
            ry[1]=sy - math.sin(math.radians(syaw+90))
            rx.append(rx[j] + rx[1] - rx[0])
            ry.append(ry[j] + ry[1] - ry[0])
        #후진을 마친 후에 그 점으로부터 경로 재탐색하기 위해서 다음 좌표값(차가 보고 있는 방향*60 벡터) 설정해주기
        #아래 코드에 bMove값 설정하는 데에 있어서 리스트 값 추가해줘야함
        rx.append(rx[checkTooClose] + math.cos(math.radians(syaw+90))*60)
        ry.append(ry[checkTooClose] + math.sin(math.radians(syaw+90))*60)
    else:
        #리스트 첫번째 값은 시작점, 두번째 값은 (시작할 때 차가 보고 있는 방향)*60 벡터
        rx[0]=sx
        ry[0]=sy
        rx[1]=sx + math.cos(math.radians(syaw+90))*60
        ry[1]=sy + math.sin(math.radians(syaw+90))*60
    
    print("Start Planning")
    #후진이 필요없는 경우는 시작지점, 후진이 필요한 경우는 후진을 마친 지점으로부터 주차장 도착까지의 거리 만큼 R list 개수 추가하기 위한 변수 설정
    addRlist = int((math.sqrt((P_ENTRY[0] - rx[checkTooClose])**2 + (P_ENTRY[1] - ry[checkTooClose])**2))) + int(checkTooClose*1.2) #tracking 리스트 개수 
    
    for i in range(addRlist):
        if checkTooClose ==0: # 후진하지 않았다면,
            bmoveX = rx[i+1] - rx[i] #현재 순간 변화율 x 벡터 값 (beforeMove)
            bmoveY = ry[i+1] - ry[i] #현재 순간 변화율 y 벡터 값 (beforeMove)
            wmoveX = P_ENTRY[0]-rx[i+1] #움직이려고 하는 x 벡터 값 (wantMove)
            wmoveY = P_ENTRY[1]-ry[i+1] #움직이려고 하는 y 벡터 값 (wantMove)
        else: # 후진한 경우
            bmoveX = rx[checkTooClose+i+2] - rx[checkTooClose+i+1] #현재 순간 변화율 x 벡터 값 (beforeMove)
            bmoveY = ry[checkTooClose+i+2] - ry[checkTooClose+i+1] #현재 순간 변화율 y 벡터 값 (beforeMove)
            wmoveX = P_ENTRY[0]-rx[checkTooClose+i+1] #움직이려고 하는 x 벡터 값 (wantMove)
            wmoveY = P_ENTRY[1]-ry[checkTooClose+i+1] #움직이려고 하는 y 벡터 값 (wantMove)
        bmoveSize = math.sqrt(bmoveX**2 + bmoveY**2) # 벡터 크기
        wmoveSize = math.sqrt(wmoveX**2 + wmoveY**2) # 움직이고자 하는 벡터 크기
        
        #(가장 최근 추가된 리스트 값 + 다음 프레임 동안 움직이는 벡터 값) 을 리스트에 추가
        if i < addRlist/2:#초반에는 급 커브가 있기 때문에 조금 더 완만한 커브를 위해서 순간 변화율의 벡터값을 2배로 줬음
            rx.append(rx[checkTooClose+i+1] + (bmoveX/bmoveSize*2*(addRlist-i)+wmoveX/wmoveSize*i)/(addRlist))
            ry.append(ry[checkTooClose+i+1] + (bmoveY/bmoveSize*2*(addRlist-i)+wmoveY/wmoveSize*i)/(addRlist))
        else:#마지막 좌표값은 P_ENTRY값이 되야하기 때문에 후반부 주행에서는 원래 의도했던 식 그대로 작성
            rx.append(rx[checkTooClose+i+1] + (bmoveX/bmoveSize*(addRlist-i)+wmoveX/wmoveSize*i)/(addRlist) ) 
            ry.append(ry[checkTooClose+i+1] + (bmoveY/bmoveSize*(addRlist-i)+wmoveY/wmoveSize*i)/(addRlist) ) 
            # + pVecX/pVecSize*i/addRlist*0.3
            # + pVecY/pVecSize*i/addRlist*0.3
        print(bmoveSize, wmoveSize)
        
    
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================

#각도 20도 이상부터는 차이가 없음 >> planning에서 멀어지면 각도를 틀어서 돌아오게 하는 정도로 코드를 짜는게 맞을거 같음
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry , addRlist , nextroot , checkTooClose 
    #초기화
    angle = np.radians(0)
    speed = 50
    angle = 0
    
    #차가 루트를 잘따라오면 다음 nextroot 값으로 바꿔주기
    #(현재 차 위치)와 (planning dot)거리의 제곱이 3000 이하이고
    # nextroot값이 전체 리스트 개수보다 -50 일 때 >>> pygame.draw.cricle에 [nextroot + 20]으로 설정했기 때문에 끝까지 설정하면 index error생김
    if (x-rx[nextroot])**2 +(y-ry[nextroot])**2 < 3000 and nextroot < addRlist - 50:  
        # +1로 하면 차량 속도 때문에 다음 좌표를 잘 못 읽음. 숫자를 크게 하면 pygame.draw.cricle이 뚝뚝 끊겨 보이기 때문에 2가 제일 적절
        nextroot += 2 
        
    if (P_ENTRY[0]-x)**2 + (P_ENTRY[1]-y)**2 > 8000 : #----시작하고 주차 전까지 plan한 루트 따라가는 경우
        pygame.draw.circle(screen,[255,130,0],[rx[nextroot+20],ry[nextroot+20]],3,2)# tracking 라인을 따라 draw
        
        moveAngle = 0 #현재 움직이고 있는 차량 각도
        
        moveX = rx[nextroot] - x #움직여야 하는 x 벡터 값
        moveY = ry[nextroot] - y #움직여야 하는 y 벡터 값
        moveSize = math.sqrt(moveX**2 + moveY**2) #벡터 크기
        
        #리스트의 다음 root 벡터가 어디로 향하는지?를 Check
        checkX = rx[nextroot+1]-rx[nextroot]
        checkY = ry[nextroot+1]-ry[nextroot]
        if checkX <= 0 and checkY >= 0 :  #다음 값이 좌측 하단으로 내려가는 방향인지 Check > 맞다면 후진
            speed = -40
            angle = 0
        else: # 좌측하단 방향으로 가는게 아니라면 
            speed = 50
            if moveSize == 0: #sin값으로 각도를 구하려다 보니 분모 값이 0이면 결정이 안됨.. 
                moveAngle = yaw #현재 차량 방향으로 움직이게 하기 위해서 움직이고자 하는 각도(moveAngle) = 현재 차량 각도(yaw)로 설정 
            else:
                moveAngle = -math.degrees(math.asin(moveY/moveSize)) #움직임 각도 (-는 데카르트 좌표계에서 y축이 뒤집힌 모양이기 때문)
            angle = (yaw - moveAngle)*10 #조향각이 작을 시에 planning root방향으로 조정이 들어갈 때 출렁임이 심해서 *10 해줬음
            
    else :#----주차하는 과정
        if (P_END[0] - x)**2 + (P_END[1] - y)**2 < 4000: # 현재 차 위치와 주차 방지턱 위치 사이 거리의 제곱이 4000 이하일 때
            speed = 0 #주차 (parking)
        else: #주차하기 전 차량 각도 맞추는 과정
            pmoveX = P_END[0]-P_ENTRY[0] #주차장 x벡터 값
            pmoveY = P_END[1]-P_ENTRY[1] #주차장 y벡터 값
            pmoveSize = math.sqrt(pmoveX**2 + pmoveY**2) # 주차장 벡터 크기
            pmoveAngle = -math.degrees(math.asin(pmoveY/pmoveSize)) #주차장 각도 (-는 데카르트 좌표계에서 y축이 뒤집힌 모양이기 때문)
            #마지막 주차 자리에서 큰 조향각이 필요해서 *50(최대 각도로 꺾게)해줬음 / 조향 각도는 (현재 차량 각도) - (주차장 각도)
            angle =  (yaw - pmoveAngle)*50
            speed = 30 #감속하면서 주차
            
        if (P_ENTRY[0] - x)**2 + (P_ENTRY[1] - y)**2 < 5000: #거의 다 도착했을 때 주차장 한 가운데 라인 그려주기 (시범 동영상 나온 부분)
            pygame.draw.line(screen, [255,255,0], P_ENTRY,P_END,4)
            
    print(addRlist , nextroot)
    drive(angle,speed)
    
    
