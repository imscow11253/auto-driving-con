# 제6회 국민대학교 자율주행경진대회 예선 참가 자료
</br>

2023에 실시되었던 제6회 국민대학교 자율주행경진대회 예선전에 참가하여 푼 3개의 과제에 대한 코드이다. 아쉽게도 본선에 진출하지 못했다. 문제는 대회 보안 상 공개할 수 없어 당시 우리 팀이 풀었던 파이썬 코드만 있다. 
</br></br></br>

---

</br>
***1번 과제*** : 
</br>자동차 객체로부터 전방, 좌측, 우측, 좌측 대각선, 우측 대각선으로부터의 센서 값을 받아와 목적지까지 장애물에 부딫히지 않고 자율주행할 수 있도록 하는 과제이다. 센서 값은 사물까지의 거리 값이다. 
</br>센서 값의 변화를 통해 코너 상황인지, 전방 장애물 상황인지를 분류하여 해당 상황에 맞는 핸들 조향각과 차량 속도를 조절했다. 
</br>주요 해결점은 센서 값에 따른 핸들 조향각과 차량 속도를 어느정도의 비율로 설정할 것인가이다. 임계값을 변화시키면서 시뮬레이션을 해보고 가장 적절한 값을 찾는 방식으로 진행했다. 

---

</br>
***2번 과제*** : 
</br>3D 환경의 시뮬레이터 환경에서 자동차 객체 정면 카메라로부터 실시간 영상 데이터를 분석해서 자율주행하여 목적지까지 도달하도록 하는 과제이다. 
</br>실시간 영상을 파이썬으로 분석하여 차선을 분류했다. 한 프레임마다 관심영역 설정, 버드아이뷰 적용, 슬라이딩 윈도우를 통해 차선을 추출하여 중앙값의 변화에 따라 핸들 조향각과 차량 속도를 조절하도록 했다. 
</br>주요 해결점은 코너, 오르막,내리막 상황에서의 차선을 얼마나 잘 인식할 것인가이다. 코너 시에는 시뮬이레이터 영상에서 차선 하나가 사라져서, 그리고 오르막,내리막에서는 두 차선이 동시에 사라져서 차선 인식이 어려웠다. if문 분기를 통해 예외처리를 해줌으로써 해결했으나 완벽하지 못하다.

---

</br>
***3번 과제***
</br>2D 환경의 UI에서 차량 객체를 특정 목적지에 주차시키는 과제이다. 똑같이 핸들 조향각과 차량 속도를 조절하면서 차량을 제어하면 된다.
</br>목적지 점과 차량 시작점 사이의 planning을 해서 차량의 이동경로를 구하고(= 선을 긋고) 차량의 현재 백터 값과 이동경로 백터 값이 동일해질 때까지 핸들 조향각을 이동경로 벡터 방향으로 값을 적용해주었다.
</br>주요 해결점은 위의 방식으로 진행하면 차량의 이동경로가 곡선으로 부드럽게 이동하는게 아니라 한 번에 큰 값이 적용되어서 곡선으로 만들어지지 않는다. 그래서 기준값을 정해두고 차량의 백터 값과 이동 경로 벡터값 크기의 합이 일정하도록 해서 차량의 벡터에 이동 경로 벡터를 계속 연산해주면, 곡선과 같은 형태로 차량의 이동이 가능해진다. 