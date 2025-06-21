## TurtleBot 자율주행 프로젝트


## 📌 프로젝트 개요
본 프로젝트는 **Robotis turtlebot3 waffle**과 ROS 2 Humble를 활용하여 흰색과 노란색 선으로 이루어진 라인을 따라 이동 및 팔의 움직임을 제어 구현하는 팀 프로젝트입니다.  

---

## 🛠️ 사용 환경

| 항목         | 내용                          |
|--------------|-------------------------------|
| OS           | Ubuntu 22.04 LTS              |
| ROS2 Version | ROS2 Humble       |
| 사용 로봇     | Robotis Turtlebot3 waffle  |
| 보드   | Nvidia Jetson Nano​ , OpenCR       |
| Manipulator​ | DYNAMIXEL XM Series  |
| Camera | Logi cam |

---

## 🧑‍🤝‍🧑 팀 구성

| 이름 | 역할 |
|------|------|
| 팀장 정민섭 | 전체 목표 설정, 결과 진행 회의, 차선 인식 주행 코드 작성, PD 제어 작성  |
| 팀원 문준웅 | 코드 실행파일 병합, 차선 인식 주행 코드 작성​ |
| 팀원 이경민 | 하드웨어 세팅, ArUco Marker 인식 구현, PPT 제작 |
| 팀원 최정호 | Camera calibration, Manipulator, PPT 제작 |

---

## 📅 작업 일정

| 날짜       | 작업 내용 요약                     |
|------------|------------------------------------|
| 2025.06.16 ~ 2025.06.16 | 기획안 작성, 프로젝트 기획 및 주제 선정​    |
| 2025.06.17 ~ 2025.06.17 | 로봇암의 기본 pose 세팅, 카메라 calibration ,차선 인식1차(기본)​ |
| 2025.06.18 ~ 2025.06.18  | 차선인식2차(빛, 라인트레킹, 예외처리), ArUco Macker 작업​  |
| 2025.06.19 ~ 2025.06.19 | 차선인식3차(오류 수정), Pick_n_place 구현, 발표 자료 제작​   |
| 개발 기간 | 총 4일        |

---

## 실행

- 자율주행
```sh
ros2 launch turtlebot3_autorace_mission rokey3.launch.py
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

- Manipulator​

```sh
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
ros2 run acruo_yolo acuro_detect
```
turtlebot3_ws/src/aruco_yolo/pick_n_place.py 실행


## 차선 인식 알고리즘 흐름도 

<pre lang="markdown"> ``` ------> | brightness |  ------>  | 히스토그램 평활화 | ------>  | Guassian Blur | ------>  | HSV | ------> | morphology | ``` </pre>

1. brightness - 실시간 빛의 밝기 정도를 계산
2. 히스토그램 평활화 - H,S,V 채널중 v(명도) 채널만 평활화를 적용
3. Guassian Blur - v 채널 평활화한 마스크에만 Guassian Blau를 적용하여 반사된 조명 노이즈 제거
4. HSV - 노란색 차선과 흰색 차선을 빛의 밝기 정도에 따라 영역을 다르게 주어 실시간으로 영역이 변경
5. morphology - close 연산을 사용하여 조명으로 가려진 차선을 팽창시키고 불필요한 부분 침식 적용

## 차선 유지 제어
1. 곡률이 지나치게 큰경우 이전 곡률을 사용하도록 제어
2. 이전 곡률과 현재 곡률 차이가 큰 경우 이전 곡률을 사용하도록 제어
- 이것으로 차선을 벗어나가려다가 다시 이전 값으로 돌아와 차선 유지 개선

## 개선점
1. 프로젝트 시작 때 HSV 영역을 한가지만 사용하였는데 빛의 밝기 평균정도를 계산하여 동적으로 HSV 색상 영역을 검출하여 차선 인식률 개선
2. 동적으로 색상을 검출하였지만 날씨나 조명이 날마다 달라 영역이 바뀜. </br> 이것을 개선하기 위해 히스토그램 평활화를 사용하여 빛의 변경에 대한 인식률 개선
3. 1번과 2번을 하였음에도 차선이 인식이 안되었을 경우 로봇이 차선 오인식 하여 급격하게 틀어지는 경우를 이전 곡률과 비교하여 차이가 크다면 이전 값을 그대로 사용하도록 개선
4. HSV 색 영역을 빠르게 검출하기 위해 H,S,V 를 트랙바로 조절하여 실시간으로 확인할 수 있는 GUI를 제작 ( test.py )

![스크린샷 2025-06-19 20-15-32](https://github.com/user-attachments/assets/2e048a59-d955-435a-9a63-15b459061c05)

##시연영상

https://github.com/user-attachments/assets/7b53718f-8006-4bc6-af1f-625095c656f7



