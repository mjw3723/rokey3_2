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

## 차선 인식 알고리즘 흐름도 

<pre lang="markdown"> ``` ------> | brightness |  ------>  | 히스토그램 평활화 | ------>  | Guassian Blur | ------>  | HSV | ------> | morphology | ``` </pre>


