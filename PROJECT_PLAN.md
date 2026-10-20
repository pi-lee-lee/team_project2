# TurtleBot 물류창고 프로젝트 기획

## 0. 초기 구상

1. 웹소켓 + Django 사용
2. TurtleBot에 Raspberry Pi + LiDAR LDS-02 사용
3. 경로 계획은 A* 등 일반적인 알고리즘 사용
4. LiDAR로 동적 객체 추적
5. 사용 흐름
   - 맵 생성 → 맵에 물류 위치 좌표 지정
   - 웹 지도에서 위치 선택 → 로봇 이동
   - 물류 적재 후 UI에서 "적재 완료" 확인
   - 다음 물류 위치로 이동 또는 복귀

---

## 1. 기본 스택 및 연산 위치

| 항목 | 선택지 | 추천 |
|---|---|---|
| ROS 버전 | ROS 2 Humble (Ubuntu 22.04) / Jazzy (24.04) | **Humble** (TurtleBot3 공식 자료가 가장 많음) |
| 로봇 모델 | Burger / Waffle Pi | 보유 모델 기준 (적재량 Burger 약 15kg, Waffle 약 30kg) |
| 연산 위치 | Pi에서 전부 / Pi는 센서·모터, PC에서 SLAM·Nav2 | **Pi: bringup + LiDAR만**, PC 서버: Nav2·SLAM·Django |

- Pi 4에서 Nav2와 동적 객체 추적을 함께 돌리면 성능이 빠듯함

## 2. 경로 계획: A* 직접 구현 vs Nav2

- Nav2에 이미 A*/Dijkstra(NavFn), Smac Planner(Hybrid-A*), 로컬 플래너(DWB, MPPI) 포함
- **추천:** 처음엔 Nav2 기본 플래너로 전체 흐름 완성 → 이후 직접 구현한 A*를 **Nav2 Global Planner 플러그인**으로 교체
  - 알고리즘 구현 경험 + Nav2의 장애물 회피·복구 동작을 함께 활용
- 전역 경로(A*)와 지역 경로(장애물 회피)를 분리해서 설계. 동적 장애물은 지역 플래너가 처리

## 3. 웹 ↔ ROS 2 연결 구조 (핵심)

```
[Browser] ⇄ WebSocket ⇄ [Django + Channels] ⇄ (Redis) ⇄ [ROS2 Bridge Node] ⇄ [Nav2 / Robot]
```

- Django WebSocket: **Django Channels + Daphne/Uvicorn + Redis(channel layer)**
- **rclpy를 Django 프로세스 안에서 직접 돌리지 않기** (이벤트 루프·스레드 충돌). 별도 ROS 2 브리지 노드로 분리
- 다른 선택지: `rosbridge_suite` + `roslibjs`로 브라우저가 ROS에 직접 연결
  - 빠르게 만들 수 있지만 업무 로직(작업 큐, 권한, 로그)이 흩어지는 단점
- WebSocket 메시지 규격(JSON 스키마) 사전 정의
  - 예: `robot.pose`, `robot.status`, `task.create`, `task.update`, `task.confirm_load`, `robot.estop`
- 로봇 위치는 5~10Hz로 제한해서 전송
- 맵은 OccupancyGrid 원본 대신 **PNG + 메타데이터(resolution, origin)** 로 전송

## 4. 좌표계 및 맵 관리

- **웹 픽셀 ↔ 맵 좌표 변환 규칙**: `map.yaml`의 `resolution`과 `origin` 사용, 이미지 y축 반전 주의
- **물류 위치는 `(x, y, yaw)`로 저장**: 선반 앞에서 바라볼 방향까지 포함
  - 선반 위치와 로봇 정차 위치(접근 포즈)를 분리 저장 권장
- **맵 버전 관리**: 맵을 다시 만들면 좌표가 틀어지므로 위치 데이터를 맵 ID에 연결
- 통행 금지·감속 구역 필요 여부 → Nav2 Keepout / Speed Filter
- **AMCL 초기 포즈 지정 방식**: 고정 홈 위치에서 출발 / 웹에서 지정

## 5. 작업 흐름 및 상태 머신

```
IDLE → MOVING → ARRIVED(적재 대기) → LOADED → MOVING(다음 위치) … → RETURNING → IDLE
                  ↓ (어느 상태에서든)
                ERROR / PAUSED / CANCELED
```

결정할 사항:
- [ ] 여러 위치 동시 선택 허용 여부, 방문 순서 최적화(가까운 순 / 간단한 TSP)
- [ ] 실패 처리: 경로 막힘, 목표 도달 불가, 위치 추정 실패, 시간 초과일 때 재시도 / 건너뛰기 / 복귀
- [ ] 도착 후 아무도 싣지 않을 때의 적재 대기 타임아웃
- [ ] "적재 완료" 버튼을 누가 누르는지 (관리자 PC / 현장 작업자 스마트폰이면 모바일 UI 필요)
- [ ] 작업 취소, 일시정지, 비상정지(E-stop)
- [ ] 로봇이 바쁠 때 들어온 요청을 처리하는 작업 큐 정책

## 6. 동적 객체 추적

**목적부터 결정**
- 목적이 "피하기"라면 Nav2 costmap(obstacle layer) + MPPI로 대부분 해결
- 목적이 "추적·표시·예측"이라면 별도 파이프라인 필요

**추적 파이프라인 예시**
1. LiDAR 스캔에서 정적 맵과 겹치는 점 제거
2. 남은 점 클러스터링 (Euclidean / DBSCAN)
3. Kalman Filter + 헝가리안 매칭으로 ID·속도 추적
4. 웹 지도에 표시, 필요 시 costmap에 반영

**LDS-02 한계**
- 2D 평면 한 층만 감지 (Burger 기준 약 17cm 높이). 사람은 다리만 보이고, 위로 튀어나온 선반이나 지게차 포크는 감지 불가
- 낮은 해상도와 스캔 속도(약 5Hz). 빠르게 움직이는 물체는 추적이 어려움
- 유리·검은 물체 인식이 약함

## 7. 운영 및 안전

- [ ] **배터리**: 자동 도킹 없음. 웹에 전압 표시, 기준치 이하일 때 자동 복귀 여부
- [ ] **네트워크**: 창고 Wi-Fi 품질, `ROS_DOMAIN_ID`, Wi-Fi 환경의 DDS 탐색 문제 (Fast DDS Discovery Server 고려)
- [ ] **연결 끊김**: 하트비트가 끊기면 정지할지, 작업을 계속할지
- [ ] **적재 방식**: 트레이·바구니 장착, 무게 센서나 물리 버튼 추가 여부
- [ ] 속도 제한

## 8. 확장성 및 기타

- **멀티 로봇 대비**: 지금은 한 대라도 DB와 ROS 네임스페이스에 `robot_id` 미리 반영
- **사용자 권한**: 관리자(맵·위치 편집) / 작업자(호출·적재 확인)
- **DB 모델 초안**: `Map`, `Location`, `Robot`, `Task`, `TaskStep`, `EventLog`
- **시뮬레이션 환경**: Gazebo + `turtlebot3_gazebo` + **AWS RoboMaker Small Warehouse World**(ROS 2 브랜치)로 먼저 개발

---

## 9. 진행 순서 (MVP 단계별)

1. **시뮬레이션**: Gazebo 창고 맵에서 SLAM으로 맵 생성, Nav2로 목표 지점 이동
2. **브리지**: ROS 2 브리지 노드 ↔ Django Channels 연결, 웹에 로봇 위치 실시간 표시
3. **위치 관리**: 웹 지도에서 물류 위치 등록(x, y, yaw), 클릭하면 이동
4. **작업 흐름**: 상태 머신, 적재 확인 버튼, 다음 위치 또는 복귀, 실패 처리
5. **실제 로봇 이식**: Pi + LDS-02로 옮기고 네트워크·성능 튜닝
6. **고도화**: 직접 구현한 A* 플러그인, 동적 객체 추적, 경로 최적화, 멀티 로봇

## 10. 다음에 정할 것

- [ ] 시스템 아키텍처 다이어그램
- [ ] Django 모델 설계
- [ ] WebSocket 메시지 규격

---

## 11. 구현 현황 (2026-10-19)

실행 방법과 동작 확인 결과는 [README.md](README.md) 참고.

| 항목 | 상태 |
|---|---|
| ROS 2 Humble, Waffle Pi, ROS_DOMAIN_ID=44 | 완료 |
| Django + Channels(WebSocket) + MySQL | 완료 |
| 웹 UI: 지도 표시, 로봇 위치/경로 실시간 표시, 위치 등록, 초기 위치 지정, 여기로 이동, 비상정지 | 완료 |
| 작업 흐름: 물류 위치 → 적재 확인 → 다음 위치 → 홈 복귀 | 구현 완료, 실제 로봇 전체 흐름 시험은 아직 |
| 경로 계획: 직접 구현한 A* (`astar.py`) | 완료, 실제 로봇에서 동작 확인 |
| 경로 추종: Nav2 controller_server 대신 직접 구현한 pure pursuit (`follower.py`) | 완료, 실제 로봇에서 동작 확인 |
| 위치 추정: Nav2 AMCL | 사용 중 |
| 안전장치: TF 끊김 정지, 앞쪽 장애물 정지/대기, 경로 이탈/진행 없음 감지, 재계획 | 구현 완료, 장애물 정지는 실제 시험 아직 |
| 동적 객체 추적 | 제외 (6번 참고: 저속 물체는 정지/대기로 대응) |

계획에서 바뀐 점:
- 2번(Nav2 FollowPath로 추종) → 이 환경에서 controller_server 안의 TF 수신이 멈추는 문제로 직접 구현한 추종기로 변경 (README 8번 참고)
- 1번(연산 위치) → 계획대로 Pi는 bringup만, 나머지는 PC
