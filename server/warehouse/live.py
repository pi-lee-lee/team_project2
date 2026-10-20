"""자주 바뀌는 로봇 상태 (위치, 경로, 상태 보고) 를 DB 대신 메모리에 둔다. 단일 서버 프로세스 전제.

consumers 와 services/planning 이 같이 쓰기 때문에 별도 모듈로 둔다.
"""
LIVE = {}  # robot_name -> {'pose': {...}, 'path': [...], 'health': {...}, 'mode': ...}
