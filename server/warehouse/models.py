from django.db import models


class Map(models.Model):
    """SLAM으로 만든 맵. 이미지는 PNG(위쪽 행 = y 최대값)로 저장한다."""

    name = models.CharField(max_length=100, unique=True)
    resolution = models.FloatField()  # m/pixel
    origin_x = models.FloatField()    # 이미지 왼쪽 아래 픽셀의 월드 좌표
    origin_y = models.FloatField()
    width = models.IntegerField()
    height = models.IntegerField()
    png = models.BinaryField()
    is_active = models.BooleanField(default=False)
    updated_at = models.DateTimeField(auto_now=True)
    # SLAM 맵(slam_live)을 만든 브리지 실행 번호. SLAM 좌표계(원점 = 시작 때 로봇 위치)는 실행마다 다르므로
    # 새 실행이면 이전 맵과 위치를 지운다
    session = models.CharField(max_length=40, blank=True, default='')
    # 웹 화면에서 맵을 돌려 보여 줄 각도 (rad, 반시계). SLAM 을 새로 시작하면 그때 로봇이 바라보는 방향이
    # 화면 위쪽이 되도록 정한다 (좌표는 그대로, 보기만 돌림). SLAM 맵을 저장하면 같이 복사
    view_yaw = models.FloatField(default=0.0)

    def __str__(self):
        return self.name

    def to_dict(self):
        return {
            'id': self.id,
            'name': self.name,
            'resolution': self.resolution,
            'origin': [self.origin_x, self.origin_y],
            'width': self.width,
            'height': self.height,
            'view_yaw': self.view_yaw,
            # 맵이 2초마다 바뀌므로 ms 단위로 (초 단위면 같은 초 안의 갱신을 브라우저 캐시가 막을 수 있음)
            'image_url': f'/map/{self.id}.png?v={int(self.updated_at.timestamp() * 1000)}',
        }


class Location(models.Model):
    KIND_PICKUP = 'pickup'
    KIND_HOME = 'home'
    KIND_CHOICES = [(KIND_PICKUP, '물류 위치'), (KIND_HOME, '홈(복귀 위치)')]

    map = models.ForeignKey(Map, on_delete=models.CASCADE, related_name='locations')
    name = models.CharField(max_length=100)
    kind = models.CharField(max_length=10, choices=KIND_CHOICES, default=KIND_PICKUP)
    x = models.FloatField()
    y = models.FloatField()
    yaw = models.FloatField(default=0.0)  # rad, 도착 시 바라볼 방향
    # 이 위치에 놓인 물건(LED)을 카메라가 인식했을 때의 라벨 (예: red, B). 비우면 위치 이름으로 비교
    cargo = models.CharField(max_length=50, blank=True, default='')

    class Meta:
        ordering = ['kind', 'name']

    def __str__(self):
        return f'{self.name} ({self.x:.2f}, {self.y:.2f})'

    def to_dict(self):
        return {'id': self.id, 'name': self.name, 'kind': self.kind,
                'x': self.x, 'y': self.y, 'yaw': self.yaw, 'cargo': self.cargo_label()}

    def cargo_label(self):
        return self.cargo or self.name


class Zone(models.Model):
    """웹에서 그린 금지 구역. A* 가 장애물로 보고 피한다 (위치 추정에는 영향 없음).
    rect: (x1, y1)~(x2, y2) 사각형 (맵 축 기준), line: (x1, y1)-(x2, y2) 선분 (두께 0.1m 가상 벽)."""
    SHAPE_RECT = 'rect'
    SHAPE_LINE = 'line'
    SHAPE_CHOICES = [(SHAPE_RECT, '사각형'), (SHAPE_LINE, '선(가상 벽)')]

    map = models.ForeignKey(Map, on_delete=models.CASCADE, related_name='zones')
    name = models.CharField(max_length=100, blank=True, default='')
    shape = models.CharField(max_length=10, choices=SHAPE_CHOICES, default=SHAPE_RECT)
    x1 = models.FloatField()
    y1 = models.FloatField()
    x2 = models.FloatField()
    y2 = models.FloatField()

    class Meta:
        ordering = ['id']

    def __str__(self):
        return f'{self.name or self.shape} ({self.x1:.2f}, {self.y1:.2f})-({self.x2:.2f}, {self.y2:.2f})'

    def to_dict(self):
        return {'id': self.id, 'name': self.name, 'shape': self.shape,
                'x1': self.x1, 'y1': self.y1, 'x2': self.x2, 'y2': self.y2}


class Robot(models.Model):
    STATE_OFFLINE = 'OFFLINE'
    STATE_IDLE = 'IDLE'
    STATE_MOVING = 'MOVING'
    STATE_WAITING_LOAD = 'WAITING_LOAD'
    STATE_WAITING_UNLOAD = 'WAITING_UNLOAD'   # 배달 도착, 받는 사람이 꺼내기를 기다림
    STATE_ERROR = 'ERROR'
    STATE_CHOICES = [(s, s) for s in
                     (STATE_OFFLINE, STATE_IDLE, STATE_MOVING, STATE_WAITING_LOAD, STATE_WAITING_UNLOAD, STATE_ERROR)]

    name = models.CharField(max_length=50, unique=True)
    state = models.CharField(max_length=20, choices=STATE_CHOICES, default=STATE_OFFLINE)
    message = models.CharField(max_length=255, blank=True)
    battery_voltage = models.FloatField(null=True, blank=True)
    battery_percentage = models.FloatField(null=True, blank=True)
    last_seen = models.DateTimeField(null=True, blank=True)

    def __str__(self):
        return self.name

    def to_dict(self):
        return {
            'name': self.name,
            'state': self.state,
            'message': self.message,
            'battery_voltage': self.battery_voltage,
            'battery_percentage': self.battery_percentage,
        }


class Task(models.Model):
    STATUS_RUNNING = 'running'
    STATUS_DONE = 'done'
    STATUS_CANCELED = 'canceled'
    STATUS_FAILED = 'failed'
    STATUS_CHOICES = [(s, s) for s in (STATUS_RUNNING, STATUS_DONE, STATUS_CANCELED, STATUS_FAILED)]

    robot = models.ForeignKey(Robot, on_delete=models.CASCADE, related_name='tasks')
    status = models.CharField(max_length=10, choices=STATUS_CHOICES, default=STATUS_RUNNING)
    created_at = models.DateTimeField(auto_now_add=True)
    finished_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ['-created_at']

    def current_step(self):
        return self.steps.exclude(status__in=TaskStep.FINISHED).order_by('order').first()

    def to_dict(self):
        return {
            'id': self.id,
            'status': self.status,
            'created_at': self.created_at.isoformat(),
            'steps': [s.to_dict() for s in self.steps.order_by('order')],
        }


class TaskStep(models.Model):
    ACTION_LOAD = 'load'      # 도착 후 적재 확인을 기다림
    ACTION_UNLOAD = 'unload'  # 도착 후 수령(물건 꺼냄) 확인을 기다림 (LED 배달)
    ACTION_MOVE = 'move'      # 도착하면 바로 완료 (복귀, 단순 이동)
    ACTION_CHOICES = [(ACTION_LOAD, '적재'), (ACTION_UNLOAD, '수령'), (ACTION_MOVE, '이동')]

    STATUS_PENDING = 'pending'
    STATUS_MOVING = 'moving'
    STATUS_ARRIVED = 'arrived'
    STATUS_DONE = 'done'
    STATUS_FAILED = 'failed'
    STATUS_SKIPPED = 'skipped'
    STATUS_CHOICES = [(s, s) for s in (STATUS_PENDING, STATUS_MOVING, STATUS_ARRIVED,
                                       STATUS_DONE, STATUS_FAILED, STATUS_SKIPPED)]
    FINISHED = (STATUS_DONE, STATUS_SKIPPED)

    task = models.ForeignKey(Task, on_delete=models.CASCADE, related_name='steps')
    order = models.IntegerField()
    action = models.CharField(max_length=10, choices=ACTION_CHOICES)
    # 위치가 나중에 삭제돼도 작업 기록은 남도록 좌표를 복사해 둔다
    location = models.ForeignKey(Location, on_delete=models.SET_NULL, null=True, blank=True)
    name = models.CharField(max_length=100)
    x = models.FloatField()
    y = models.FloatField()
    yaw = models.FloatField()
    status = models.CharField(max_length=10, choices=STATUS_CHOICES, default=STATUS_PENDING)
    # LED 배달: 이 단계에서 카메라로 확인할 물건 라벨 (적재: 이 라벨이 보이면 완료, 수령: 아무것도 안 보이면 완료)
    cargo = models.CharField(max_length=50, blank=True, default='')
    # 이동 기록 (이동 시간 학습용, planning.py). 출발할 때 채우고 도착하면 finished_at
    start_x = models.FloatField(null=True, blank=True)
    start_y = models.FloatField(null=True, blank=True)
    start_yaw = models.FloatField(null=True, blank=True)
    plan_length = models.FloatField(null=True, blank=True)   # 출발 시 A* 경로 길이 (m)
    plan_turn = models.FloatField(null=True, blank=True)     # 출발 시 경로의 총 회전량 (rad)
    predicted_sec = models.FloatField(null=True, blank=True)  # 출발 시 예측한 이동 시간
    started_at = models.DateTimeField(null=True, blank=True)
    finished_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ['order']

    @property
    def goal_id(self):
        return f'step-{self.id}'

    def to_dict(self):
        return {'id': self.id, 'order': self.order, 'action': self.action, 'name': self.name,
                'x': self.x, 'y': self.y, 'yaw': self.yaw, 'status': self.status, 'cargo': self.cargo,
                'predicted_sec': self.predicted_sec,
                'started_at': self.started_at.isoformat() if self.started_at else None}


class EventLog(models.Model):
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE, null=True, blank=True)
    message = models.CharField(max_length=255)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-created_at']

    def to_dict(self):
        return {'message': self.message, 'created_at': self.created_at.isoformat()}
