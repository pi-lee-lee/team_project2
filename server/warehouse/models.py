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
            'image_url': f'/map/{self.id}.png?v={int(self.updated_at.timestamp())}',
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

    class Meta:
        ordering = ['kind', 'name']

    def __str__(self):
        return f'{self.name} ({self.x:.2f}, {self.y:.2f})'

    def to_dict(self):
        return {'id': self.id, 'name': self.name, 'kind': self.kind,
                'x': self.x, 'y': self.y, 'yaw': self.yaw}


class Robot(models.Model):
    STATE_OFFLINE = 'OFFLINE'
    STATE_IDLE = 'IDLE'
    STATE_MOVING = 'MOVING'
    STATE_WAITING_LOAD = 'WAITING_LOAD'
    STATE_ERROR = 'ERROR'
    STATE_CHOICES = [(s, s) for s in
                     (STATE_OFFLINE, STATE_IDLE, STATE_MOVING, STATE_WAITING_LOAD, STATE_ERROR)]

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
    ACTION_LOAD = 'load'  # 도착 후 적재 확인을 기다림
    ACTION_MOVE = 'move'  # 도착하면 바로 완료 (복귀, 단순 이동)
    ACTION_CHOICES = [(ACTION_LOAD, '적재'), (ACTION_MOVE, '이동')]

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
                'x': self.x, 'y': self.y, 'yaw': self.yaw, 'status': self.status,
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
