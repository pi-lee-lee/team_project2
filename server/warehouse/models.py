from django.db import models


class Map(models.Model):
    """A map built with SLAM. The image is stored as PNG (top row = max y)."""

    name = models.CharField(max_length=100, unique=True)
    resolution = models.FloatField()  # m/pixel
    origin_x = models.FloatField()    # World coordinates of the image's bottom-left pixel
    origin_y = models.FloatField()
    width = models.IntegerField()
    height = models.IntegerField()
    png = models.BinaryField()
    is_active = models.BooleanField(default=False)
    updated_at = models.DateTimeField(auto_now=True)
    # Bridge run ID that built the SLAM map (slam_live). The SLAM frame (origin = robot pose at start) differs per run,
    # so a new run clears the previous map and locations
    session = models.CharField(max_length=40, blank=True, default='')

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
            # In ms because the SLAM map changes every 0.5-2 s (with seconds, the browser cache could hide updates within the same second)
            'image_url': f'/map/{self.id}.png?v={int(self.updated_at.timestamp() * 1000)}',
        }


class Location(models.Model):
    KIND_PICKUP = 'pickup'
    KIND_HOME = 'home'
    KIND_DROPOFF = 'dropoff'   # Where the orderer receives the items
    KIND_CHOICES = [(KIND_PICKUP, '물류 위치'), (KIND_HOME, '홈(복귀 위치)'), (KIND_DROPOFF, '사람 위치(수령)')]

    map = models.ForeignKey(Map, on_delete=models.CASCADE, related_name='locations')
    name = models.CharField(max_length=100)
    kind = models.CharField(max_length=10, choices=KIND_CHOICES, default=KIND_PICKUP)
    x = models.FloatField()
    y = models.FloatField()
    yaw = models.FloatField(default=0.0)  # rad, heading to face on arrival
    # Camera label of the item (LED) placed at this location (e.g. red, B). Empty = compare with the location name.
    # Order sections can list several, comma-separated (e.g. led_red, led_green)
    cargo = models.CharField(max_length=200, blank=True, default='')

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
    """Keep-out zone drawn on the web. A* treats it as an obstacle (does not affect localization).
    rect: (x1, y1)-(x2, y2) rectangle (map-axis aligned), line: (x1, y1)-(x2, y2) segment (0.1 m thick virtual wall)."""
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
    STATE_WAITING_UNLOAD = 'WAITING_UNLOAD'   # Delivery arrived, waiting for the recipient to take the item out
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
    ACTION_LOAD = 'load'      # After arrival, wait for load confirmation
    ACTION_UNLOAD = 'unload'  # After arrival, wait for unload (item taken out) confirmation (LED delivery)
    ACTION_MOVE = 'move'      # Done on arrival (return home, plain move)
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
    # Copy the coordinates so the task record survives if the location is deleted later
    location = models.ForeignKey(Location, on_delete=models.SET_NULL, null=True, blank=True)
    name = models.CharField(max_length=100)
    x = models.FloatField()
    y = models.FloatField()
    yaw = models.FloatField()
    status = models.CharField(max_length=10, choices=STATUS_CHOICES, default=STATUS_PENDING)
    # LED delivery: item label to check with the camera at this step (load: done when the label is seen, unload: done when nothing is seen)
    cargo = models.CharField(max_length=50, blank=True, default='')
    # Order: required counts for this step {label: count} (load: what to load at this section, unload: the whole order). Empty = not an order
    items = models.JSONField(blank=True, default=dict)
    # Travel record (for learning travel time, planning.py). Filled on departure; finished_at on arrival
    start_x = models.FloatField(null=True, blank=True)
    start_y = models.FloatField(null=True, blank=True)
    start_yaw = models.FloatField(null=True, blank=True)
    plan_length = models.FloatField(null=True, blank=True)   # A* path length at departure (m)
    plan_turn = models.FloatField(null=True, blank=True)     # Total turning of the path at departure (rad)
    predicted_sec = models.FloatField(null=True, blank=True)  # Travel time predicted at departure
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
                'items': self.items or {},
                'predicted_sec': self.predicted_sec,
                'started_at': self.started_at.isoformat() if self.started_at else None}


class EventLog(models.Model):
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE, null=True, blank=True)
    message = models.CharField(max_length=255)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-created_at']

    def to_dict(self):
        return {'message': self.message, 'created_at': self.created_at.isoformat(),
                'robot_name': self.robot.name if self.robot_id else None}
