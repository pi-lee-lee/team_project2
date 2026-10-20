"""맵을 원점 기준으로 회전했을 때(tools/straighten_map.py), 등록된 위치 좌표도 같은 각도로 돌린다.

    python manage.py rotate_locations 2.40 [--map maze_map] [--dry-run]

각도는 straighten_map.py 가 출력한 rotation_deg (반시계 방향 +).
"""
import math

from django.core.management.base import BaseCommand, CommandError
from django.db import transaction

from warehouse.models import Location, Map


class Command(BaseCommand):
    help = '등록된 위치를 맵 원점 기준으로 회전'

    def add_arguments(self, parser):
        parser.add_argument('degrees', type=float)
        parser.add_argument('--map', help='맵 이름 (기본: 활성 맵)')
        parser.add_argument('--dry-run', action='store_true')

    @transaction.atomic
    def handle(self, degrees, map=None, dry_run=False, **options):
        m = Map.objects.filter(name=map).first() if map else Map.objects.filter(is_active=True).first()
        if m is None:
            raise CommandError('맵을 찾을 수 없습니다.')
        t = math.radians(degrees)
        c, s = math.cos(t), math.sin(t)
        for loc in Location.objects.filter(map=m):
            x, y = c * loc.x - s * loc.y, s * loc.x + c * loc.y
            yaw = math.atan2(math.sin(loc.yaw + t), math.cos(loc.yaw + t))
            self.stdout.write(f'{loc.name}: ({loc.x:.3f}, {loc.y:.3f}, {math.degrees(loc.yaw):.1f}°) → '
                              f'({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')
            if not dry_run:
                loc.x, loc.y, loc.yaw = x, y, yaw
                loc.save(update_fields=['x', 'y', 'yaw'])
        self.stdout.write(self.style.SUCCESS('미리보기만 했습니다 (--dry-run)' if dry_run else f'{degrees:+.2f}° 회전 적용'))
