"""map_saver로 저장한 map.yaml을 DB에 불러온다.

    python manage.py import_map ~/maps/warehouse.yaml [--name warehouse]
"""
import os

from django.core.management.base import BaseCommand, CommandError

from warehouse.mapimage import encode_png, read_pgm
from warehouse.services import save_map


def parse_yaml(path):
    """map.yaml은 단순한 key: value 형식이라 PyYAML 없이 읽는다."""
    conf = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                key, value = line.split(':', 1)
                conf[key.strip()] = value.strip()
    return conf


class Command(BaseCommand):
    help = 'map.yaml + pgm 파일을 활성 맵으로 불러온다'

    def add_arguments(self, parser):
        parser.add_argument('yaml_path')
        parser.add_argument('--name', help='맵 이름 (기본: yaml 파일 이름). 브리지의 map_name과 같아야 함')

    def handle(self, yaml_path, name=None, **options):
        yaml_path = os.path.expanduser(yaml_path)
        if not os.path.exists(yaml_path):
            raise CommandError(f'파일이 없습니다: {yaml_path}')
        conf = parse_yaml(yaml_path)
        image = os.path.join(os.path.dirname(yaml_path), conf['image'].strip('\'"'))
        origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
        width, height, rows = read_pgm(image)
        if conf.get('negate', '0') == '1':
            rows = [bytes(255 - p for p in r) for r in rows]
        name = name or os.path.splitext(os.path.basename(yaml_path))[0]
        m = save_map(name, float(conf['resolution']), origin[0], origin[1],
                     width, height, encode_png(rows, width, height))
        self.stdout.write(self.style.SUCCESS(
            f'맵 "{m.name}" 불러옴 ({width}x{height}, {m.resolution} m/px), 활성 맵으로 지정'))
