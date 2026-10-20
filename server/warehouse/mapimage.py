"""OccupancyGrid / PGM -> PNG 변환 (Pillow 없이 표준 라이브러리만 사용)."""
import struct
import zlib


def occupancy_to_gray(value):
    """OccupancyGrid 값(-1, 0~100)을 map_saver와 같은 회색조 값으로 변환."""
    if value < 0 or value > 100:
        return 205  # unknown
    return 254 - int(value * 254 / 100)


def grid_to_rows(data, width, height):
    """OccupancyGrid(row 0 = 아래쪽)를 이미지 행 순서(row 0 = 위쪽)로 변환."""
    lut = bytes(occupancy_to_gray(v if v < 128 else v - 256) for v in range(256))
    rows = []
    for r in range(height - 1, -1, -1):
        rows.append(bytes(data[r * width:(r + 1) * width]).translate(lut))
    return rows


def encode_png(rows, width, height):
    """8bit grayscale PNG 인코딩."""
    raw = b''.join(b'\x00' + row for row in rows)

    def chunk(tag, body):
        return (struct.pack('>I', len(body)) + tag + body
                + struct.pack('>I', zlib.crc32(tag + body) & 0xFFFFFFFF))

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr)
            + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def read_pgm(path):
    """map_saver가 저장한 P5(binary) / P2(ascii) PGM 읽기. (width, height, rows) 반환."""
    with open(path, 'rb') as f:
        content = f.read()

    tokens = []
    pos = 0
    while len(tokens) < 4:
        # 공백과 주석 건너뛰기
        while content[pos:pos + 1].isspace():
            pos += 1
        if content[pos:pos + 1] == b'#':
            while content[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not content[pos:pos + 1].isspace():
            pos += 1
        tokens.append(content[start:pos])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])

    if magic == b'P5':
        pixels = content[pos + 1:pos + 1 + width * height]
    elif magic == b'P2':
        pixels = bytes(int(v) for v in content[pos:].split()[:width * height])
    else:
        raise ValueError(f'지원하지 않는 PGM 형식: {magic!r}')
    if maxval != 255:
        pixels = bytes(min(255, p * 255 // maxval) for p in pixels)
    rows = [pixels[r * width:(r + 1) * width] for r in range(height)]
    return width, height, rows


def decode_png(png):
    """encode_png 로 만든 8bit 회색조 PNG 를 (width, height, rows) 로 되돌린다 (행 0 = 위쪽).
    필터 0(None) 만 쓰는 이 프로젝트의 PNG 전용."""
    pos = 8
    width = height = None
    idat = b''
    while pos < len(png):
        length = struct.unpack('>I', png[pos:pos + 4])[0]
        tag = png[pos + 4:pos + 8]
        body = png[pos + 8:pos + 8 + length]
        if tag == b'IHDR':
            width, height, depth, color = struct.unpack('>IIBB', body[:10])
            if depth != 8 or color != 0:
                raise ValueError('8bit 회색조 PNG 만 지원합니다.')
        elif tag == b'IDAT':
            idat += body
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = width + 1
    rows = []
    for r in range(height):
        if raw[r * stride] != 0:
            raise ValueError('필터가 있는 PNG 는 지원하지 않습니다.')
        rows.append(raw[r * stride + 1:(r + 1) * stride])
    return width, height, rows


def gray_to_occupancy(value):
    """map_saver 회색조 → OccupancyGrid 값 (-1 미탐색, 0~100)."""
    if value == 205:
        return -1
    return round((254 - min(value, 254)) * 100 / 254)
