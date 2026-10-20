"""카메라 없이 LED 인식 결과를 흉내 낸다: cargo/detected 에 라벨을 몇 초 동안 발행.

    source scripts/env.sh
    python3 tools/cargo_sim.py B            # B 가 실린 것처럼 (3초)
    python3 tools/cargo_sim.py ''           # 아무것도 없는 것처럼 (꺼냄)
    python3 tools/cargo_sim.py B --seconds 0   # Ctrl-C 까지 계속

브리지는 1초 동안 같은 값일 때만 서버에 알리고, 발행이 끝나도 마지막 값을 유지한다.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label', help="보이는 라벨 ('' = 없음)")
    ap.add_argument('--seconds', type=float, default=3.0, help='발행 시간 (0 = 계속)')
    args = ap.parse_args()
    rclpy.init()
    node = Node('cargo_sim')
    pub = node.create_publisher(String, 'cargo/detected', 10)
    end = time.time() + args.seconds if args.seconds > 0 else float('inf')
    try:
        while time.time() < end:
            pub.publish(String(data=args.label))
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    print(f'[cargo_sim] 발행 끝: {args.label or "(없음)"}', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
