"""Fake the LED detection result without a camera: publish a label on cargo/detected for a few seconds.

    source scripts/env.sh
    python3 tools/cargo_sim.py B            # as if B is loaded (3 s)
    python3 tools/cargo_sim.py ''           # as if nothing is there (unloaded)
    python3 tools/cargo_sim.py B --seconds 0   # keep going until Ctrl-C

The bridge reports to the server only after the value has been stable for 1 s, and keeps the last value after publishing stops.
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
