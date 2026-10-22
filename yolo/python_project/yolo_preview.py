#!/usr/bin/env python3
"""Run from VS Code to open a detection window for the CSI camera.

Close the window or press Ctrl+C to stop this preview's detector.
The original /yolo/image_raw workflow is independent of this preview.
"""
from yolo_runtime import run

if __name__ == '__main__':
    run(preview=True)
