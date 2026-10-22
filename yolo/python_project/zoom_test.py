"""Save the zoom stream; zoom_camera.py must be running first."""
from pathlib import Path
from test import main

if __name__ == '__main__':
    main(default_topic='/camera/zoom',
         default_output=Path.home() / 'camera_dataset/zoom_images')
