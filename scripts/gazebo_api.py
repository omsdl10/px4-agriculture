"""Load the installed Gazebo Python bindings into the project venv."""
import os
import sys
from pathlib import Path
# Gazebo wheels are supplied by the installed simulator, not PyPI.
extra=os.environ.get('GZ_PYTHON_PATH')
if extra:sys.path.append(extra)
if sys.platform=='darwin':
    path=Path('/opt/homebrew/lib')/f'python{sys.version_info.major}.{sys.version_info.minor}'/'site-packages'
    if path.is_dir():sys.path.append(str(path))
from gz.transport import Node
from gz.msgs.clock_pb2 import Clock
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.image_pb2 import Image

def stamp(header):return header.stamp.sec+header.stamp.nsec/1e9
