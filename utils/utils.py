import numpy as np
import pybullet as p
import pybullet_data
from PhysicalEngine.utils import macro_const

const = macro_const.Const()


def print_separator(n=50):
    print('\n' + n*'-' + '\n')

# Rendering Setting
def disable_rendering():
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

# Dynamic Setting (Force, Velocity)


# Rotation Transformation

# Camera
def set_camera(distance, yaw, pitch, height, x_offset=0, y_offset=0):
    """
    Set camera position.
    Note that for simplicity, target position of the camera was fixed and followed the center of objects.
    ------------------
    distance[float]: camera distance.
    yaw[float]: camera yaw angle (in degrees, left/right).
    pitch[float]: camera pitch angle (in degrees, up/down).
    height[float]: camera height.
    x_offset[float]: offset to modify x coordinate. By default is 0.
    y_offset[float]: offset to modify y coordinate. By default is 0. 
    """
    x = distance*np.cos(yaw*const.PI/180) + x_offset
    y = distance*np.sin(yaw*const.PI/180) + y_offset
    p.resetDebugVisualizerCamera(distance, yaw+90, pitch, [x,y,height]) 
