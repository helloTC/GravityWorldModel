import numpy as np
import pybullet as p
import pybullet_data
from PhysicalEngine.utils import macro_const

const = macro_const.Const()


def print_separator(n=50):
    print('\n' + n*'-' + '\n')


# Pybullet Model
def load_pybullet(filename, fixed_base=False):
    if filename.endswith('.urdf'):
        flags = get_urdf_flags(**kwargs)
        body = p.loadURDF(filename, useFixedBase=fixed_base, flags=flags,
                            globalScaling=scale, physicsClientId=CLIENT)
    elif filename.endswith('.obj'):
        body = create_obj(filename, scale=scale, **kwargs)
    else:
        raise ValueError(filename)
    return body

# Rendering Setting
def disable_rendering():
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

# Dynamic Setting (Force, Velocity)


# Rotation Transformation

# Camera
def set_camera(distance, yaw, pitch, height):
    """
    Set camera position.
    Note that for simplicity, target position of the camera was fixed and followed the center of objects.
    ------------------
    distance[float]: camera distance.
    yaw[float]: camera yaw angle (in degrees, left/right).
    pitch[float]: camera pitch angle (in degrees, up/down).
    height[float]: camera height. 
    """
    x = distance*np.cos(yaw*const.PI/180)
    y = distance*np.sin(yaw*const.PI/180)
    p.resetDebugVisualizerCamera(distance, yaw+90, pitch, [x,y,height]) 
