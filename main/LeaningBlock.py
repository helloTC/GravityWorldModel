# Code for leaning block paradigm

import pybullet as p
import pybullet_data
import numpy as np
from PhysicalEngine.utils import macro_const
from PhysicalEngine.utils import stability_func
import time

const = macro_const.Const()

if __name__ == '__main__':
    # Link Physical Agent
    AgentID = p.connect(p.GUI)

    # Prepare floor
    planeID = stability_func.create_floor()

    # Do Not Render Images during Building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    colorlist = [const.RED, const.GREEN, const.BLUE, const.BROWN, const.YELLOW]
    bodyID = []
    # Prepare BOX
    for i in range(6):
        bodyID.append(stability_func.create_box([0+i*0.10,0,0.1+i*0.2], [0.6,0.6,0.2], colorlist[np.random.choice(5)]))

    pos_list = []
    for i in range(6):
        pos_list.append([0+i*0.1,0,0.1+i*0.2])

    # Rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


    # Set Camera Parameters
    roll = 0
    radius = 3
    camera_speed = 0.1
    # Start Simulation

    # Record Video
    # mp4logging = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'test.mp4')

    # No Gravity
    p.setGravity(0,0,0)
    # for i in range(2700):
        # Set Camera Angle
    #     if roll>360:
    #         roll = 0
    #     roll += camera_speed
    #     x = radius*np.cos(roll*const.PI/180)
    #     y = radius*np.sin(roll*const.PI/180)
    #     p.resetDebugVisualizerCamera(radius, roll+90, -30, [x,y,1])

    #     p.stepSimulation()
    #     time.sleep(const.TIME_STEP)
    
    # Start to set gravity
    # p.setGravity(0,0,const.GRAVITY)
    for i in range(1000):
        p.stepSimulation()
        # if i<240:
        #     p.applyExternalForce(bodyID[-1], -1, [2,0,0], pos_list[-1], flags=p.WORLD_FRAME)
        time.sleep(const.TIME_STEP)

    # p.stopStateLogging(mp4logging)



