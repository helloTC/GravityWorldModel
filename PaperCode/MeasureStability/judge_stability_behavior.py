# Code to perform stability inference.
# Each run consists of 60 configurations, which were shown in different camera angle and colors.
# 0 denotes stable configuration, 7 denotes unstable configuration

import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import os
import pickle
import time

const = macro_const.Const()
def run_behavior(subj, exp_date, run_num):
    if os.path.isfile('judgment_stability_tmp.npy'):
        os.remove('judgment_stability_tmp.npy')
    # Link Physical Agent
    AgentID = p.connect(p.GUI, options='--width=750 --height=960 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    # Load Plane
    floorID = stability_func.create_floor(color=const.BLACK, friction=1, client=AgentID)

    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # Load Cylinder
    cylinderID = stability_func.create_cylinder(
                                      position=(0,0,0),
                                      orientation=(0,0,0),
                                      radius=10,
                                      height=0.10,
                                      texture='/Users/huangtaicheng/Repository/PhysicalEngine/textures/wood_repeat.png',
                                      mass=0,
                                      friction=1)

    # Prepare Box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]
    # Load all configurations
    config_parpath = '/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BoxStimuli'
    config_names = np.array(os.listdir(config_parpath))
    # Randomization
    np.random.shuffle(config_names)

    # Judgment
    pos_list_all = []
    box_list_all = []
    for cn in config_names:
        # Load configurations
        with open(os.path.join(config_parpath, cn), 'rb') as f:
            params = pickle.load(f)

        # Load box positions
        pos_list = np.array(params['pos_list'])
        pos_list_all.append(pos_list)
        pos_list[...,-1] += 0.10/2
        box_list = np.array(params['box_list'])
        box_list_all.append(box_list)
        # Generate configurations in the platform
        boxIDs = []
        for i in range(pos_list.shape[0]):
            boxIDs.append(stability_func.create_box(
                     pos_list[i,:], 
                     box_size[box_list[i]],
                     color=color_list[np.random.choice(len(color_list))],
                     mass=0.2,
                     friction=1)
                    )
        # Set Camera
        # set camera angle in random direction
        yaw = np.random.uniform(0,360)
        pitch=-20
        camera_speed = 0.5
        distance = 5.0
        height = 3
        utils.set_camera(distance, yaw, pitch, height)
        # Rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        for i in range(int(360/camera_speed)):
            utils.set_camera(distance, yaw, pitch, height)
            if yaw>360:
                yaw=0
            yaw += camera_speed
            time.sleep(const.TIME_STEP)

        # textid = p.addUserDebugText('1 -> 7: stable -> unstable', [-,0,4.5], textSize=1)
        # Make judgment
        os.system(f'python gui.py')        
        # p.removeUserDebugItem(textid)

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        for boxID in boxIDs:
            p.removeBody(boxID)
    p.disconnect()

    pos_list_all = np.array(pos_list_all)
    box_list_all = np.array(box_list_all)
    # Save all record info
    # Behavior judgment was temporarily saved in 'judgment_stability_tmp.npy'
    behjudge = np.load('judgment_stability_tmp.npy')
    outdata = {}
    outdata['behjudge'] = behjudge
    # Configurations
    outdata['config_names'] = config_names
    # Configurations Position
    outdata['pos_list'] = pos_list_all
    # Configuration Box
    outdata['box_list'] = box_list_all
    # Box size
    outdata['box_size'] = box_size
    with open(subj+'_'+exp_date+'_'+run_num+'_stabjudge.pkl', 'wb') as f:
        pickle.dump(outdata, f)
    
    # Delete Temporarily behavior jugment file
    os.remove('judgment_stability_tmp.npy')

if __name__ == '__main__':
    run_behavior('subj1', '20220430', '6')
    





