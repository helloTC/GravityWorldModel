import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import pickle
import time
import os

const = macro_const.Const()
if __name__ == '__main__':
    # Link Physical Agent
    AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
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
                                      radius=5,
                                      height=0.1,
                                      texture='/Users/huangtaicheng/Repository/PhysicalEngine/textures/wood_repeat.png',
                                      mass=0,
                                      friction=1)
    # Prepare Box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]

    if os.path.isfile('stable_configurations_info_pretest.pkl'):
        # Load configurations
        with open('stable_configurations_info_pretest.pkl', 'rb') as f:
            config_info = pickle.load(f)
    else:
        # Initialize configurations
        config_info = {}
        config_info['position'] = []
        config_info['boxidx'] = []
        config_info['pos_xmax'] = []
        config_info['pos_ymax'] = []
        config_info['overlap_xthr'] = []
        config_info['overlap_ythr'] = []
        # config_info['isstable'] = []
        config_info['boxsize'] = box_size

    iter_time = 0
    while 1:
        x_max = np.random.uniform(0.3, 0.9)
        y_max = np.random.uniform(0.3, 0.9)
        overlap_thr_x = np.random.uniform(0.2, 0.8)
        overlap_thr_y = np.random.uniform(0.2, 0.8)
        # Generate configurations
        boxpos, boxidx = stability_func.place_boxes_on_space(10, box_size, pos_range_x=(-x_max,x_max), pos_range_y=(-y_max,y_max), overlap_thr_x=0.6, overlap_thr_y=0.6)
        boxpos = np.array(boxpos)
        boxidx = np.array(boxidx)

        boxpos[:,-1]+=0.1/2

        boxIDs = []
        for i in range(10):
            boxIDs.append(stability_func.create_box(
                          boxpos[i],
                          box_size[boxidx[i]],
                          color=color_list[np.random.choice(len(color_list))],
                          mass=0.2,
                          friction=0.5)
                          )

        box_pos_ori_all = []
        for i, boxID in enumerate(boxIDs):
            box_pos_ori, box_ori_ori = p.getBasePositionAndOrientation(boxID)
            box_pos_ori_all.append(box_pos_ori)

        # Set Camera angle
        utils.set_camera(distance=3, yaw=0, pitch=-30, height=3)

        # Rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # yaw = 90
        # pitch=-30
        # camera_speed = 0.9
        # distance = 2.0
        # height = 3
        # for i in range(int(270/camera_speed)):
        # # while 1:
        #     utils.set_camera(distance, yaw, pitch, height)
        #     if yaw>360:
        #         yaw = 0
        #     yaw += camera_speed
        #     p.stepSimulation()
        #     time.sleep(const.TIME_STEP)

        p.setGravity(0,0,-9.8)

        for i in range(500):
            p.stepSimulation()
            time.sleep(const.TIME_STEP)

        box_pos_fin_all = []
        for i, boxID in enumerate(boxIDs):
            box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
            box_pos_fin_all.append(box_pos_fin)
        isstable_all = stability_func.examine_stability(box_pos_ori_all, box_pos_fin_all)
        if np.all(isstable_all):
            isstable = True
        else:
            isstable = False

        p.setGravity(0,0,0)

        # Human judgment
        if isstable:
            config_info['position'].append(boxpos)
            config_info['boxidx'].append(boxidx)
            config_info['pos_xmax'].append(x_max)
            config_info['pos_ymax'].append(y_max)
            config_info['overlap_xthr'].append(overlap_thr_x)
            config_info['overlap_ythr'].append(overlap_thr_y)
            iter_time += 1
            print('Iteration time: {}'.format(iter_time))

        for i, boxID in enumerate(boxIDs):
            p.removeBody(boxID)
        
        if iter_time % 5 == 0:
            # Temporary store
            with open('stable_configurations_info_pretest.pkl', 'wb') as f:
                pickle.dump(config_info, f)
    p.disconnect()





