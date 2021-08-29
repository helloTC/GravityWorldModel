import numpy as np 
import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import pickle
import time

def adjust_parameter_threshold(param, stab, targ_stab = 0.5, rate = 0.03):
    """
    """
    param_update = param + (stab - targ_stab)*rate
    return param_update

const = macro_const.Const()
if __name__ == '__main__':
    # Link Physical Agent
    AgentID = p.connect(p.DIRECT)
    # AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')

    # Load Plane
    floorID = stability_func.create_floor(color=const.BLACK, friction=1, client=AgentID)
    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # Load box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    # Load boxes
    # Initialize box positions
    with open('/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BoxStimuli/106_steady.pkl', 'rb') as f:
        params = pickle.load(f) 
    pos_list = np.array(params['pos_list'])
    box_list = np.array(params['box_list'])
    boxIDs = []
    for i in range(len(params['box_list'])):
        boxIDs.append(stability_func.create_box(
                        pos_list[i,:], 
                        box_size[box_list[i]],
                        mass=0.2,
                        friction=1))

    box_pos_ori_all = []
    box_ori_ori_all = []
    for i, boxID in enumerate(boxIDs):
        pos_ori, ori_ori = p.getBasePositionAndOrientation(boxID)
        box_pos_ori_all.append(pos_ori)
        box_ori_ori_all.append(ori_ori)

    param_init = 0.05
    param = 1.0*param_init
    stab = 1.0
    targ_stab = 0.5

    while round(np.abs(stab-targ_stab),2)>=0.01:
        # Adjust iteration, larger error, less iterations
        if round(np.abs(stab-targ_stab),2) >= 0.40:
            iteration = 10
        elif round(np.abs(stab-targ_stab),2) >= 0.10:
            iteration = 30
        elif round(np.abs(stab-targ_stab),2) >= 0.07:
            iteration = 50
        elif round(np.abs(stab-targ_stab),2) >= 0.03:
            iteration = 100
        else:
            iteration = 500
        print('Iteration time {}'.format(iteration))
        actual_list = []
        # Rendering
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        for ite in range(iteration):
            # First Prepare initial stimulus
            for i, boxID in enumerate(boxIDs):
                p.resetBasePositionAndOrientation(boxID, box_pos_ori_all[i], box_ori_ori_all[i])
            # Adjust Config
            # print('Param is {}'.format(param)) 
            box_pos_adj, box_ori_adj = stability_func.adjust_confg_position_fixdistance(boxIDs, param)
            for i, boxID in enumerate(boxIDs):
                p.resetBasePositionAndOrientation(boxID, box_pos_adj[i], box_ori_adj[i])
            # Provide gravity
            p.setGravity(0,0,-9.8)
            for i in range(300):
                p.stepSimulation()
                time.sleep(const.TIME_STEP)

            box_pos_fin_all = []
            for i, boxID in enumerate(boxIDs):
                box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_fin_all.append(box_pos_fin)

            isstable = stability_func.examine_stability(box_pos_adj, box_pos_fin_all, tol=1e-3)
            if True in isstable:
                print('Actual: Fall')
                actual_list.append(False)
            else:
                print('Actual: Stable')
                actual_list.append(True)
        actual_list = np.array(actual_list)
        stab = sum(actual_list)/len(actual_list)
        # Adjust update rate
        if round(np.abs(stab-targ_stab),2) >= 0.40:
            rate = 0.10
        elif round(np.abs(stab-targ_stab),2) >= 0.10:
            rate = 0.05
        elif round(np.abs(stab-targ_stab),2) >= 0.07:
            rate = 0.03
        elif round(np.abs(stab-targ_stab),2) >= 0.03:
            rate = 0.02
        else:
            rate = 0.01
        # Adjust parameter
        param = adjust_parameter_threshold(param, stab, rate=rate)
        print('Stability: {}; Updated Parameters: {}'.format(stab, param))

    
