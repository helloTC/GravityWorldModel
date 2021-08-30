import numpy as np 
import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import pickle
# import time
import os

const = macro_const.Const()
def adjust_parameter_threshold(param, stab, targ_stab = 0.5, rate = 0.03):
    """
    """
    param_update = param + (stab - targ_stab)*rate
    param_diff= param - param_update
    return param_update, param_diff

def find_stability_parameters(box_configs, param_init=0.08, targ_stab=0.50, iteration=500, stab_tol=0.02, isshow=False):
    """
    Accomplish function to iteratively find the best position displacement paramter for specific stability of a configuration.
    We iteratively examined stability of a specific parameter with enough iteration times, then optimized the parameter using a gradient descent method.
    -------------------------------
    box_configs[string]: pickle files recorded positions of boxes.
    param_init[float]: initialized displacement parameter.
    targ_stab[float]: target stability of the configuration.
    iteration[int]: simulation iteration time for evaluating stability of each parameter. 
    stab_tol[float]: stability tolerance. The parameter will be convergent when its difference to the target stability smaller than stab_tol.

    Return:
    --------------
    stab_all[float]: Stabilities correspond to each parameter across the whole optimization progress.
    param_all[float]: Parameters iteratively across the whole optimization progress.
    """
    # Link Physical Agent
    if not isshow:
        AgentID = p.connect(p.DIRECT)
    else:
        AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')

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
    with open(box_configs, 'rb') as f:
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

    stab_all = []
    param_all = []
    param = 1.0*param_init
    stab = 1.0

    while True:
        # Adjust iteration, larger error, less iterations
        if round(np.abs(stab-targ_stab),2) >= 0.10:
            rate = 0.10
        elif round(np.abs(stab-targ_stab),2) >= 0.07:
            rate = 0.05
        elif round(np.abs(stab-targ_stab),2) >= 0.04:
            rate = 0.03
        elif round(np.abs(stab-targ_stab),2) > stab_tol:
            rate = 0.01
        else:
            break
        print('Iteration time {}'.format(iteration))
        actual_list = []
        # Rendering
        if isshow:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        for ite in range(iteration):
            # First Prepare initial stimulus
            for i, boxID in enumerate(boxIDs):
                p.resetBasePositionAndOrientation(boxID, box_pos_ori_all[i], box_ori_ori_all[i])
            # Adjust Config
            box_pos_adj, box_ori_adj = stability_func.adjust_confg_position_fixdistance(boxIDs, param)
            for i, boxID in enumerate(boxIDs):
                p.resetBasePositionAndOrientation(boxID, box_pos_adj[i], box_ori_adj[i])
            # Provide gravity
            p.setGravity(0,0,-9.8)
            p.setTimeStep(const.TIME_STEP)
            for i in range(300):
                p.stepSimulation()

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
        # Adjust parameter
        param, _ = adjust_parameter_threshold(param, stab, targ_stab=targ_stab, rate=rate)
        print('Stability: {}; Updated Parameters: {}'.format(stab, param)) 
        stab_all.append(stab)
        param_all.append(param)
    stab_all = np.array(stab_all)
    param_all = np.array(param_all)
    return stab_all, param_all

if __name__ == '__main__':
    box_parpath = '/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BoxStimuli'
    box_configs = os.path.join(box_parpath, '106_steady.pkl')
    stab, param = find_stability_parameters(box_configs, param_init=0.08, targ_stab=0.80, iteration=500, stab_tol=0.01, isshow=False)
