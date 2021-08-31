import numpy as np 
import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import pickle
import os
from multiprocessing import Pool

const = macro_const.Const()
def adjust_parameter_threshold(param, stab, targ_stab = 0.5, rate = 0.03):
    """
    """
    param_update = param + (stab - targ_stab)*rate
    param_diff= param - param_update
    return param_update, param_diff

def find_stability_parameters(box_configs, param_init=0.08, targ_stab=0.50, iteration=500, stab_tol=0.02, isshow=False):
    """
    Accomplish function to iteratively find the best gravity parameter for specific stability of a configuration.
    We iteratively examined stability of a specific parameter with enough iteration times, then optimized the parameter using a gradient descent method.
    -------------------------------
    box_configs[string]: pickle files recorded positions of boxes.
    param_init[float]: initialized gravity parameter.
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
            rate = 0.08
        elif round(np.abs(stab-targ_stab),2) >= 0.07:
            rate = 0.05
        elif round(np.abs(stab-targ_stab),2) >= 0.05:
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
            # Provide gravity
            # Randomize angle in horizontal position
            hor_angle = np.random.uniform(0, 2*const.PI)
            norm_array = np.array([np.sin(hor_angle)*np.sin(param), np.cos(hor_angle)*np.sin(param), np.cos(param)]) 
            p.setGravity(*(const.GRAVITY*norm_array))
            p.setTimeStep(const.TIME_STEP)
            for i in range(500):
                p.stepSimulation()
            p.setGravity(0,0,0)
            box_pos_fin_all = []
            for i, boxID in enumerate(boxIDs):
                box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_fin_all.append(box_pos_fin)

            isstable = stability_func.examine_stability(box_pos_ori_all, box_pos_fin_all, tol=1e-3)
            if True in isstable:
                # print('    Actual: Fall')
                actual_list.append(False)
            else:
                # print('    Actual: Stable')
                actual_list.append(True)
        actual_list = np.array(actual_list)
        stab = sum(actual_list)/len(actual_list)
        # Adjust parameter
        param, _ = adjust_parameter_threshold(param, stab, targ_stab=targ_stab, rate=rate)
        print('Target Stability{}; Stability: {}; Updated Parameters: {}'.format(targ_stab, stab, param)) 
        # An unstable initialized configuration could cause parameters smaller than 0.
        # -1 means invalid value
        if param < 0:
            stab_all.append(-1)
            param_all.append(-1)
            break
        else:
            stab_all.append(stab)
            param_all.append(param)
    stab_all = np.array(stab_all)
    param_all = np.array(param_all)
    return stab_all, param_all

if __name__ == '__main__':
    # Prepare box names
    box_parpath = '/home/hellotc/PhysicsEngine/PNAS2013_exp1_stimuli_use'
    # box_parpath = '/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BoxStimuli'
    box_names = os.listdir(box_parpath)
    # box_names = [bn for bn in box_names if 'steady' in bn]
    box_names = [box_names[0]]

    # find_stability_parameters(os.path.join(box_parpath, box_names[0]), iteration=500, stab_tol=0.01, isshow=True)
    # Prepare parameters
    # stability_target = np.arange(0.1,1.0,0.1)
    stability_target = [0.5]
    
    out_box_names = []
    out_stab = []
    out_param = []
    # Multiprocess
    pool = Pool(processes=2)
    multiproc_handle = []
    for bn in box_names:
        for stab_target in stability_target:
            print('Target Stability {}'.format(stab_target))
            out_box_names.append(bn)
            multiproc = pool.apply_async(find_stability_parameters, (os.path.join(box_parpath, bn), 0.08, stab_target, 500, 0.005, False))
            multiproc_handle.append(multiproc)
    pool.close()
    pool.join()

    for multiproc in multiproc_handle:
        multiproc_data = multiproc.get()
        out_stab.append(multiproc_data[0][-1])
        out_param.append(multiproc_data[1][-1])
    out_box_names = np.array(out_box_names)
    out_stab = np.array(out_stab)
    out_param = np.array(out_param)
    out_dict = {}
    out_dict['box_names'] = out_box_names
    out_dict['stab'] = out_stab
    out_dict['param'] = out_param
    with open('gravity_parameters.pkl', 'wb') as f:
        pickle.dump(out_dict, f)



