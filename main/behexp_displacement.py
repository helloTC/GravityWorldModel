import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import pickle
import time
import os

def reward_strategy(orig_reward, actualperf, judgeperf, weight={'TP':1, 'TN': 1, 'FP': -1, 'FN':-1}):
    """
    """
    assert list(weight.keys()) == ['TP', 'TN', 'FP', 'FN'], "Please assign weights to TP, TN, FP and FN."
    # assert sum(weight.values()) == 0, "Summation of weight values needs to be 0."
    if actualperf & judgeperf:
        cond = 'TP'
    elif (not actualperf) & (not judgeperf):
        cond = 'TN'
    elif (not actualperf) & judgeperf:
        cond = 'FP'
    elif actualperf & (not judgeperf):
        cond = 'FN'
    else:
        pass
    reward = orig_reward + weight[cond]
    reward = round(reward, 2)
    return reward

def load_configs(parpath, params, config_name, num_select=50):
    """
    parpath[string]
    params[list]
    config_names[string]
    """
    config_positions = []
    config_orientations = []
    config_params = []
    config_stab = []
    for param in params:
        config_displacement_name = os.path.join(parpath, config_name+'_'+str(param)+'_clean.pkl')
        with open(config_displacement_name, 'rb') as f:
            config = pickle.load(f)
        position_config = config['position']
        orientation_config = config['orientation']
        # stable_config = config['cond']
        config_total_num = len(position_config)
        fall_num = int(round(config_total_num*(1-param),2))
        fall_config_position = np.array(position_config[:fall_num])
        stab_config_position = np.array(position_config[fall_num:])
        fall_config_orientation = np.array(orientation_config[:fall_num])
        stab_config_orientation = np.array(orientation_config[fall_num:])
        num_select_fall = int(round(num_select*(1-param),2))
        num_select_stab = int(round(num_select*param,2))
        fall_config_selidx = np.random.choice(np.arange(len(fall_config_position)), num_select_fall, replace=False)
        stab_config_selidx = np.random.choice(np.arange(len(stab_config_position)), num_select_stab, replace=False)
        fall_config_position_select = fall_config_position[fall_config_selidx, ...]
        stab_config_position_select = stab_config_position[stab_config_selidx, ...]
        fall_config_orientation_select = fall_config_orientation[fall_config_selidx, ...]
        stab_config_orientation_select = stab_config_orientation[stab_config_selidx, ...]
        config_positions.extend(fall_config_position_select)
        config_positions.extend(stab_config_position_select)
        config_orientations.extend(fall_config_orientation_select)
        config_orientations.extend(stab_config_orientation_select)
        config_stab.extend([False]*len(fall_config_position_select))
        config_stab.extend([True]*len(stab_config_position_select))
        config_params.extend([param]*num_select)
        
    config_positions = np.array(config_positions)
    config_orientations = np.array(config_orientations)
    config_stab = np.array(config_stab)
    config_params = np.array(config_params)
    # Randomize
    config_rdm_idx = np.arange(config_positions.shape[0])
    np.random.shuffle(config_rdm_idx)
    return config_positions[config_rdm_idx, ...], config_orientations[config_rdm_idx, ...], config_params[config_rdm_idx], config_stab[config_rdm_idx]

def getKeyPressed(time_max=4.0):
    """
    """
    textid_judge = p.addUserDebugText('Stable or Unstable', [-1.5,-1.5,5], textSize=2, textColorRGB=[1,0,0])
    judge = None
    time0 = time.time()
    while True:
        time1 = time.time()
        time_pass = time1 - time0
        # Stop criterion
        if time_pass >= (time_max)-0.019:
            p.removeUserDebugItem(textid_judge)
            break
        keys = p.getKeyboardEvents()
        if judge is not None:
            keys = [None]
        if (ord('0') in keys) and (keys[ord('0')] & p.KEY_WAS_TRIGGERED):
            judge = True
            p.removeUserDebugItem(textid_judge)
            textid_judge = p.addUserDebugText('Your Judgment is Stable', [-1.5, -1.5, 5], textSize=2, textColorRGB=[1,0,0])
        elif (ord('1') in keys) and (keys[ord('1')] & p.KEY_WAS_TRIGGERED):
            judge = False
            p.removeUserDebugItem(textid_judge)
            textid_judge = p.addUserDebugText('Your Judgment is Fall', [-1.5, -1.5, 5], textSize=2, textColorRGB=[1,0,0])
        else:
            pass
    return judge, textid_judge

def getKeyPressed_trigger():
    """
    """
    while True:
        keys = p.getKeyboardEvents()
        if (ord('s') in keys) and (keys[ord('s')]& p.KEY_WAS_TRIGGERED):
            break
        else:
            pass
    return None

const = macro_const.Const()
if __name__ == '__main__':
    # Link Physical Agent
    AgentID = p.connect(p.GUI, options='--width=1366 --height=1000 --background_color_red=0 --background_color_green=0 --background_color_blue=0')

    # Load Plane
    floorID = stability_func.create_floor(color=const.BLACK, friction=1.0, client=AgentID)

    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    config_name = '116_steady'
    # Prepare position displacement configurations
    parpath_displacement = '/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/StimulusNoise/GenerateStimuli/Position'
    pos_params = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    num_select = 30
    configs_position, configs_orientation, configs_params, config_stab = load_configs(parpath_displacement, pos_params, config_name, num_select=num_select)
    configs_num = configs_position.shape[0]

    judge_list = []
    actual_list = []
    rt_list = [] 

    # Prepare Box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]
    # Set Camera angle
    yaw = 0
    pitch=-30
    camera_speed = 0.93
    distance = 3
    height = 3
    utils.set_camera(distance, yaw, pitch, height)
    # Load Cylinder
    cylinderID = stability_func.create_cylinder(
                                      position=(0,0,0),
                                      radius=10,
                                      height=0.1,
                                      texture='/Users/huangtaicheng/Repository/PhysicalEngine/textures/wood_repeat.png',
                                      mass=0,
                                      friction=1.0)
    # Load boxes
    with open('/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BoxStimuli/'+config_name+'.pkl', 'rb') as f:
        params = pickle.load(f) 
    # Add cylinder height
    pos_list = np.array(params['pos_list'])
    pos_list[:,-1]+=0.1/2
    box_list = np.array(params['box_list'])
    boxIDs = []
    for i in range(len(params['box_list'])):
        boxIDs.append(stability_func.create_box(
                     pos_list[i,:], 
                     box_size[box_list[i]],
                     color=color_list[np.random.choice(len(color_list))],
                     mass=0.2,
                     friction=1.0))

    box_pos_ori_all = []
    box_ori_ori_all = []
    for i, boxID in enumerate(boxIDs):
        pos_ori, ori_ori = p.getBasePositionAndOrientation(boxID)
        box_pos_ori_all.append(pos_ori)
        box_ori_ori_all.append(ori_ori)
    # Rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    textid_judge = p.addUserDebugText('Wait for trigger', [-1.5,-1.5,5], textSize=3, textColorRGB=[1,1,1])
    # Wait until start
    _ = getKeyPressed_trigger()
    time_exp_start = time.time()

    p.removeUserDebugItem(textid_judge)
    
    for cn in range(configs_num):
        time_orig = time.time()

        for i, boxID in enumerate(boxIDs):
            p.resetBasePositionAndOrientation(boxID, configs_position[cn, i, :], configs_orientation[cn, i, :])


        utils.set_camera(distance, yaw, pitch, height)
        time_camera_orig = time.time()
        while True:
            utils.set_camera(distance, yaw, pitch, height)
            yaw += camera_speed
            if yaw > 360:
                yaw = 0
            time.sleep(0.007)
            time_past = time.time() - time_camera_orig
            if time_past > 4-0.019: # show stimuli for 4s.
                break

        _ = p.getKeyboardEvents() # Get KeyboardEvents to clear keyboard buffer
        time_rt_orig = time.time()
        # Check Keyboard
        judge, textid_judge = getKeyPressed(time_max=(8.0-time_rt_orig+time_orig)) # 4s for a subject to press keys
        judge_list.append(judge)
        time_rt_fin = time.time()
        rt = time_rt_orig - time_rt_fin
        rt_list.append(rt)

        time_fin = time.time()
        print('Time for each trial is {}'.format(time_fin-time_exp_start))

    time_exp_end = time.time()
    p.disconnect()

    # actual_list = np.array(actual_list)
    actual_list = config_stab
    judge_list = np.array(judge_list)
    outlist = {}
    outlist['sigma'] = np.array(configs_params)
    outlist['rt'] = np.array(rt_list)
    outlist['actual_list'] = np.array(actual_list)
    outlist['judge_list'] = np.array(judge_list)
    outlist['config_positions'] = configs_position
    with open('/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BehaviorData/SDT_Behav/PositionControl/'+config_name+'_htc_5-5_nofb.pkl', 'wb') as f:
        pickle.dump(outlist, f)


