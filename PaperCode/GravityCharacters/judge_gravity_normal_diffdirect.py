# Behavior design to examine if our judgment to normality of gravity affected by our visions.
import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import pickle
import time
import os

const = macro_const.Const()
def run_paradigm(subjname, exp_date, runnum, angle):
    # Link Physical Agent
    AgentID = p.connect(p.GUI, options='--width=768 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    # Load Plane
    angle_rad = angle * np.pi/180
    if angle_rad > np.pi/2:
        floorID = stability_func.create_floor(position=(0, np.cos(angle_rad)-np.sin(angle_rad), np.cos(angle_rad)+np.sin(angle_rad)-9*np.cos(angle_rad)), 
                                          orientation=(angle_rad,0,0), 
                                          color=const.BLACK,
                                          friction=1, 
                                          client=AgentID)

        # Load Cylinder
        cylinderID = stability_func.create_cylinder(
                                      position=(0,np.cos(angle_rad)-np.sin(angle_rad),np.cos(angle_rad)+np.sin(angle_rad)-9*np.cos(angle_rad)),
                                      orientation=(angle_rad,0,0),
                                      radius=20,
                                      height=0.1,
                                      texture='/Users/huangtaicheng/Repository/PhysicalEngine/textures/wood_repeat.png',
                                      mass=0,
                                      friction=1)

    else:
        floorID = stability_func.create_floor(position=(0, np.cos(angle_rad)-np.sin(angle_rad), np.cos(angle_rad)+np.sin(angle_rad)), 
                                          orientation=(angle_rad,0,0), 
                                          color=const.BLACK,
                                          friction=1, 
                                          client=AgentID)

        # Load Cylinder
        cylinderID = stability_func.create_cylinder(
                                      position=(0,np.cos(angle_rad)-np.sin(angle_rad),np.cos(angle_rad)+np.sin(angle_rad)),
                                      orientation=(angle_rad,0,0),
                                      radius=20,
                                      height=0.1,
                                      texture='/Users/huangtaicheng/Repository/PhysicalEngine/textures/wood_repeat.png',
                                      mass=0,
                                      friction=1)

    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # Prepare Box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]

    stimuli_param = np.repeat(np.linspace(0, np.pi/4, 16), 6)
    horizontal_angle = np.linspace(0, 2*np.pi, 96)
    np.random.shuffle(stimuli_param)
    np.random.shuffle(horizontal_angle)

    with open('../GenerateStimuli/unstable_configurations_info.pkl', 'rb') as f:
        configs = pickle.load(f)

    output = {}
    rt = []
    key_record = []
    box_position_all = []
    box_list_all = []

    for n, sp in enumerate(stimuli_param):
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        select_config_idx = np.random.choice(np.arange(configs['position'].shape[0]))
        pos_list = 1.0*configs['position'][select_config_idx,...]
        pos_list[:,-2] += 1
        pos_list[:,-1] += 1

        box_position_all.append(pos_list)
        box_list = configs['boxidx'][select_config_idx,:]
        box_list_all.append(box_list)

        boxIDs = []
        for i in range(len(box_list)):
            posbox  = pos_list[i,:]
            if angle_rad > np.pi/2:
                boxIDs.append(stability_func.create_box(
                            position=(posbox[0], posbox[1]*np.cos(angle_rad)-posbox[2]*np.sin(angle_rad), posbox[2]*np.cos(angle_rad)+posbox[1]*np.sin(angle_rad)-9*np.cos(angle_rad)), 
                            size = box_size[box_list[i]],
                            orientation=(angle_rad,0,0),
                            color=color_list[np.random.choice(len(color_list))],
                            mass=0.2,
                            friction=1))                    
            else:
                boxIDs.append(stability_func.create_box(
                            position=(posbox[0], posbox[1]*np.cos(angle_rad)-posbox[2]*np.sin(angle_rad), posbox[2]*np.cos(angle_rad)+posbox[1]*np.sin(angle_rad)),
                            size = box_size[box_list[i]],
                            orientation=(angle_rad,0,0),
                            color=color_list[np.random.choice(len(color_list))],
                            mass=0.2,
                            friction=1))

        # Set Camera angle
        if angle_rad == 0:
            utils.set_camera(distance=4.0, yaw=-45, pitch=-20, height=4.0)
        elif angle_rad <= np.pi/2:
            utils.set_camera(distance=2.0, yaw=-45, pitch=0, height=2.0, x_offset=3, y_offset=-6)
        else:
            utils.set_camera(distance=4.0, yaw=-45, pitch=-20, height=4.0)
        
        horizon_alpha = horizontal_angle[n]
        # print('SP: {}, horizon angle: {}'.format(sp*180/np.pi, horizon_alpha*180/np.pi))
        # Set gravity
        p.setGravity(const.GRAVITY*np.sin(sp)*np.sin(horizon_alpha), 
                    const.GRAVITY*(-1.0*np.cos(sp)*np.sin(angle_rad)+np.sin(sp)*np.cos(angle_rad)*np.cos(horizon_alpha)), 
                    const.GRAVITY*(np.cos(sp)*np.cos(angle_rad)+np.sin(sp)*np.sin(angle_rad)*np.cos(horizon_alpha)))

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        yaw = -45
        pitch=-20
        camera_speed = 0.5
        distance = 5
        height = 3.5
        for i in range(int(270/camera_speed)):
        # while 1:
            utils.set_camera(distance, yaw, pitch, height)
            if yaw>360:
                yaw = 0
            yaw += camera_speed
            time.sleep(const.TIME_STEP)

        time.sleep(1)
        _ = p.getKeyboardEvents()

        # flag = False
        time0 = time.time()
        while 1:
            p.stepSimulation()
            time.sleep(const.TIME_STEP)

            keys = p.getKeyboardEvents()
            if (ord('f') in keys) and (keys[ord('f')] & p.KEY_WAS_TRIGGERED):
                key_record.append(True)
                rt.append(time.time()-time0)
                # flag = True
                break
            elif (ord('j') in keys) and (keys[ord('j')] & p.KEY_WAS_TRIGGERED):
                key_record.append(False)
                rt.append(time.time()-time0)
                # flag = True
                break
            else:
                pass
        # if (flag is False):
        #     key_record.append(None)
        #     rt.append(None)
        #     flag = True
        for i, boxID in enumerate(boxIDs):
            p.removeBody(boxID)

    p.disconnect()

    output['judge'] = np.array(key_record)
    output['rt'] = np.array(rt)
    output['gravity_direction'] = stimuli_param
    output['horizontal_direction'] = horizontal_angle
    output['box_position'] = box_position_all
    output['box_list'] = box_list_all
    output['box_size'] = box_size
    with open(subjname+'_'+exp_date+'_'+str(int(angle))+'_'+runnum+'_diffdirect.pkl', 'wb') as f:
        pickle.dump(output, f)

if __name__ == '__main__':
    run_paradigm('subj1', '20220429', '4', 180)
