import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import pickle
import time

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
                                      radius=10,
                                      height=0.1,
                                      texture='/Users/huangtaicheng/Repository/PhysicalEngine/textures/wood_repeat.png',
                                      mass=0.2,
                                      friction=1)
    # Prepare Box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]
     # Load box positions
    # with open('/Users/huangtaicheng/workingdir/Code/pybullet/formalwork/BoxStimuli/56_fall.pkl', 'rb') as f:
    #     params = pickle.load(f)
    with open('unstable_configurations_info.pkl', 'rb') as f:
        params = pickle.load(f)

    # idx = np.random.choice(np.arange(2000))
    idx = 100
    pos_list = np.array(params['position'])[idx,...] 
    box_list = np.array(params['boxidx'])[idx,...]
    # pos_traj = params['all_traj_position'][-1,...]
    # ori_traj = params['all_traj_orientation'][-1,...]
    boxIDs = []
    for i in range(pos_list.shape[0]):
        boxIDs.append(stability_func.create_box(
                     pos_list[i,:], 
                     box_size[box_list[i]],
                     color=color_list[np.random.choice(len(color_list))],
                     # color=[const.BLUE],
                     mass=0.2,
                     friction=1)
                    )

    # box_pos_ori_all = []
    # box_ori_ori_all = []
    # for i, boxID in enumerate(boxIDs):
    #     box_pos_ori, box_ori_ori = p.getBasePositionAndOrientation(boxID)
    #     box_pos_ori_all.append(box_pos_ori)
    #     box_ori_ori_all.append(box_ori_ori)

    # for i, boxID in enumerate(boxIDs):
    #     p.resetBasePositionAndOrientation(boxID, box_pos_ori_all[i], box_ori_ori_all[i])
    # Set Camera angle
    # utils.set_camera(distance=3, yaw=0, pitch=-30, height=3)


    # Check Keyboard
    # while 1:
    #     keys = p.getKeyboardEvents()
    #     if ord('0') in keys or ord('1') in keys:
    #         print_keys = list(keys)[0]
    #         print(chr(print_keys))
    #         break
    #     else:
    #         pass

    # p.setTimeStep(const.TIME_STEP)
    # G_magnitude = -9.8
    # vertical_force_angle = 0.20
    # horizontal_force_angle = np.random.uniform(0, 2*const.PI)

    yaw = 180
    pitch=0
    camera_speed = 0.5
    distance = 5.0
    height = 3
    utils.set_camera(distance, yaw, pitch, height)
    # Rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    # mp4logging = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'test1.mp4')
    # for i in range(200):
    #     distance -= 0.01
    #     utils.set_camera(distance, yaw, pitch, height)
    #     time.sleep(const.TIME_STEP)

    # for i in range(int(360/camera_speed)):
    #     utils.set_camera(distance, yaw, pitch, height)
    #     if yaw>360:
    #         yaw=0
    #     yaw += camera_speed
    #     time.sleep(const.TIME_STEP)
    # p.stopStateLogging(mp4logging)

    # for i in range(int(270/camera_speed)):
    # while 1:
    #     utils.set_camera(distance, yaw, pitch, height)
    #     if yaw>360:
    #         yaw = 0
    #     yaw += camera_speed
    #     p.stepSimulation()
    # 
    #     time.sleep(const.TIME_STEP)

    p.setGravity(0,0,-9.8)

    # for i in range(500):
    #     p.stepSimulation()
    #     time.sleep(const.TIME_STEP)

    # for i in np.arange(499,0,-1):
    #     for j, boxID in enumerate(boxIDs):
    #         p.resetBasePositionAndOrientation(boxID, pos_traj[i,j,:], ori_traj[i,j,:])
    #     time.sleep(const.TIME_STEP*2)
        # time.sleep(2)

    # for i in [0]*100:
    #     for j, boxID in enumerate(boxIDs):
    #         p.resetBasePositionAndOrientation(boxID, pos_traj[i,j,:], ori_traj[i,j,:])
    #     time.sleep(const.TIME_STEP)


    p.disconnect()





