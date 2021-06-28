# Tile blocks and ask participant to evaluate stability of the configuration
import pybullet as p
from PhysicalEngine.utils import stability_func
from PhysicalEngine.utils import macro_const
import time
import pickle
import numpy as np

const = macro_const.Const()

if __name__ == '__main__':
    # Link Physical Agent
    AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    # Prepare Floor
    planeID = stability_func.create_floor(color=const.BLACK, client=AgentID)
    
    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # Prepare Cylinder
    cylinderID = stability_func.create_cylinder(
                                      position=(0,0,0),
                                      radius=5,
                                      height=0.1,
                                      texture='../textures/wood_repeat.png',
                                      mass=0)

    # Prepare Box
     # Three types of boxes
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.RED, const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]
     # Load box positions
    with open('/home/hellotc/PhysicsEngine/PNAS2013_exp1_stimuli_train/102_steady.pkl', 'rb') as f:
        params = pickle.load(f)
    assert len(params['pos_list']) == len(params['box_list']), "blocks are mismatched."
    pos_list = np.array(params['pos_list'])
    # Add cylinder height
    pos_list[:,-1]+=0.1/2
    box_list = np.array(params['box_list'])
    boxID = []
    for i in range(len(params['box_list'])):
        boxID.append(stability_func.create_box(
                     pos_list[i,:], 
                     box_size[box_list[i]],
                     color=color_list[np.random.choice(len(color_list))],
                     mass=0.2)
                    )
    
    # Rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
   
    # Set Camera
    roll = 0
    camera_speed = 0.2
    radius = 3

    # Show Block Configuration
    for i in range(1350):
        if roll>360:
            roll=0
        roll += camera_speed
        x = radius*np.cos(roll*const.PI/180)
        y = radius*np.sin(roll*const.PI/180)
        p.resetDebugVisualizerCamera(radius, roll+90, -20, [x,y,2])
        time.sleep(const.TIME_STEP)

    # Start Simulation
    p.setGravity(0,0,const.GRAVITY)
    for i in range(1000):
        p.stepSimulation()
        if i<200:
            p.applyExternalForce(boxID[0], -1, [3,0,0], pos_list[0,:], flags=p.WORLD_FRAME)
        time.sleep(const.TIME_STEP)


    #------------- Measure Position Difference -------------# 
    fin_pos = []
    # Get Final Position of each block
    for i in range(len(boxID)):
        fin_pos_tmp, fin_ori_tmp = p.getBasePositionAndOrientation(boxID[i])
        fin_pos.append(fin_pos_tmp)
    fin_pos = np.array(fin_pos)
    pos_diff = np.sum((fin_pos-pos_list)**2, axis=1)
    print('{} blocks were moved'.format(np.sum(pos_diff>0.005)))

    # p.disconnect()








