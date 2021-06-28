# Scripts to do test of some functions

import pybullet as p
from PhysicalEngine.utils import macro_const
from PhysicalEngine.utils import stability_func
import time
import numpy as np
import pickle

const = macro_const.Const()

if __name__ == '__main__':
    # Link Physical Agent
    # AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    AgentID = p.connect(p.DIRECT)
    # Prepare Floor
    planeID = stability_func.create_floor(color=const.BLACK, client=AgentID)

    # Prepare Cylinder
    # cylinderID = stability_func.create_cylinder(
    #                                   position=(0,0,0),
    #                                   radius=5,
    #                                   height=0,
    #                                   texture='../textures/wood_repeat.png',
    #                                   mass=0)

    # Do Not Render Images during Building    
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)    
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)    
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)  

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
    # pos_list[:,-1]+=0.1/2
    box_list = np.array(params['box_list'])
    boxIDs = []
    for i in range(len(params['box_list'])):
        boxIDs.append(stability_func.create_box(
                     pos_list[i,:], 
                     box_size[box_list[i]],
                     color=color_list[np.random.choice(len(color_list))],
                     mass=0.2)
                    )
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    
    # box_pos_adj, box_ori_adj = stability_func.adjust_confg_position(boxIDs, 0.08)
    # for i, boxID in enumerate(boxIDs):
    #     p.resetBasePositionAndOrientation(boxID, box_pos_adj[i], box_ori_adj[i])
    # p.setGravity(0,0,const.GRAVITY)
    # for i in range(500):
    #     p.stepSimulation()
    #     time.sleep(const.TIME_STEP)

    # box_pos_fin = []
    # for boxID in boxIDs:
    #     box_pos_tmp, box_ori_tmp = p.getBasePositionAndOrientation(boxID)
    #     box_pos_fin.append(box_pos_tmp)
    # stability_func.examine_stability(box_pos_adj, box_pos_fin)

    # run IPE
    confidence = stability_func.run_IPE(boxIDs, 0.05, 0.05)



