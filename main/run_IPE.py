# Scripts to do test of some functions

import pybullet as p
from PhysicalEngine.utils import macro_const
from PhysicalEngine.utils import stability_func
import time
import numpy as np
import os
import pickle

from multiprocessing import Process

const = macro_const.Const()

def main(parpath, config_path, output_path=None, isgui=False):
    """
    Run IPE simulation on a configuration.
    -------------
    parpath[str]: parent path of the configuration.
    config_path[str]: configuration path, end with .pkl
    isgui[bool]: GUI or not

    Return
    """
    # Link Physical Agent
    if isgui:
        AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    else:
        AgentID = p.connect(p.DIRECT)
    # Prepare Floor
    planeID = stability_func.create_floor(color=const.BLACK, friction=0.1, client=AgentID)
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
    with open(os.path.join(parpath, config_path), 'rb') as f:
        params = pickle.load(f)
    assert len(params['pos_list']) == len(params['box_list']), "blocks are mismatched."
    pos_list = np.array(params['pos_list'])
    box_list = np.array(params['box_list'])
    boxIDs = []
    for i in range(len(params['box_list'])):
        boxIDs.append(stability_func.create_box(
                     pos_list[i,:], 
                     box_size[box_list[i]],
                     color=color_list[np.random.choice(len(color_list))],
                     mass=0.2,
                     friction=0.1)
                    )
    if isgui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
   
    # run IPE
    pos_sigma = [0, 0.04, 0.08, 0.12, 0.16, 0.20]
    force_sigma = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]
    conf_all = []
    for pos_tmp in pos_sigma:
        conf_pos = []
        for force_tmp in force_sigma:
            conf_tmp = stability_func.run_IPE(boxIDs, pos_tmp, force_tmp, n_iter=100)
            conf_pos.append(sum(conf_tmp)/100.0)
        conf_all.append(conf_pos)
    conf_all = np.array(conf_all)
    conf_all = 1.0-conf_all
    # Output
    if output_path is None:
        return conf_all
    else:
        np.save(os.path.join(output_path, config_path.split('.')[0]+'.npy'), conf_all)

if __name__ == '__main__':
    # Try Multiprocess
    parpath = '/home/hellotc/PhysicsEngine/PNAS2013_exp1_stimuli_use'
    outpath = '/home/hellotc/workingdir/IntuitivePhysics/IPE_origin'

    process_list = []
    config_name_all = os.listdir(parpath)
    config_name_all.sort()
    for config_name in config_name_all:
        a = Process(target=main, args=(parpath, config_name, outpath, False))
        a.start()
        process_list.append(a)
    
    for i in process_list:
        a.join()


