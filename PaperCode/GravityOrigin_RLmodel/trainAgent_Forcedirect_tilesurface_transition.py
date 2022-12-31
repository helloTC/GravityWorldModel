import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import pickle
import time
import random

const = macro_const.Const()
def trainAgent_RL_gvtsystem(surf_angle, blocknum=None, gvt_magnitude=-9.8, theta_steps=30, phi_steps=30, weight=None, stimuli_num=1000, lr=0.10, simprop=0.2):
    # Link Physical Agent
    # AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    AgentID = p.connect(p.DIRECT)

    # Load Plane
    angle_rad = surf_angle * np.pi/180
    if angle_rad > np.pi/2:
        floorID = stability_func.create_floor(position=(0, np.cos(angle_rad)-np.sin(angle_rad), np.cos(angle_rad)+np.sin(angle_rad)-9*np.cos(angle_rad)), 
                                          orientation=(angle_rad,0,0), 
                                          color=const.BLACK,
                                          friction=1.0, 
                                          client=AgentID)
    
    else:
        floorID = stability_func.create_floor(position=(0, np.cos(angle_rad)-np.sin(angle_rad), np.cos(angle_rad)+np.sin(angle_rad)), 
                                          orientation=(angle_rad,0,0), 
                                          color=const.BLACK,
                                          friction=1.0, 
                                          client=AgentID)

    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # Prepare force direction

    theta_all = np.linspace(0, np.pi, theta_steps)
    phi_all = np.linspace(0, 2*np.pi, phi_steps)

    forceangle_all = [(i,j) for i in theta_all for j in phi_all]
    if weight is None:
        forceangle_weight = np.ones(len(forceangle_all))
    else:
        forceangle_weight = 1.0*weight

    forceangle_all_select = np.zeros(len(forceangle_all))

    # Prepare box
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.RED, const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]

    forceangle_weight_traj = []
    stab_diff_traj = []
    for itr in range(stimuli_num):
        print('Iterate time {}'.format(itr))

        x_max = np.random.uniform(0.2, 2.0)
        y_max = np.random.uniform(0.2, 2.0)
        overlap_thr_x = np.random.uniform(0.2, 0.8)
        overlap_thr_y = np.random.uniform(0.2, 0.8)

        if blocknum is None:
            boxnum_all = np.arange(2,15)
            boxnum = boxnum_all[itr%len(boxnum_all)]
        else:
            boxnum = blocknum
        # Generate configurations
        boxpos, boxidx = stability_func.place_boxes_on_space(boxnum, box_size, pos_range_x=(-x_max,x_max), pos_range_y=(-y_max,y_max), overlap_thr_x=overlap_thr_x, overlap_thr_y=overlap_thr_y)

        boxpos = np.array(boxpos)
        boxpos_rot = 1.0*boxpos
        boxpos[:,-1] += 1
        boxpos[:,-2] += 1
        if angle_rad > np.pi/2:
            boxpos_rot[:,1] = boxpos[:,1]*np.cos(angle_rad)-boxpos[:,2]*np.sin(angle_rad)
            boxpos_rot[:,2] = boxpos[:,2]*np.cos(angle_rad)+boxpos[:,1]*np.sin(angle_rad)-9*np.cos(angle_rad)
        else:
            boxpos_rot[:,1] = boxpos[:,1]*np.cos(angle_rad)-boxpos[:,2]*np.sin(angle_rad)
            boxpos_rot[:,2] = boxpos[:,2]*np.cos(angle_rad)+boxpos[:,1]*np.sin(angle_rad)

        boxpos_rot = np.round(boxpos_rot,2)

        boxIDs = []
        for i in range(len(boxpos_rot)):
            boxIDs.append(stability_func.create_box(
                        boxpos_rot[i,:], 
                        box_size[boxidx[i]],
                        color=color_list[np.random.choice(len(color_list))],
                        mass = 0.2,
                        orientation=(angle_rad, 0, 0),
                        friction=1.0))

        box_pos_orig_real = []
        box_ori_orig_real = []
        for i, boxID in enumerate(boxIDs):
            box_pos_orig, box_ori_orig = p.getBasePositionAndOrientation(boxID)
            box_pos_orig_real.append(box_pos_orig)
            box_ori_orig_real.append(box_ori_orig)
        box_pos_orig_real = np.array(box_pos_orig_real)
        box_ori_orig_real = np.array(box_ori_orig_real)

        # Rendering
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # Simulate according to real environment
        p.setGravity(0, 
                    const.GRAVITY*(-1.0*np.sin(angle_rad)), 
                    const.GRAVITY*(np.cos(angle_rad)))

        for i in range(int(500*simprop)):
            p.stepSimulation()
            # time.sleep(const.TIME_STEP)

        box_pos_fin_real = []
        for i, boxID in enumerate(boxIDs):
            box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
            box_pos_fin_real.append(box_pos_fin)
        box_pos_fin_real = np.array(box_pos_fin_real)

        # Now examine simulation results
        # Made multiple simulations, random select force angles 
        # Prepare force in the simulated engine.
        forceangle_idx = random.choices(np.arange(len(forceangle_all)), weights=forceangle_weight, k=3)

        forceangle_weight_traj.append(np.array(forceangle_weight))
        stab_diff_allsim = []
        for f, fi in enumerate(forceangle_idx):
            for i, boxID in enumerate(boxIDs):
                p.removeBody(boxID)

            # Reconfigurate the stimuli
            boxIDs = []
            for i in range(len(boxpos_rot)):
                boxIDs.append(stability_func.create_box(
                        boxpos_rot[i,:], 
                        box_size[boxidx[i]],
                        color=color_list[np.random.choice(len(color_list))],
                        mass = 0.2,
                        orientation=(angle_rad, 0, 0),
                        friction=1.0))
            
            forceangle_all_select[fi] += 1

            # Tiled Gravity Space
            # norm_array = -1.0*np.array([np.sin(forceangle_all[forceangle_idx][0])*np.sin(forceangle_all[forceangle_idx][1]), 
            #                        np.sin(forceangle_all[forceangle_idx][0])*np.cos(forceangle_all[forceangle_idx][1])*np.cos(angle_rad)-np.cos(forceangle_all[forceangle_idx][0])*np.sin(angle_rad), 
            #                        np.cos(forceangle_all[forceangle_idx][0])*np.cos(angle_rad)+np.sin(forceangle_all[forceangle_idx][0])*np.sin(angle_rad)*np.cos(forceangle_all[forceangle_idx][1])])

            # Upright Gravity Space
            norm_array = np.array([np.sin(forceangle_all[fi][0])*np.sin(forceangle_all[fi][1]), np.sin(forceangle_all[fi][0])*np.cos(forceangle_all[fi][1]), np.cos(forceangle_all[fi][0])])
            # print(norm_array)
            # p.setGravity(*(const.GRAVITY)*norm_array)
            p.setGravity(*(gvt_magnitude)*norm_array)

            # Simulation
            for i in range(int(500*simprop)):
                p.stepSimulation()
                # time.sleep(const.TIME_STEP)

            box_pos_fin_sim = []
            for i, boxID in enumerate(boxIDs):
                box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_fin_sim.append(box_pos_fin) 
            box_pos_fin_sim = np.array(box_pos_fin_sim)
            isstable_all = stability_func.examine_stability(box_pos_fin_real, box_pos_fin_sim)
            stability_update = sum(isstable_all)/len(isstable_all)
            print('stability is {}'.format(stability_update))
            stab_diff_allsim.append(stability_update)


            # Optimize weights
            forceangle_weight[fi] = forceangle_weight[fi] + lr*(stability_update-forceangle_weight[fi])
        stab_diff_allsim = np.array(stab_diff_allsim)
        stab_diff_traj.append(np.mean(stab_diff_allsim))

        for i, boxID in enumerate(boxIDs):
            p.removeBody(boxID)

    p.disconnect()
    forceangle_weight_traj = np.array(forceangle_weight_traj)
    stab_diff_traj = np.array(stab_diff_traj)
    return forceangle_weight, forceangle_weight_traj, forceangle_all, stab_diff_traj

def plot_animation(forceangle, weight_traj, issave=False):
    """
    """
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    a = np.array([i[0] for i in forceangle])
    b = np.array([i[1] for i in forceangle])
    scatter = ax.scatter(a*180/np.pi, b*180/np.pi)
    scatter.set_cmap('rainbow')
    def update(frame):
        ax.set_title('Stimuli {}'.format(frame))
        scatter.set_color(plt.get_cmap('rainbow')(weight_traj[frame, :]))
        return scatter,
    
    anim = animation.FuncAnimation(fig, update, frames=np.arange(weight_traj.shape[0]), interval=10)
    if issave:
        anim.save(issave+'.gif')
    plt.show()

if __name__ == '__main__':
    # weight_180, weight_traj_180, forceangle_180, stab_diff_traj_180 = trainAgent_RL_gvtsystem(0, gvt_magnitude, theta_steps=61, phi_steps=61, stimuli_num=100000, lr=0.15)
    blocknum = None
    weight, weight_traj, forceangle, stab_diff_traj = trainAgent_RL_gvtsystem(0, blocknum=blocknum, theta_steps=61, phi_steps=61, stimuli_num=100000, lr=0.15, simprop=0.30)
    print('0 degrees. Block number {}'.format(blocknum))
    
    outdata = {}
    outdata['weight'] = weight
    outdata['weight_traj'] = weight_traj
    outdata['forceangle'] = forceangle
    outdata['qvalues'] = stab_diff_traj

    # weight_180, weight_traj_180, forceangle_180 = trainAgent_RL_gvtsystem(180, gvt_magnitude, weight=weight_0, theta_steps=61, phi_steps=61, stimuli_num=100000, lr=0.10)

    # gvt_magnitude = -9.8
    # weight_0, weight_traj_0, forceangle_0 = trainAgent_RL_gvtsystem(0, gvt_magnitude, theta_steps=65, phi_steps=65, stimuli_num=100000, lr=0.10)
    # weight_180, weight_traj_180, forceangle_180 = trainAgent_RL_gvtsystem(180, gvt_magnitude, weight=weight_0, theta_steps=65, phi_steps=65, stimuli_num=100000, lr=0.10)
    # print('End of '+str(gvt_magnitude))

    # plot_animation(forceangle_180, weight_traj_180, issave='gvt_180_init')






