import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const
import numpy as np
from sklearn.linear_model import LogisticRegression
from scipy import stats
import random
import pickle

def gaussian_func(x, sigma, A):
    """
    Build a gaussian function
    """
    y = A*np.exp((-1)*x**2/(2*sigma**2))
    return y

def transfer_gaussian_to_mat(x, sigma, A):
    """
    """
    if sigma == 0:
        gaussian_curve = np.zeros(len(x))
        gaussian_curve[15] = A
    else:
        # Generate gaussian curve
        gaussian_curve = gaussian_func(x, sigma, A)
    # Because gaussian curve is symmetric, we get curves ranges from 0 to 45.
    gaussian_curve = gaussian_curve[15:]
    # Transfer gaussian curve to mat
    judgerat_mat = np.tile(gaussian_curve, (16,1)).T
    return judgerat_mat

def sample_gvtdeg(judgerat, sample_num):
    """
    """
    angle_list = [(i,j) for i in np.linspace(0,45,judgerat.shape[0]) for j in np.linspace(0,360,judgerat.shape[1])]
    sample_angles = random.choices(angle_list, weights=judgerat.flatten(), k=sample_num)
    return sample_angles

def get_configuration_position(center_pos, box_size, orientation=None):
    """
    Get all end points in one block. 
    """
    if orientation is None:
        orientation = (0,0,0,1)
    pos_all = []
    symbols_all = np.array([[1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1], 
                        [-1, 1, 1], [-1, 1, -1], [-1, -1, 1], [-1, -1, -1]])
    for i in range(symbols_all.shape[0]):
        pos_tmp = center_pos + 0.5*np.array(box_size)*symbols_all[i,:]
        pos_proj = stability_func.transform_pos_by_orientation(pos_tmp, orientation)
        pos_all.append(pos_proj)
    pos_all = np.array(pos_all)
    return pos_all

def get_all_pos(boxIDs, box_size, boxidx):
    """
    """
    pos_all = []
    for i, boxID in enumerate(boxIDs):
        box_pos_fin_tmp, box_ori_fin_tmp = p.getBasePositionAndOrientation(boxID)
        pos_tmp = get_configuration_position(box_pos_fin_tmp, box_size[boxidx[i]], box_ori_fin_tmp)
        pos_all.append(pos_tmp)
    pos_all = np.array(pos_all)
    return pos_all

const = macro_const.Const()
def prepare_configurations(nstab=1000, nfall=1000, nbox=None, sigma=0):
    """
    """
    stability_act_all = []
    stability_sim_all = []
    pos_all_act = [] 
    pos_all_sim = [] 

    stab_num = 0
    fall_num = 0

    box_size = np.array([[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]])

    # Do not render images during building
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    # Prepare Train Configurations
    while 1:
        flag = 0
        print('  Stable {}, Fall {}'.format(stab_num, fall_num))
        xmax = np.random.uniform(0.2,2.0)
        ymax = np.random.uniform(0.2,2.0)

        nbox_act = nbox

        boxpos, boxidx = stability_func.place_boxes_on_space(nbox_act, box_size, 
                                        pos_range_x=(-1.0*xmax, xmax), 
                                        pos_range_y=(-1.0*ymax, ymax),
                                        overlap_thr_x=np.random.uniform(0.2, 0.8),
                                        overlap_thr_y=np.random.uniform(0.2, 0.8))
        boxpos = np.array(boxpos)
        ndimensions = nbox_act*8*3

        boxIDs = []
        for i in range(boxpos.shape[0]):
            boxIDs.append(stability_func.create_box(
                    boxpos[i,:], 
                    box_size[boxidx[i]],
                    color=color_list[np.random.choice(len(color_list))],
                    mass=0.2,
                    friction=1)
                    )

        # Set Gravity
        # Prepare gravity
        p.setGravity(0,0,-9.8)

        pos_traj_act = []
        # Perform simulation
        for i in range(500):
            p.stepSimulation()
            pos_traj_tmp = np.zeros((nbox*8*3))
            if (i+1)%5==0:
                pos_traj = get_all_pos(boxIDs, box_size, boxidx)
                pos_traj_tmp[:ndimensions] = pos_traj.flatten()
                pos_traj_act.append(pos_traj_tmp)
        pos_traj_act = np.array(pos_traj_act)

        # Measure stability
        box_pos_fin_act = []
        box_ori_fin_act = []
        for i, boxID in enumerate(boxIDs):
            box_pos_fin, box_ori_fin = p.getBasePositionAndOrientation(boxID)
            box_pos_fin_act.append(box_pos_fin) 
            box_ori_fin_act.append(box_ori_fin)

        box_pos_fin_act = np.array(box_pos_fin_act)

        isstable_all = stability_func.examine_stability(boxpos, box_pos_fin_act)
        stability_act = sum(isstable_all)/len(isstable_all)

        if (stability_act < 1) & (fall_num < nfall):
            flag = 1
            fall_num += 1
            stability_act_all.append(stability_act)
            pos_all_act.append(pos_traj_act)
        elif (stability_act == 1) & (stab_num < nstab):
            flag = 1
            stab_num += 1
            stability_act_all.append(stability_act)
            pos_all_act.append(pos_traj_act)
        else:
            pass

        # Remove configurations
        for boxID in boxIDs:
            p.removeBody(boxID)

        # -------------------------
        # When gravity has variance
        # -------------------------
        if (sigma is not None) & (flag == 1):
            # Re-Generate configurations
            boxIDs = []
            for i in range(boxpos.shape[0]):
                boxIDs.append(stability_func.create_box(
                        boxpos[i,:], 
                        box_size[boxidx[i]],
                        color=color_list[np.random.choice(len(color_list))],
                        mass=0.2,
                        friction=1)
                        )

            # Prepare gravity
            # 0.8358 is the fitted amplitude from human's behavior
            judgerat = transfer_gaussian_to_mat(np.linspace(-45,45,31), sigma, 0.8358)
            sample_angles = sample_gvtdeg(judgerat, 1)
            sample_angles = np.array(sample_angles)
            # Transfer angle to radian
            sample_angles = np.pi*sample_angles/180
            theta = sample_angles[0,0]
            phi = sample_angles[0,1]
            gvt_array = np.array([np.sin(theta)*np.sin(phi),
                                  np.sin(theta)*np.cos(phi),
                                  np.cos(theta)])
            p.setGravity(*(const.GRAVITY*gvt_array))

            pos_traj_sim = []
            # Perform simulation
            for i in range(500):
                p.stepSimulation()
                pos_traj_tmp = np.zeros((nbox*8*3))
                if (i+1)%5==0:
                    pos_traj = get_all_pos(boxIDs, box_size, boxidx)
                    pos_traj_tmp[:ndimensions] = pos_traj.flatten()
                    pos_traj_sim.append(pos_traj_tmp)
            pos_traj_sim = np.array(pos_traj_sim)

            pos_all_sim.append(pos_traj_sim)

            # Measure stability
            box_pos_fin_sim = []
            for i, boxID in enumerate(boxIDs):
                box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_fin_sim.append(box_pos_fin) 

            box_pos_fin_sim = np.array(box_pos_fin_sim)

            isstable_all = stability_func.examine_stability(boxpos, box_pos_fin_sim)
            stability_sim = sum(isstable_all)/len(isstable_all)
            stability_sim_all.append(stability_sim)

            for boxID in boxIDs:
                p.removeBody(boxID)

        if fall_num == nfall & stab_num == nstab:
            break

    stability_act_all = np.array(stability_act_all)
    stability_sim_all = np.array(stability_sim_all)
    pos_all_act = np.array(pos_all_act)
    pos_all_sim = np.array(pos_all_sim)
    return stability_act_all, stability_sim_all, pos_all_act, pos_all_sim 

if __name__ == '__main__':
    AgentID = p.connect(p.DIRECT)
    # AgentID = p.connect(p.GUI, options='--width=512 --height=768 --background_color_red=0 --background_color_green=0 --background_color_blue=0')
    floorID = stability_func.create_floor(color=const.BLACK, friction=1, client=AgentID)

    # Prepare configurations
    box_size = [[0.4, 0.4, 3*0.4], [0.4, 3*0.4, 0.4], [3*0.4, 0.4, 0.4]]
    color_list = [const.RED, const.GREEN, const.BLUE, const.YELLOW, const.PURPLE, const.CYAN, const.BROWN, const.GREY]

    sigma = 18
    acc_sim_dyn_all = []
    acc_act_dyn_all = []
    for niter in range(10):
        acc_sim_dyn = []
        acc_act_dyn = []
        for nbox in np.arange(2,11,1):
            # Get block positions for training data
            stability_train_act, stability_train_sim, pos_train_act, pos_train_sim = prepare_configurations(nstab=100, nfall=100, nbox=nbox, sigma=sigma)
            stability_train_act[stability_train_act<1] = 2
            stability_train_sim[stability_train_sim<1] = 2

            posdiff_train_act = np.zeros_like(pos_train_act)
            posdiff_train_sim = np.zeros_like(pos_train_sim)
            for ntimes in range(100):
                posdiff_train_act[:,ntimes,:] = pos_train_act[:,ntimes,:]-pos_train_act[:,0,:]
                posdiff_train_sim[:,ntimes,:] = pos_train_sim[:,ntimes,:]-pos_train_sim[:,0,:]

            # Get block positions for testing data
            stability_test_act, stability_test_sim, pos_test_act, pos_test_sim = prepare_configurations(nstab=100, nfall=100, nbox=nbox, sigma=sigma)
            stability_test_act[stability_test_act<1] = 2
            stability_test_sim[stability_test_sim<1] = 2

            posdiff_test_act = np.zeros_like(pos_test_act)
            posdiff_test_sim = np.zeros_like(pos_test_sim)
            for ntimes in range(100):
                posdiff_test_act[:,ntimes,:] = pos_test_act[:,ntimes,:]-pos_test_act[:,0,:]
                posdiff_test_sim[:,ntimes,:] = pos_test_sim[:,ntimes,:]-pos_test_sim[:,0,:]

            acc_sim_curv = []
            acc_act_curv = []

            # Calculate prediction accuracies
            for ntimes in range(100):
                model_sim_dyn = LogisticRegression(max_iter=1000)
                model_sim_dyn.fit(posdiff_train_sim[:,ntimes,:], stability_train_act)
                acc_sim = model_sim_dyn.score(posdiff_test_sim[:,ntimes,:], stability_test_act)

                model_act_dyn = LogisticRegression(max_iter=1000)
                model_act_dyn.fit(posdiff_train_act[:,ntimes,:], stability_train_act)
                acc_act = model_act_dyn.score(posdiff_test_act[:,ntimes,:], stability_test_act)

                acc_sim_curv.append(acc_sim)
                acc_act_curv.append(acc_act)
            acc_sim_curv = np.array(acc_sim_curv)
            acc_act_curv = np.array(acc_act_curv)

            acc_sim_dyn.append(acc_sim_curv)
            acc_act_dyn.append(acc_act_curv)
        acc_sim_dyn = np.array(acc_sim_dyn)
        acc_act_dyn = np.array(acc_act_dyn)

        acc_sim_dyn_all.append(acc_sim_dyn)
        acc_act_dyn_all.append(acc_act_dyn)
    acc_sim_dyn_all = np.array(acc_sim_dyn_all)
    acc_act_dyn_all = np.array(acc_act_dyn_all)

    outdata = {}
    outdata['acc_sim_dyn'] = acc_sim_dyn_all
    outdata['acc_act_dyn'] = acc_act_dyn_all

    with open('data/classifyAcc_sigma'+str(sigma)+'.pkl', 'wb') as f:
        pickle.dump(outdata, f)
