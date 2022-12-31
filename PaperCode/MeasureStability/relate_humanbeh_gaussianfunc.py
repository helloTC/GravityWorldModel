import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import random
from os.path import join as pjoin
import pickle
import cv2
from multiprocessing import Pool
from scipy.optimize import curve_fit

def gaussian_func(x, sigma, A):
    """
    Build a gaussian function
    """
    y = A*np.exp((-1)*x**2/(2*sigma**2))
    return y


def transfer_mat_to_gaussian(judgerat):
    """
    Transfer judgerat to a gaussian-like curve
    """
    judgerat_pos = judgerat[:,:8]
    judgerat_neg = judgerat[:,8:]
    judgerat_update = np.zeros((31,8))
    judgerat_update[:16,:] += judgerat_neg[::-1,:]
    judgerat_update[15:,:] += judgerat_pos
    judgerat_update[15,:] /= 2
    judgerat_update = judgerat_update.mean(axis=-1)
    return judgerat_update


def fit_gaussian(judgerat_update):
    """
    """
    params, _ = curve_fit(gaussian_func, np.linspace(-45,45,31), judgerat_update)
    sigma, A = params[0], params[1]
    return sigma, A


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


def load_humanbeh():
    """
    """
    # Find configurations
    parpath_beh = '/data0/home/liulab/workingdir/Projects/IntuitivePhysics_Stable/Code/StabilityJudgment_Behavior'
    subjname_all = ['subj'+str(i) for i in np.arange(1,11)]
    runs = ['1', '2', '3', '4', '5', '6']

    actual_beh_all = []
    for _, subj in enumerate(subjname_all):
        print('Subject {}'.format(subj))
        actual_beh_all_subj = []
        for run in runs:
            # Get behavior data
            with open(pjoin(parpath_beh, subj, subj+'_'+run+'_'+'stabjudge.pkl'), 'rb') as f:
                tmp_data = pickle.load(f)
            config_names = np.sort(tmp_data['config_names'])
            actual_beh_all_tmp = tmp_data['behjudge'][np.argsort(tmp_data['config_names'])]
            actual_beh_all_tmp = (7-actual_beh_all_tmp)/7.0
            actual_beh_all_subj.append(actual_beh_all_tmp)
        actual_beh_all_subj = np.array(actual_beh_all_subj)
        actual_beh_all.append(actual_beh_all_subj)
    actual_beh_all = np.array(actual_beh_all)
    return actual_beh_all, config_names


const = macro_const.Const()
def evaluate_single_configuration(config_name, sigma):
    """
    """
    AgentID = p.connect(p.DIRECT)
    # Load Plane
    floorID = stability_func.create_floor(color=const.BLACK, friction=1, client=AgentID)
    # Prepare Box
    box_size = [[0.4,0.4,1.2], [0.4,1.2,0.4], [1.2,0.4,0.4]]
    parpath_config = '/data0/home/liulab/workingdir/Projects/IntuitivePhysics_Stable/BoxStimuli'
    # Simulated behavior
    sim_beh_all = []

    with open(pjoin(parpath_config, config_name), 'rb') as f:
        config_info = pickle.load(f)
    pos_list = np.array(config_info['pos_list'])
    box_list = np.array(config_info['box_list'])

    # Generate Stacks
    boxIDs = []
    for i in range(len(box_list)):
        boxIDs.append(stability_func.create_box(
            pos_list[i,:],
            box_size[box_list[i]],
            mass=0.2,
            friction=1))

    # Each participant perform 60 judgments
    # Each judgment was determined from [100] simulations
    # Prepare angle samples.
    n_angles = 60*100

    # 0.8358 is the fitted amplitude from human's behavior
    judgerat = transfer_gaussian_to_mat(np.linspace(-45,45,31), sigma, 0.8358) 
    sample_angles = sample_gvtdeg(judgerat, n_angles)
    sample_angles = np.array(sample_angles)
    sample_angles = sample_angles.reshape(60,100,2)
    # Transfer angle to radian
    sample_angles = np.pi*sample_angles/180
        
    for j in range(60):
        print('Judge {}'.format(j))
        sim_beh_eachjudge = []
        for s in range(1):
            for i, boxID in enumerate(boxIDs):
                p.resetBasePositionAndOrientation(boxID, pos_list[i,:], [0,0,0,1])
            gvt_array = np.array([np.sin(sample_angles[j,s,0])*np.sin(sample_angles[j,s,1]),
                                  np.sin(sample_angles[j,s,0])*np.cos(sample_angles[j,s,1]),
                                  np.cos(sample_angles[j,s,0])])
            # gvt_array = np.array([0,0,1])
            p.setGravity(*(const.GRAVITY*gvt_array))

            for _ in range(500):
                p.stepSimulation()
            
            box_pos_fin_all = []
            for i, boxID in enumerate(boxIDs):
                box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_fin_all.append(box_pos_fin)
            box_pos_fin_all = np.array(box_pos_fin_all)

            isstable_all = stability_func.examine_stability(pos_list, box_pos_fin_all)
            isstable_prob = np.sum(isstable_all)/len(isstable_all)
            sim_beh_eachjudge.append(isstable_prob)
        sim_beh_all.append(np.mean(sim_beh_eachjudge))
    p.disconnect()
    return sim_beh_all


if __name__ == '__main__':
    # Get subject's stability judgment
    actual_beh_all, config_names = load_humanbeh()
    # Load human gravity judgement
    judgerat = np.load('data/humgvtjudge.npy')
    # Average them together
    judgerat = judgerat.mean(axis=-1) # Different subjects
    judgerat = judgerat.mean(axis=0) # Different directions
    # Transfer mat to gaussian-like curve
    judgerat_update = transfer_mat_to_gaussian(judgerat)

    # Estimate stability
    pool = Pool(processes=150)
    multiproc_handle = []
    # sigma_all = [0,5,10,15,20,25,30,35,40]
    sigma_all = [0]
    for sigma in sigma_all:
        multiproc_sigma = []
        for n, config in enumerate(config_names):
            multiproc = pool.apply_async(evaluate_single_configuration, (config, sigma))
            multiproc_sigma.append(multiproc)
        multiproc_handle.append(multiproc_sigma)
    
    pool.close()
    pool.join()

    sim_beh_all = np.zeros((len(sigma_all), len(config_names), 60))
    for i, k in enumerate(sigma_all):
        sim_beh_all_config = []
        for n, config in enumerate(config_names):
            multiproc_data = multiproc_handle[i][n].get()
            sim_beh_all[i,n,:] = multiproc_data

