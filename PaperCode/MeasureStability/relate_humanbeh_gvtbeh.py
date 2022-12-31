# New code to simulate stability in physical engine.
import pybullet as p
from PhysicalEngine.utils import stability_func, macro_const, utils
import numpy as np
import random
from os.path import join as pjoin
import pickle
import cv2
from multiprocessing import Pool


def load_subjgvtbeh(deg):
    """
    """
    parpath = '/data0/home/liulab/workingdir/Projects/IntuitivePhysics_Stable/Code/GVTdirection_Behavior'
    subjs = ['subj'+str(i) for i in np.arange(1,11)]
    runs = ['1', '2', '3', '4']

    theta_range = np.linspace(0, np.pi/4, 16)
    phi_range = np.linspace(0, 2*np.pi, 16)

    judgerat_runs = np.zeros((16,16, len(runs)))
    judgerat_subj = np.zeros((16,16, len(subjs)))

    for s, subj in enumerate(subjs):
        for r, run in enumerate(runs):
            with open(pjoin(parpath, 'subjects_updated', subj, subj+'_'+deg+'_'+run+'_diffdirect.pkl'), 'rb') as f:
                data = pickle.load(f)
            judgemat = np.zeros((16,16))
            alljudges = np.zeros_like(judgemat)

            theta_idx = np.array([list(theta_range).index(i) for i in data['gravity_direction']])
            phi_idx = np.array([list(phi_range).index(i) for i in data['horizontal_direction']])

            for i, judge in enumerate(data['judge']):
                if judge is None:
                    judge = 0
                judgemat[theta_idx[i], phi_idx[i]] += judge
                alljudges[theta_idx[i], phi_idx[i]] += 1

            judgerat = judgemat/alljudges
            for i in range(16):
                judgerat[i,:][np.isnan(judgerat[i,:])] = judgerat[i,:][~np.isnan(judgerat[i,:])].mean()
            judgerat_runs[...,r] = judgerat
        judgerat_subj[...,s] = judgerat_runs.mean(axis=-1)
    return judgerat_subj

def interp_humanjudge(judgerat, size):
    """
    """
    judgerat = cv2.resize(judgerat, size)
    return judgerat

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
    subjname_all = ['subj'+str(i) for i in range(1,11)]
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
def evaluate_single_configuration(config_name, simtime, judgerat):
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
    n_angles = 60*simtime
    sample_angles = sample_gvtdeg(judgerat, n_angles)
    sample_angles = np.array(sample_angles)
    sample_angles = sample_angles.reshape(60,simtime,2)
    # Transfer angle to radian.
    sample_angles = np.pi*sample_angles/180
    for j in range(60):
        print('Judge {}'.format(j))
        sim_beh_eachjudge = []
        for s in range(simtime):
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
    # Get subject's judgment
    actual_beh_all, config_names = load_humanbeh()
    # Get human's judgement
    judgerat_subj_0 = load_subjgvtbeh('0')
    judgerat_subj_180 = load_subjgvtbeh('180')
    judgerat = (judgerat_subj_0.mean(axis=-1) + judgerat_subj_180.mean(axis=-1))/2
    # Interpolate
    judgerat_int = interp_humanjudge(judgerat, (16,16))
    # Estimate stability
    pool = Pool(processes=150)
    multiproc_handle = []
    # simtimes = [1, 2, 3, 4, 5, 10, 20, 30, 50, 100]
    simtimes = [1]
    for simtime in simtimes:
        multiproc_simtimes = []
        for n, config in enumerate(config_names):
            multiproc = pool.apply_async(evaluate_single_configuration, (config, simtime, judgerat_int))
            multiproc_simtimes.append(multiproc)
        multiproc_handle.append(multiproc_simtimes)
    
    pool.close()
    pool.join()

    sim_beh_all = np.zeros((len(simtimes), len(config_names), 60))
    for i, k in enumerate(simtimes):
        sim_beh_all_config = []
        for n, config in enumerate(config_names):
            multiproc_data = multiproc_handle[i][n].get()
            sim_beh_all[i,n,:] = multiproc_data

    # sim_beh_all = evaluate_single_configuration('106_steady.pkl', 100, judgerat_int)




