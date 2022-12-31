import numpy as np
from PhysicalEngine.utils import stability_func, macro_const
import pickle
import pybullet as p
import random
from multiprocessing import Pool
import matplotlib.pyplot as plt

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

const = macro_const.Const()
def simulate_inference_rdmconfig(rgarea, sigma, nstim=100):
    """
    """
    AgentID = p.connect(p.DIRECT)
    # Load Plane
    floorID = stability_func.create_floor(color=const.BLACK, friction=1, client=AgentID)

    box_size = [[0.4, 0.4, 1.2], [0.4, 1.2, 0.4], [1.2, 0.4, 0.4]]
    stability_real_all = []
    stability_sim_all = []
    # avgheight = []
    maxheight = []

    nstab = 0
    nfall = 0
    while 1:
    # Iterate to get nstim (100) stable and fall stacks
        flag = 0
        print('    Stable {}, Fall {}'.format(nstab, nfall))

        # nbox_act = np.random.randint(2,10)
        nbox_act = 10

        x_max = rgarea
        y_max = rgarea
        overlap_thr_x = np.random.uniform(0.2, 0.8)
        overlap_thr_y = np.random.uniform(0.2, 0.8)

        # Generate Stacks
        boxpos, boxidx = stability_func.place_boxes_on_space(nbox_act, box_size, pos_range_x=(-x_max, x_max), pos_range_y=(-y_max,y_max), 
                                                            overlap_thr_x=overlap_thr_x, overlap_thr_y=overlap_thr_y)
        boxpos = np.array(boxpos)
        boxidx = np.array(boxidx)

        boxIDs = []
        for i in range(len(boxpos)):
            boxIDs.append(stability_func.create_box(
                boxpos[i],
                box_size[boxidx[i]],
                mass = 0.2,
                friction=1
            ))
        
        # Stability inference without noise in real environment
        p.setGravity(0,0,const.GRAVITY)
        for i in range(int(500)):
            p.stepSimulation()
        
        box_pos_fin_real = []
        for i, boxID in enumerate(boxIDs):
            box_pos_fin, _ = p.getBasePositionAndOrientation(boxID)
            box_pos_fin_real.append(box_pos_fin)
        box_pos_fin_real = np.array(box_pos_fin_real)

        isstable_all = stability_func.examine_stability(boxpos, box_pos_fin_real)
        stability_real = sum(isstable_all)/len(isstable_all)

        if (stability_real<1) & (nfall<nstim):
            flag = 1
            nfall += 1
            stability_real_all.append(stability_real)
            # avgheight.append(boxpos[:,-1].mean())
            maxheight.append(boxpos[:,-1].max())
        elif (stability_real == 1) & (nstab < nstim):
            flag = 1
            nstab += 1
            stability_real_all.append(stability_real)
            # avgheight.append(boxpos[:,-1].mean())
            maxheight.append(boxpos[:,-1].max())
        else:
            pass

        # Remove stacks
        for boxID in boxIDs:
            p.removeBody(boxID)

        # -------------------------
        # When graviy has variance
        # -------------------------
        if flag == 1:
            # Re-Generate stacks
            boxIDs = []
            for i in range(boxpos.shape[0]):
                boxIDs.append(stability_func.create_box(
                    boxpos[i,:],
                    box_size[boxidx[i]],
                    mass=0.2,
                    friction=1
                ))
        
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

            # Perform simulation
            for i in range(500):
                p.stepSimulation()

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
    
        if nfall == nstim & nstab == nstim:
            break
    
    # avgheight = np.array(avgheight)
    maxheight = np.array(maxheight)
    stability_real_all = np.array(stability_real_all)
    stability_sim_all = np.array(stability_sim_all)
    p.disconnect()
    # return stability_real_all, stability_sim_all, avgheight
    return stability_real_all, stability_sim_all, maxheight

if __name__ == '__main__':
    # Multiprocessing
    pool = Pool(processes=150)
    multiproc_handle = []
    rgareas = np.arange(0.2,2.0,0.2)

    sigma = 35
    nstim = 100
    for rgarea in rgareas:
        multiproc = pool.apply_async(simulate_inference_rdmconfig, (rgarea, sigma, nstim))
        multiproc_handle.append(multiproc)

    pool.close()
    pool.join()

    stability_real = np.zeros((len(rgareas), 2*nstim))
    stability_sim = np.zeros_like(stability_real)
    avgheight = np.zeros_like(stability_real)

    for i, rgarea in enumerate(rgareas):
        stability_real[i,:] = multiproc_handle[i].get()[0]
        stability_sim[i,:] = multiproc_handle[i].get()[1]
        avgheight[i,:] = multiproc_handle[i].get()[2]

    data = {}
    data['stability_real'] = stability_real
    data['stability_sim'] = stability_sim
    data['avgheight'] = avgheight
    with open('heightprejudice_sigma'+str(sigma)+'.pkl', 'wb') as f:
        pickle.dump(data, f)

    # Plot figures
    stabdiff = stability_sim-stability_real
    stabdiff_mean = stabdiff.mean(axis=-1)
    stabdiff_ste = stabdiff.std(axis=-1)/np.sqrt(199)
    avgheight_mean = avgheight.mean(axis=-1)
    avgheight_ste = avgheight.std(axis=-1)/np.sqrt(199)
    plt.errorbar(avgheight_mean, stabdiff_mean, xerr=avgheight_ste, yerr=avgheight_ste, fmt='|', color='#079AF3')
    # plt.xlim(0.4,2.2)
    plt.xlim(1,7)
    # plt.ylim(-0.5,0)
    plt.ylim(-0.8,0)
    plt.xticks([])
    plt.yticks([])
    plt.savefig('JD_sigma'+str(sigma)+'.png', dpi=300)
    plt.close()
    