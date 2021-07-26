import pybullet as p
import pybullet_data
from PhysicalEngine.utils import macro_const
import numpy as np
import time

const = macro_const.Const()

def create_floor(urdf=None, color=None, texture=None, friction=0.1, client=0):
    """
    Create floor
    -------------
    urdf[.urdf]: plane urdf
    color[list]: RGBA, plane color.
    texture[.jpg/.png]: texture image
    friction[float]: lateral friction of the floor

    Return:
    --------
    floorID: floor ID.
    """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    if urdf is None:
        urdf = 'plane.urdf'
    planeID = p.loadURDF('plane.urdf')

    if color is not None:
        # Load texture
        p.changeVisualShape(planeID, -1, rgbaColor=color, physicsClientId=client)

    if texture is not None:                                              
        textureID = p.loadTexture(texture)    
        p.changeVisualShape(planeID, -1, textureUniqueId=textureID, physicsClientId=client)
    # Change lateral friction
    p.changeDynamics(planeID, -1, lateralFriction=friction)
    return planeID

def create_cylinder(position, radius, height, color=None, texture=None, mass=1, friction=0.1, client=0, isCollision=True):
    """
    create cylinder in physical scene.
    ----------------------
    position[3-element tuple]: Center position of the cylinder
    radius[float]: radius of the cylinder
    height[float]: height of the cylinder
    color[4-element tuple]: rgba color
    texture[.jpg/.png]: texture image
    mass[float]: mass of the cylinder
    friction[float]: lateral friction
    isCollision[Bool]: is collision or not.

    Return:
    -------
    cylinderID: cylinder ID
    """
    cylinderVisualShape = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                              radius=radius,
                                              length=height,
                                              physicsClientId=client)
    if isCollision:
        cylinderCollisionShape = p.createCollisionShape(
                                              shapeType=p.GEOM_CYLINDER,
                                              radius=radius,
                                              height=height,
                                              physicsClientId=client)
    else:
        cylinderCollisionShape = const.NULL_OBJ

    cylinderID = p.createMultiBody(
                              baseMass=mass, 
                              baseCollisionShapeIndex=cylinderCollisionShape,
                              baseVisualShapeIndex=cylinderVisualShape,
                              basePosition=position,
                              physicsClientId=client
                              )

    if color is not None:
    # change color
        p.changeVisualShape(cylinderID, -1, rgbaColor=color, physicsClientId=client)
    if texture is not None:
    # change texture
        textureID = p.loadTexture(texture)    
        p.changeVisualShape(cylinderID, -1, textureUniqueId=textureID, physicsClientId=client)
    # Set lateral friction
    p.changeDynamics(cylinderID, -1, lateralFriction=friction)
    return cylinderID

def create_box(position, size, color=None, texture=None, mass=1, friction=0.1, client=0, isCollision=True):
    """ 
    create one box in physical scene
    --------------------------
    position[3-element tuple]: position (center) of the box
    size[3-element tuple]: size of the box
    color[list]: RGBA color
    texture[.jpg/.png]: texture image
    mass[float]: mass of the box
    friction[float]: lateral friction
    client: physics client
    isCollision[Bool]: Consider collision or not.

    Return:
    -------
    boxID: box ID
    """    
    boxVisualShape = p.createVisualShape(shapeType=p.GEOM_BOX,    
                                   halfExtents=np.array(size)/2,    
                                   rgbaColor=color,    
                                   physicsClientId=client)    
    if isCollision:    
        boxCollisionShape = p.createCollisionShape(shapeType=p.GEOM_BOX,    
                                                   halfExtents=np.array(size)/2,
                                                   physicsClientId=client)
    else:
        boxCollisionShape = const.NULL_OBJ

    boxID = p.createMultiBody(baseVisualShapeIndex=boxVisualShape,
                          baseCollisionShapeIndex=boxCollisionShape,   
                          basePosition=position,
                          baseMass=mass,
                          physicsClientId=client)

    if color is not None:
        # change color
        p.changeVisualShape(boxID, -1, rgbaColor=color, physicsClientId=client)
    if texture is not None:
    # change texture
        textureID = p.loadTexture(texture)    
        p.changeVisualShape(boxID, -1, textureUniqueId=textureID, physicsClientId=client)
    # Set lateral friction
    p.changeDynamics(boxID, -1, lateralFriction=friction)
    return boxID

def check_box_in_ground(boxID, ground_height=0.0, tol=0.001):
    """
    Check if box located in ground
    -------------------------
    boxID[int]: box ID 
    ground_height[float]: baseline of the ground, if height of ground is not 0, provide it here. 
    tol[float]: tolence that boxs located on ground.

    Return:
    -------
    Bool: in/not on ground
    """
    minPos, maxPos = p.getAABB(boxID)
    if np.abs(minPos[-1]) < tol + ground_height:
        return True
    else:
        return False

def object_overlap_correct(boxIDs, velocity_tol=0.005):
    """
    Correct objects to prevent interobject penetrations
    Using Simulation to re-organize objects' configuration
    --------------------------------------------
    boxIDs[list]: box IDs
    velocity_tol[float]: velocity tolerance, default is 0.005

    Return:
    -------
    box_pos_all[three-dimensional list]: corrected configuration, position.
    box_ori_all[three-dimensional list]: corrected configuration, orientation. 
    """
    while 1:
        p.setGravity(0,0,0)
        velocity = 0
        p.stepSimulation()
        for boxID in boxIDs:
            lin_vec_tmp, ang_vec_tmp = p.getBaseVelocity(boxID)
            velocity_tmp = np.sum(np.array(lin_vec_tmp)**2+np.array(ang_vec_tmp)**2)
            velocity += velocity_tmp
            # print('  Velocity: {}'.format(velocity))
            p.resetBaseVelocity(boxID, [0,0,0], [0,0,0])
        if velocity < velocity_tol:
            break
    # Get position of each box
    box_pos_all = []
    box_ori_all = []
    for boxID in boxIDs:
        box_pos, box_ori = p.getBasePositionAndOrientation(boxID)
        box_pos_all.append(box_pos)
        box_ori_all.append(box_ori)
    return box_pos_all, box_ori_all

def adjust_box_size(boxIDs, sigma):
    """
    Adjust object size by adding shape noise to each object object.
    ----------------------
    boxIDs[list]: All boxes in the configuration
    sigma[float]: Shape noise was added as a horizontal gaussian noise.
                  The gaussian noise follows N~(0, sigma)

    Return:
    -------
    box_size_all: new shape of each box
    box_pos_all: new position of each box
    box_ori_all: new orientation of each box
    """
    box_size_all = []
    box_pos_all = []
    box_ori_all = []
    box_color_all = []
    box_mass_all = []
    box_friction_all = []
    for i, boxID in enumerate(boxIDs):
        # Get box shape
        box_size = np.array(p.getVisualShapeData(boxID)[0][3])
        box_color = p.getVisualShapeData(boxID)[0][-1]
        box_color_all.append(box_color)
        # Get box dynamics
        box_mass = p.getDynamicsInfo(boxID,-1)[0]
        box_friction = p.getDynamicsInfo(boxID,-1)[1]
        box_mass_all.append(box_mass)
        box_friction_all.append(box_friction)
        # Get box position
        box_pos, box_ori = p.getBasePositionAndOrientation(boxID)
        box_pos_all.append(box_pos)
        box_ori_all.append(box_ori)
        # Prepare Gaussian noise
        size_nos = np.random.normal(0, sigma, 3)
        size_nos[-1] = 0
        # Change Shape
        box_size_nos = box_size + size_nos
        box_size_all.append(box_size_nos)
    # Remove bodies
    for i, boxID in enumerate(boxIDs):
        p.removeBody(boxID)
        boxID = create_box(box_pos_all[i], box_size_all[i], box_color_all[i], mass=box_mass_all[i], friction=box_friction_all[i])
    # Position correction, in case of interobject penetration
    box_pos_all, box_ori_all = object_overlap_correct(boxIDs)
    return box_size_all, box_pos_all, box_ori_all

def adjust_confg_position(boxIDs, sigma):
    """
    Adjust configuration by adding position noise to each box object.
    -------------------------------
    boxIDs[list]: All boxes in the configuration
    sigma[float]: Position noise was added as a horizontal gaussian noise.
                  The gaussian noise follows N~(0, sigma)

    Return:
    -------
    box_pos_all: new position of each box
    box_ori_all: new orientation of each box
    """
    for boxID in boxIDs:
        # Get box position
        box_pos, box_ori = p.getBasePositionAndOrientation(boxID)
        # Prepare Gaussian Noise
        pos_nos = np.random.normal(0, sigma, 3)
        # No noise along Z axis
        pos_nos[-1] = 0
        # print('Noise is {}'.format(pos_nos))
        # Add noise
        box_pos_nos = box_pos + pos_nos
        p.resetBasePositionAndOrientation(boxID, box_pos_nos, box_ori)
    # Position correction, in case of interobject penetration
    box_pos_all, box_ori_all = object_overlap_correct(boxIDs)
    return box_pos_all, box_ori_all

def prepare_force_noise_inground(boxIDs, f_mag, f_angle, ground_height=0.0):
    """
    Prepare force noise.
    Force noise was only added into the box which located in ground.
    ------------------
    boxIDs[list]: All boxes in the configuration.
    f_mag[float]: Force Magnitude.
    f_angle[float]: Force Angle, need to transfer into radian measure.
    ground_height[float]: ground height.

    Return:
    -------
    targboxID[int]: the box ID needed to be forced.
    forceObj[Three-dimension list]: Force vector to be applied.
    PosObj[Three-dimension list]: Position vector to be applied.
    """
    # Find Box located in the ground ----------
    isground = []
    for boxID in boxIDs:
        isground.append(check_box_in_ground(boxID, ground_height=ground_height))
    if sum(isground)>0:
        targbox = np.random.choice(np.where(isground)[0])
    else:
        # Exception happened.
        print('No box was found located in the ground.')
        targbox = None
    if targbox is not None:
        # Force was interacted into this box.
        targboxID = boxIDs[targbox]
        # Get Position of this targbox
        box_pos, box_ori = p.getBasePositionAndOrientation(targboxID)
        # Prepare force -----------------------
        # Force orientation: uniform over the range [0, 360]
        forceObj = [f_mag*np.cos(f_angle), f_mag*np.sin(f_angle), 0]
        posObj = box_pos
    else:
        targboxID = None
        forceObj = None
        posObj = None
    return targboxID, forceObj, posObj 

def prepare_force_allblocks(boxIDs, f_mag, f_angle):
    """
    Prepare force noise
    Force noise was added into all boxes of the configuration
    ----------------
    boxIDs[list]: All boxes in the configurations.
    f_mag[float]: Force Magnitude.
    f_angle[float]: Force Angle, need to transfer into radian measure.

    Return:
    --------
    forceObj[list]: Force vector to be appied. Each element is a three dimensional list indicated force in x, y and z axis. Noted that force=0 in z axis.
    PosObj[list]: Position vector to be applied. Each element is a three dimensional list indicated position in x, y and z axis. The positionObj inherited from position of each box.
    """
    forceObj = []
    PosObj = []
    for boxID in boxIDs:
        box_pos, box_ori = p.getBasePositionAndOrientation(boxID)
        # Prepare force
        # Force orientation: uniform over the range [0, 360]
        forceVector = [f_mag*np.cos(f_angle), f_mag*np.sin(f_angle), 0]
        forceObj.append(forceVector)
        PosObj.append(box_pos)
    return forceObj, PosObj

def examine_stability(box_pos_ori, box_pos_fin, tol=0.01):
    """
    Examine the stability of the configuration.
    Stability was evaluated by checking position difference of the original configuration and the final configuration.
    ---------------------------
    box_pos_ori[three-dim list]: original box positions
    box_pos_fin[three-dim list]: final box positions
    
    Return:
    -------
    isstable[bool list]: whether configuration is stable or not, each element represent one box in the configuration.
    """
    assert len(box_pos_ori) == len(box_pos_fin), "Need to use the same configuration."
    box_num = len(box_pos_ori)
    isstable = []
    for i in range(box_num):
        # Consider its z axis shift 
        pos_diff = (box_pos_ori[i][-1] - box_pos_fin[i][-1])**2
        if pos_diff > tol:
            # print('Box {} moved'.format(i+1))
            isstable.append(True)
        else:
            isstable.append(False)
    return isstable

def run_IPE(boxIDs, pos_sigma, force_magnitude, force_time=0.2, ground_height=0.0, n_iter=1000, shownotion=True):
    """
    Run model of intuitive physical engine. Add position noise and force noise to the configuration and evaluate confidence under each parameter pair.
    Note that for position noise, we adjust position of each box, for force noise, we add force to the box that located in the ground (randomly add forces to one of the boxes). Direction of force was uniformly sampled under range around [0, 2*PI]
    -------------------
    boxIDs[list]: box IDs.
    pos_sigma[float]: Position noise was added as a horizontal gaussian noise. The gaussian noise follows N~(0, sigma).
    force_magnitude[float]: force magnitude.
    force_time[float]: add force within the first n seconds.
    ground_height[float]: ground height.
    n_iter[int]: IPE iterations.
    shownotion[bool]: Whether show notation of stability. By default is True.

    Return:
    --------
    confidence[bool list]: stability confidence
    """
    print('IPE Simulation with parameters {}:{}'.format(pos_sigma, force_magnitude))
    confidence = []
    # Record initial configuration
    box_pos_ini, box_ori_ini = [], []
    for boxID in boxIDs:
        box_pos_tmp, box_ori_tmp = p.getBasePositionAndOrientation(boxID)
        box_pos_ini.append(box_pos_tmp)
        box_ori_ini.append(box_ori_tmp)
    # Start Simulation
    for n in range(n_iter):
        # First, adjust position of the configuration.
        box_pos_adj, box_ori_adj = adjust_confg_position(boxIDs, pos_sigma)
        for i, boxID in enumerate(boxIDs):
            p.resetBasePositionAndOrientation(boxID, box_pos_adj[i], box_ori_adj[i])
        # Second, prepare force noise
          # force angle generated uniformly
        force_angle = np.random.uniform(0, 2*const.PI)
        targboxID, force_arr, position_arr = prepare_force_noise_inground(boxIDs, force_magnitude, force_angle, ground_height=ground_height)
        if targboxID is not None:
            # targboxID is None indicated no box located in the ground.
            # No need to do simulation for we have no idea on the force during simulation.
            # Here, simulation could be done
            # Get original position of each box
            # For we examine position difference along z axis, here we do not record orientation of each box.
            box_pos_ori = []
            for boxID in boxIDs:
                box_pos_tmp, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_ori.append(box_pos_tmp)
            # Simulation
              # Set gravity
            p.setGravity(0,0,const.GRAVITY)
            for i in range(400):
                p.stepSimulation()
                time.sleep(const.TIME_STEP)
                if i<force_time/(const.TIME_STEP): # Add force within the first 200ms
                    # Add force to the target box
                    p.applyExternalForce(targboxID, -1, force_arr, position_arr, p.WORLD_FRAME)
            # Evaluate stability
              # Get base position of the configuration
            box_pos_fin = []
            for boxID in boxIDs:
                box_pos_tmp, _ = p.getBasePositionAndOrientation(boxID)
                box_pos_fin.append(box_pos_tmp)
              # Examine stability
            isstable = examine_stability(box_pos_ori, box_pos_fin)
            if shownotion:
                if (True in isstable):
                    print('  {}:Unstable'.format(n+1))
                else:
                    print('  {}:Stable'.format(n+1))
            confidence.append(True in isstable)
        # Finally, initialize configuration
        for i, boxID in enumerate(boxIDs):
            p.resetBasePositionAndOrientation(boxID, box_pos_ini[i], box_ori_ini[i])
    return confidence

def place_boxes_on_space(box_num, box_size_all, pos_range_x = (-1, 1), pos_range_y = (-1, 1), overlap_thr_x=0.50, overlap_thr_y=0.50):
    """
    Place boxes on space one by one.
    This algorithm was set as follows:
    We iteratively generate box with its horizontal position located within the pos_range. If it did not overlap with previous generated boxes, then it located in the ground. If it overlapped with some of previous boxes, we placed it to the highest position among all previous boxes. 
    Larger pos_range ensures lower height of this configuration, otherwise also works.
    --------------------------
    box_num[int]: the number of boxes.
    box_size_all[list/tuple]: box size list/tuple. Each element in the list was a three-dimensional list. The actual size of the box was randomly selected from this list.
    pos_range_x[two-element tuple]: the maximum horizontal position in x axis that the box could be placed. Tuple indicated (min_x, max_x) in the space.
    pos_range_y[two-element tuple]: the maximum horizontal position in y axis that the box could be placed. Tuple indicated (min_y, max_y) in the space.
    overlap_thr_x[float]: A parameter to control stability of the stimuli. It indicated the minimal overlap between two touched boxes when the stimuli was generated. The value means proportion of the length in x axis of the present box.
    overlap_thr_y[float]: A parameter to control stability of the stimuli. It indicated the minimal overlap between two touched boxes when the stimuli was generated. The value means proportion of the length in y axis of the present box.

    Returns:
    box_pos[list]: all box positions in the configuration.
    boxsize_idx_all[list]: indices which corresponding to box_size_all in order to save the potential size of each box.
    """
    box_pos_all = []
    boxsize_idx_all = []
    for n in range(box_num):
        present_box_num = len(box_pos_all)
        # Decide size of the box
        box_size_idx = np.random.choice(len(box_size_all))
        box_size_now = box_size_all[box_size_idx]
        assert len(box_size_now) == 3, "Size of the box is a three-dimensional list."
        # Randomly generate a position for the box
        x_pos = np.random.uniform(pos_range_x[0], pos_range_x[1])
        y_pos = np.random.uniform(pos_range_y[0], pos_range_y[1])
        # Check if this box was overlapped with previous boxes
        if present_box_num == 0:
            # If no previous box, place the new one in the ground.
            box_pos_all.append([x_pos, y_pos, box_size_now[-1]/2])
        else:
            # If there are boxes, examine if the new one overlapped with previous configuration.
            z_pos = 0 + box_size_now[-1]/2
            for i, box_pos_prev in enumerate(box_pos_all):
                # Get box size
                box_size_prev = box_size_all[boxsize_idx_all[i]]
                pos_x_diff = np.abs(x_pos-box_pos_prev[0])
                pos_y_diff = np.abs(y_pos-box_pos_prev[1])
                # Overlap in x/y axis
                overlap_x = (box_size_now[0]+box_size_prev[0])/2 - pos_x_diff
                overlap_y = (box_size_now[1]+box_size_prev[1])/2 - pos_y_diff
                if (overlap_x>0) & (overlap_y>0):
                    # Exclude situations that two boxes with small overlapping.
                    # We correct the position of the present box.
                    if overlap_x < overlap_thr_x * box_size_now[0]:
                        # If overlap is too small, then correct it into a fix distance: overlap_thr_x*box_size_now
                        x_correct_dist = (box_size_now[0]+box_size_prev[0])/2 - overlap_thr_x * box_size_now[0]
                        if x_pos < box_pos_prev[0]:
                            x_pos = box_pos_prev[0] - x_correct_dist
                        else:
                            x_pos = box_pos_prev[0] + x_correct_dist
                    if overlap_y < overlap_thr_y * box_size_now[1]:
                        # Same judgment in y axis.
                        y_correct_dist = (box_size_now[1]+box_size_prev[1])/2 - overlap_thr_y * box_size_now[1]
                        if y_pos < box_pos_prev[1]:
                            y_pos = box_pos_prev[1] - y_correct_dist
                        else:
                            y_pos = box_pos_prev[1] + y_correct_dist
                    # Overlap, check if we need to update z axis of the new box.
                    z_obj_prev = box_pos_prev[-1] + box_size_prev[-1]/2 + box_size_now[-1]/2
                    if z_obj_prev > z_pos:
                        z_pos = 1.0*z_obj_prev
                else:
                    # No overlap just pass this iteration.
                    pass
            box_pos_all.append([x_pos, y_pos, z_pos])
        boxsize_idx_all.append(box_size_idx)
    return box_pos_all, boxsize_idx_all

def overlap_between_twoboxes(boxID1, boxID2):    
    """    
    Calculate overlap between two boxes in an axis.     
    ----------------------------------    
    boxID1[int]: ID of the first box    
    boxID2[int]: ID of the second box       
    
    Return:    
    ----------    
    overlap[three-dimensional array]: overlap in three axis. Note that the negative overlap indicated no overlap.
    """    
    # Get shape of the two boxes    
    boxshape1 = np.array(p.getVisualShapeData(boxID1)[0][3])    
    boxshape2 = np.array(p.getVisualShapeData(boxID2)[0][3])    
    # Get position of the two boxes    
    boxpos1 = np.array(p.getBasePositionAndOrientation(boxID1)[0])    
    boxpos2 = np.array(p.getBasePositionAndOrientation(boxID2)[0])    
    # Overlap = 0.5(shape1+shape2) - posdiff    
    overlap = 0.5*(boxshape1+boxshape2)-np.abs(boxpos1-boxpos2)             
    return overlap 







