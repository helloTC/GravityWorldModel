import pybullet as p
import pybullet_data
from PhysicalEngine.utils import macro_const
import numpy as np
import time

const = macro_const.Const()

def create_floor(urdf=None, color=None, texture=None, client=0):
    """
    Create floor
    -------------
    urdf[.urdf]: plane urdf
    color[list]: RGBA, plane color.
    texture[.jpg/.png]: texture image

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
    return planeID

def create_cylinder(position, radius, height, color=None, texture=None, mass=1, client=0, isCollision=True):
    """
    create cylinder in physical scene.
    ----------------------
    position[3-element tuple]: Center position of the cylinder
    radius[float]: radius of the cylinder
    height[float]: height of the cylinder
    color[4-element tuple]: rgba color
    texture[.jpg/.png]: texture image
    mass[float]: mass of the cylinder
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
    return cylinderID

def create_box(position, size, color=None, texture=None, mass=1, client=0, isCollision=True):
    """ 
    create one box in physical scene
    --------------------------
    position[3-element tuple]: position (center) of the box
    size[3-element tuple]: size of the box
    color[list]: RGBA color
    texture[.jpg/.png]: texture image
    mass[float]: mass of the box
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
                                                   halfExtents=np.array(size)/2,                                                   physicsClientId=client)
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
    return boxID

def check_box_in_ground(boxID, tol=0.001):
    """
    Check if box located in ground
    -------------------------
    boxID: box ID 
    tol: tolence that boxs located in ground.

    Return:
    -------
    Bool: in/not in ground
    """
    minPos, maxPos = p.getAABB(boxID)
    if np.abs(minPos[-1]) < tol:
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
    for boxID in boxIDs:
        # print('Box {}'.format(boxID))
        while 1:
            p_min, p_max = p.getAABB(boxID)
            p.stepSimulation()
            lin_vec, ang_vec = p.getBaseVelocity(boxID)
            velocity = np.sum((lin_vec-np.zeros(3))**2+(ang_vec-np.zeros(3))**2)
            # print('  Velocity: {}'.format(velocity))
            if velocity < velocity_tol:
                p.resetBaseVelocity(boxID, [0,0,0], [0,0,0])
                break
            else:
                p.resetBaseVelocity(boxID, [0,0,0], [0,0,0])
    # Get position of each box
    box_pos_all = []
    box_ori_all = []
    for boxID in boxIDs:
        box_pos, box_ori = p.getBasePositionAndOrientation(boxID)
        box_pos_all.append(box_pos)
        box_ori_all.append(box_ori)
    return box_pos_all, box_ori_all
    
def adjust_confg_position(boxIDs, sigma):
    """
    Adjust configuration by adding position noise to each box object
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
        # No noise in Z axis
        pos_nos[-1] = 0
        # print('Noise is {}'.format(pos_nos))
        # Add noise
        box_pos_nos = box_pos + pos_nos
        p.resetBasePositionAndOrientation(boxID, box_pos_nos, box_ori)
    # Position correction, in case of interobject penetration
    box_pos_all, box_ori_all = object_overlap_correct(boxIDs)
    return box_pos_all, box_ori_all  

def prepare_force_noise(boxIDs, f_mag, f_angle):
    """
    Prepare force noise.
    Force noise was only added into the box which located in ground.
    ------------------
    boxIDs[list]: All boxes in the configuration.
    f_mag[float]: Force Magnitude.
    f_angle[float]: Force Angle, need to transfer into radian measure.

    Return:
    -------
    targboxID[int]: the box ID needed to be forced.
    forceObj[Three-dimension list]: Force vector to be applied.
    positionObj[Three-dimension list]: Position vector to be applied.
    """
    # Find Box located in ground ----------
    isground = []
    for boxID in boxIDs:
        isground.append(check_box_in_ground(boxID))
    targbox = np.random.choice(np.where(isground)[0])
    # Force was interacted into this box.
    targboxID = boxIDs[targbox]
    # Get Position of this targbox
    box_pos, box_ori = p.getBasePositionAndOrientation(targboxID)
    # Prepare force -----------------------
    # Force orientation: uniform over the range [0, 360]
    forceObj = [f_mag*np.cos(f_angle), f_mag*np.sin(f_angle), 0]
    posObj = box_pos
    return targboxID, forceObj, posObj 

def examine_stability(box_pos_ori, box_pos_fin, tol=0.05):
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

def run_IPE(boxIDs, pos_sigma, force_magnitude, force_time=0.2, n_iter=1000):
    """
    Run model of intuitive physical engine. Add position noise and force noise to the configuration and evaluate confidence under each parameter pair.
    Note that for position noise, we adjust position of each box, for force noise, we add force to the box that located in the ground (randomly add forces into one box). Direction of force was uniformly sampled under range around [0, 2*PI]
    -------------------
    boxIDs[list]: box IDs.
    pos_sigma[float]: Position noise was added as a horizontal gaussian noise. The gaussian noise follows N~(0, sigma).
    force_magnitude[float]: force magnitude.
    force_time[float]: add force within the first n seconds.
    n_iter[int]: IPE iterations.

    Return:
    --------
    confidence[bool list]: stability confidence
    """
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
        targboxID, force_arr, position_arr = prepare_force_noise(boxIDs, force_magnitude, force_angle)
        # Get original position of each box
        # For we examine position difference along z axis, here we do not record orientation of each box.
        box_pos_ori = []
        for boxID in boxIDs:
            box_pos_tmp, _ = p.getBasePositionAndOrientation(boxID)
            box_pos_ori.append(box_pos_tmp)
        # Simulation
          # Set gravity
        p.setGravity(0,0,const.GRAVITY)
        for i in range(300):
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
        if (True in isstable):
            print('{}:Unstable'.format(n+1))
        else:
            print('{}:Stable'.format(n+1))
        confidence.append(True in isstable)
        # Finally, initialize configuration
        for i, boxID in enumerate(boxIDs):
            for i, boxID in enumerate(boxIDs):
                p.resetBasePositionAndOrientation(boxID, box_pos_ini[i], box_ori_ini[i])
    return confidence
          




