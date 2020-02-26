# 5 robots, POSE(X, Y, YAW)

# TODO need some sort of indication mechanism to decide which robot goes to where in each pose
# TODO assign an ID to each robot
# TODO need to configure different posesd desired formation 
#  velocity in:
#   'dead-zone' immediately around the desired formation = 0
#   'controlled-zone' a ring around the dead-zone, velocity linearly decreases from max->0
#   'ballistic-zone` outside of the controlled-zone, velocity is at its max

# TODO: zones around desired formation
# TODO potentially add noise? if get stuck in local min/max

# TODO parameters
# spacing parameter for formation

X = 0
Y= 1
YAW = 2

# Formation spacing parameter
SPACING_DIST = 0.5
LINE = [[-2*SPACING_DIST,0], 
        [SPACING_DIST, 0], 
        [0, 0], 
        [SPACING_DIST, 0], 
        [2*SPACING_DIST, 0]]

def get_desired_positions(formation, formation_pose):
    """
    formation: a list of points defining each robot position in the formation relative to the formation origin
    formation_pose: origin of formation (unit-reference) where YAW is the orientation of the formation

    returns: 
    desired_positions: list of desired positions i.e. where each robot should be in the formation
    """
    # Take formation and transform (rotate, translate) onto formation_pose
    # Rotate 
    theta = formation_pose[YAW]
    formation = np.matmul(
        [[np.cos(theta), -np.sin(theta)],
         [np.sin(theta),  np.cos(theta)]],
        formation
    )
    # Translate
    formation = formation - formation_pose[0:2]

    desired_positions = formation

    return desired_positions

def maintain_formation(current_poses, update_velocities):
    """
    current_poses: a list of current poses [X, Y, YAW] of each of the robots
    update_velocities: a list of velocities that direct each robot in the next step (directions to next navigational waypoint)

    returns: 
    velocities: list of vectors [x, y] indicating the velocities of each robot needed to maintain formation
    """
    # Unit reference
    # Use average of all the robot position to work out a unit center
    unit_reference = np.sum(current_poses, axis=0) / len(current_poses)

    # The orientation of this formation is defined by a line from the unit center to the next navigational waypoint
    # The unit center and formation orientation together define a local coordinate system around whih the new formation is based
    average_update_velocity = np.sum(update_velocities, axis=0) / len(update_velocities)
    # Formation orientation is the angle of average velocity 
    formation_orientation = np.arctan(average_update_velocity[Y]/average_update_velocity[X])
    formation_pose = np.concatenate((unit_reference, [formation_orientation]))

    # Desired positions of each of the robots in the formation
    desired_positions = get_desired_positions(formation=LINE, formation_pose=formation_pose)

    # Vector pointing from the current position to the desired position
    # TODO x, y, yaw -> velocities?
    # TODO first get unit direction vector and then multiply it by a number to get a magnitude dpeending on distance
    velocities = desired_poses - current_poses

    return velocities

# weighted (x,y) -> feedback lin (u, w)