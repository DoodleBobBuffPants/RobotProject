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


def get_desired_positions(current_poses, formation):
    """
    current_poses: a list of current poses [X, Y, YAW] of each of the robots
    formation: a list of points defining each robot position in the formation relative to the formation origin

    returns: 
    list of desired positions i.e. where each robot should be in the formation
    """
    desired_poses = [[0, 0, 0] for robot_pose in current_poses]

    retrun desired_poses

def maintain_formation(current_poses, update_velocities):
    """
    current_poses: a list of current poses [X, Y, YAW] of each of the robots
    update_velocities: a list of velocities that direct each robot in the next step 

    returns: 
    velocities: list of vectors [x, y] indicating the velocities of each robot needed to maintain formation
    """
    # TODO Unit reference
    # Use average of all the robot position to work out a unit center
    unit_reference = [0, 0]

    # TODO The orientation of this formation is defined by a line from the unit center to the next navigational waypoint
    # The unit center and formation orientation together define a local coordinate system around whih the new formation is based
    formation_orientation = 0

    # Desired positions of each of the robots in the formation
    desired_poses = get_desired_positions(current_poses)

    # Vector pointing from the current position to the desired position
    # TODO x, y, yaw -> velocities?
    # TODO first get unit direction vector and then multiply it by a number to get a magnitude dpeending on distance
    velocities = desired_poses - current_poses

    return velocities

# weighted (x,y) -> feedback lin (u, w)