import numpy as np
from init_formations import FORMATION, SPACING_DIST, INITIAL_YAW

X = 0
Y= 1
YAW = 2

ROBOT_RADIUS = 0.105 / 2.

# DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
DEAD_ZONE = ROBOT_RADIUS + (ROBOT_RADIUS / 2.)
# CONTROL_ZONE (if robot is within the controlled zone, velocity towards position linearly increases the further away it is)
CONTROLLED_ZONE = DEAD_ZONE + SPACING_DIST

def get_desired_positions(formation, formation_pose):
    """
    formation: a list of points defining each robot position in the formation relative to the formation origin
    formation_pose: origin of formation (unit-reference) where YAW is the orientation of the formation

    returns: 
    desired_positions: list of desired positions i.e. where each robot should be in the formation
    """

    # Take formation and transform (rotate, translate) onto formation_pose
    theta = formation_pose[YAW] - INITIAL_YAW

    desired_positions = np.zeros_like(formation)
    for i in range(len(formation)):
        # Rotate
        desired_positions[i] = np.matmul([[np.cos(theta), -np.sin(theta)],
                                          [np.sin(theta),  np.cos(theta)]], formation[i])
        # Translate
        desired_positions[i] = formation_pose[:2] + desired_positions[i]
    return desired_positions

def get_formation_orientation(velocity, leader_pose):
    # Just use leader pose to simulate robots catching up to leader
    return leader_pose 
 

def maintain_formation(leader_pose, follower_poses, leader_rrt_velocity):
    """
    param leader_pose: ground truth pose of the leader
    param follower_poses: the ground truth poses of the followers
    param leader_rrt_velocity: rrt_velocity of the leader

    returns: 
    velocities for followers: list of vectors [x, y] indicating the velocities of each robot needed to maintain formation
    """

    # Formation orientation is the angle of formation given the update velocity and the robots direction.
    formation_orientation = get_formation_orientation(leader_rrt_velocity, leader_pose[YAW])
    formation_pose = np.concatenate((leader_pose[:2], [formation_orientation]))

    # Could we just substitute formation pose for leader pose? DONT DO THIS WITHOUT TESTING FOR ROBUSTNESS

    # Desired positions of each of the follower robots in the formation (see comment above about replacing formation pose with leader...)
    desired_positions = get_desired_positions(formation=FORMATION, formation_pose=formation_pose)

    # velocity directs the follower robots from their current position to their (desired position in the formation)
    follower_positions = np.array([pose[:2] for pose in follower_poses])
    velocities = desired_positions - follower_positions

    # update each velocity (the displacement between the current and desired position) depending on the distance
    for i in range(len(velocities)):
        # euclidian distance
        distance = np.sqrt(np.square(velocities[i][X]) + np.square(velocities[i][Y]))

        # print('rob_pos: %s, rob_despos: %s, form_orient: %s, in_ded_zone: %s' % (follower_poses[i], desired_positions[i], formation_orientation, (distance < DEAD_ZONE)))

        # If a robot is within accepted radius of formation position, velocity should be 0
        # DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
        if distance < DEAD_ZONE:
            velocities[i] = np.zeros_like(velocities[i])
        elif distance < CONTROLLED_ZONE:
            # we do nothing, velocity is directly proportional to distance
            pass
        else:
            # robot is outside the controlled zone, so we normalize the velocity and then multiply
            # by the radius of the control zone.
            velocities[i] = velocities[i] / distance
            velocities[i] = velocities[i] * CONTROLLED_ZONE

    # print("----------------------------------")
    # print("----------------------------------")

    return velocities
