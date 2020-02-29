# 5 robots, POSE(X, Y, YAW)

# TODO parameters
# spacing parameter for formation

import numpy as np
import sys, os

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../ros/velocity_controller')
sys.path.insert(0, directory)
from init_formations import FORMATION, SPACING_DIST

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
    print("formation: ", formation)
    print("formation pose: ", formation_pose)
    # Take formation and transform (rotate, translate) onto formation_pose
    # Rotate 
    theta = formation_pose[YAW]

    desired_positions = np.zeros_like(formation)
    for i in range(len(formation)):
        # Rotate
        desired_positions[i] = np.matmul([[np.cos(theta), -np.sin(theta)],
                                          [np.sin(theta),  np.cos(theta)]], formation[i])

        # Translate
        desired_positions[i] = desired_positions[i] + formation_pose[:YAW] # Should be :YAW not :Y
    return desired_positions

def get_formation_orientation(velocity, pose):
    # check if velocity is near zero:
    # threshold = 0.01
    # if np.sqrt(average_update_velocity[X]**2 + average_update_velocity[Y]**2) < threshold:
    #     return 0.
    # else:
    # TODO check why this pi/2 is needed? The formation orientation is off by pi/2 it seems
    return np.arctan2(velocity[Y], velocity[X]) + pose #+ (np.pi/2.) 
 

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

    formation_pose = np.concatenate((leader_pose[0:2], [formation_orientation]))
    # Could we just substitute formation pose for leader pose? DONT DO THIS WITHOUT TESTING FOR ROBUSTNESS

    # Desired positions of each of the follower robots in the formation (see comment above about replacing formation pose with leader...)
    desired_positions = get_desired_positions(formation=FORMATION, formation_pose=formation_pose)

    # velocity directs the follower robots from their current position to their (desired position in the formation)
    follower_positions = np.array([pose[0:2] for pose in follower_poses])
    velocities = desired_positions - follower_positions

    # update each velocity (the displacement between the current and desired position) depending on the distance
    for i in range(len(velocities)):
        # euclidian distance
        distance = np.sqrt(np.square(velocities[i][X]) + np.square(velocities[i][Y]))

        print('rob_pos: %s, rob_despos: %s, form_orient: %s, in_ded_zone: %s' % (follower_poses[i], desired_positions[i], formation_orientation, (distance < DEAD_ZONE)))

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

    print("----------------------------------")
    print("----------------------------------")


    # return the follower velocities
    return velocities


    # # END OF NEW CODE
    # # THE CODE BELOW IS THE OLD CODE FROM THE UNIT-REFERENCE MODEL (JUST FOR OUR PURPOSES)
    # # make all robot yaws positive
    # for i in range(0, len(follower_poses)):
    #     current_poses[i][YAW] = (current_poses[i][YAW] + (2.*np.pi)) % (2.*np.pi)

    # # Unit reference
    # # Use average of all the robot position to work out a unit center
    # unit_reference = np.sum(current_poses, axis=0) / len(current_poses)
    # average_pose = unit_reference[YAW]  # Assuming all the robot yaws are positive when this is computed (from above for loop)
    # print("unit reference: ", unit_reference)


    # # The orientation of this formation is defined by a line from the unit center to the next navigational waypoint
    # # The unit center and formation orientation together define a local coordinate system around whih the new formation is based
    # average_update_velocity = np.sum(update_velocities, axis=0) / len(update_velocities)
    # print("avg rrt velocity: ", average_update_velocity)

    # # Formation orientation is the angle of average velocity 
    # formation_orientation = get_formation_orientation(average_update_velocity, average_pose)
    # formation_pose = np.concatenate((unit_reference[:2], [formation_orientation]))

    # # Desired positions of each of the robots in the formation
    # desired_positions = get_desired_positions(formation=FORMATION, formation_pose=formation_pose)

    # # velocity directs the robot from its current position to its desired position in the formation
    # current_positions = np.array([pose[0:2] for pose in current_poses])
    # velocities = desired_positions - current_positions

    # # update each velocity (the displacement between the current and desired position) depending on the distance
    # for i in range(len(velocities)):
    #     # euclidian distance
    #     distance = np.sqrt(np.square(velocities[i][X]) + np.square(velocities[i][Y]))

    #     print('rob_pos: %s, rob_despos: %s, form_orient: %s, in_ded_zone: %s' % (current_poses[i], desired_positions[i], formation_orientation, (distance < DEAD_ZONE)))

    #     # If a robot is within accepted radius of formation position, velocity should be 0
    #     # DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
    #     if distance < DEAD_ZONE:
    #         velocities[i] = np.zeros_like(velocities[i])
    #     elif distance < CONTROLLED_ZONE:
    #         # we do nothing, velocity is directly proportional to distance
    #         pass
    #     else:
    #         # robot is outside the controlled zone, so we normalize the velocity and then multiply
    #         # by the radius of the control zone.
    #         velocities[i] = velocities[i] / distance
    #         velocities[i] = velocities[i] * CONTROLLED_ZONE

    # print("----------------------------------")
    # print("----------------------------------")


    # return velocities
