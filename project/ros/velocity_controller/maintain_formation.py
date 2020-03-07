from init_formations import FORMATION, SPACING_DIST, INITIAL_YAW

import numpy as np

X = 0
Y= 1
YAW = 2

ROBOT_RADIUS = 0.105 / 2.

# DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
DEAD_ZONE = 1.5 * ROBOT_RADIUS
# CONTROL_ZONE (if robot is within the controlled zone, velocity towards position linearly increases the further away it is)
CONTROLLED_ZONE = DEAD_ZONE + SPACING_DIST

def get_desired_positions(formation, formation_pose):

    # Take formation and transform (rotate, translate) onto formation_pose
    theta = formation_pose[YAW]

    desired_positions = np.zeros_like(formation)
    for i in range(len(formation)):
        # Rotate
        desired_positions[i] = np.matmul([[np.cos(theta), -np.sin(theta)],
                                          [np.sin(theta),  np.cos(theta)]], formation[i])
        # Translate
        desired_positions[i] = formation_pose[:2] + desired_positions[i]
    return desired_positions


def maintain_formation(leader_pose, follower_poses, leader_rrt_velocity):

    # Formation orientation is the angle of the formation given the leader's direction.
    formation_orientation = leader_pose[YAW] - INITIAL_YAW
    formation_pose = np.concatenate((leader_pose[:2], [formation_orientation]))

    # Desired positions of each of the follower robots in the formation (see comment above about replacing formation pose with leader...)
    desired_positions = get_desired_positions(formation=FORMATION, formation_pose=formation_pose)

    # velocity directs the follower robots from their current position to their (desired position in the formation)
    follower_positions = np.array([pose[:2] for pose in follower_poses])
    velocities = desired_positions - follower_positions

    # update each velocity (the displacement between the current and desired position) depending on the distance
    for i in range(len(velocities)):

        distance = np.linalg.norm(velocities[i])

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

    return velocities, desired_positions
