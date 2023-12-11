import numpy as np

# def normalize(v):
#     return v / np.sum(v)


# def update_target_bias(previous_contacts, initial_bias):
#     """
#     Update the estimate of the object location based on previous contacts.

#     Args:
#         previous_contacts (set): Set of 3D coordinates of the previous touches.
#         initial_bias (np.array): Initial [mean_x, mean_y, std_x, std_y] for the target bias.

#     Returns:
#         np.array: Updated [mean_x, mean_y, std_x, std_y] for the target bias.
#     """
#     if not previous_contacts:
#         return initial_bias

#     # Convert previous contacts to a numpy array
#     contacts = np.array(list(previous_contacts))[:, :2]  # Assuming contacts are 2D points

#     # Calculate the new mean as the average of previous contacts
#     new_mean = np.mean(contacts, axis=0)

#     # Calculate the new standard deviation
#     # This could be the standard deviation of the contacts, 
#     # or a combination of initial std and contacts std
#     new_std = np.std(contacts, axis=0)
#     if initial_bias is not None:
#         # Weighted average of the initial std and new std
#         initial_std = np.array(initial_bias[2:])
#         new_std = (new_std + initial_std) / 2

#     # Combine new mean and std into the updated bias
#     updated_bias = np.array([new_mean[0], new_mean[1], new_std[0], new_std[1]])

#     return updated_bias


def sample_new_target(past_points, bias_position=None, 
                    angle_std = 0.1, dist_std = 0.1, reach_std = 0.1, object_radius=0.05):
    """
        Proposes a new location to reach with the hand.
        
        Args:
            p_WG: current translation of gripper in world frame
            p_WCs: set: set of 3D coordinates of the previous touches
            target_bias: np.array: [mean_x, mean_y, std_x, std_y] for the target bias
            xmin: float: minimum x coordinate of the table
            xmax: float: maximum x coordinate of the table
            ymin: float: minimum y coordinate of the table
            ymax: float: maximum y coordinate of the table
            grid_size: int: how finely to discretize the table
            object_area: float: size of the object on the table
        Returns:    
            new_target: np.array: 2D coordinates of the next target
    """
    # Create a grid over the table
    dist = np.linalg.norm(bias_position - past_points[-1])
    angle = np.arctan2(bias_position[1] - past_points[-1][1], bias_position[0] - past_points[-1][0])
    noisy_angle = np.random.normal(angle, angle_std)
    noisy_dist = np.random.normal(dist, dist_std)
    while noisy_dist < object_radius:
        noisy_dist = np.random.normal(dist, dist_std)
    N = 100
    pseudo_target = np.array([dist * 2 * np.cos(noisy_angle), dist * 2 * np.sin(noisy_angle)]) + past_points[-1]
    line_points = np.array([np.linspace(past_points[-1][0], pseudo_target[0], N), np.linspace(past_points[-1][1], pseudo_target[1], N)]).T
    noisy_target = np.array([noisy_dist * np.cos(noisy_angle), noisy_dist * np.sin(noisy_angle)]) + past_points[-1]
    probabilities = np.exp(-0.5 * ((line_points - noisy_target)**2 / reach_std**2).sum(axis=1))
    for i in range(N):
        for p in past_points:
            if np.linalg.norm(p - line_points[i]) < object_radius:
                probabilities[i] = 0
                break
    # print(probabilities)
    probabilities = probabilities / np.sum(probabilities)
    next_point = line_points[np.random.choice(N, p=probabilities)]
    # plot the heatmap
    # plt.scatter(line_points[:, 0], line_points[:, 1], c=probabilities, marker='o', s=10)
    # plt.scatter(line_points[:, 0], line_points[:, 1], c='lightgray', marker='o', s=10)
    return next_point

