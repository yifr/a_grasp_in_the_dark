import numpy as np


def normalize(v):
    return v / np.sum(v)


def update_target_bias(previous_contacts, initial_bias):
    """
    Update the estimate of the object location based on previous contacts.

    Args:
        previous_contacts (set): Set of 3D coordinates of the previous touches.
        initial_bias (np.array): Initial [mean_x, mean_y, std_x, std_y] for the target bias.

    Returns:
        np.array: Updated [mean_x, mean_y, std_x, std_y] for the target bias.
    """
    if not previous_contacts:
        return initial_bias

    # Convert previous contacts to a numpy array
    contacts = np.array(list(previous_contacts))[:, :2]  # Assuming contacts are 2D points

    # Calculate the new mean as the average of previous contacts
    new_mean = np.mean(contacts, axis=0)

    # Calculate the new standard deviation
    # This could be the standard deviation of the contacts, 
    # or a combination of initial std and contacts std
    new_std = np.std(contacts, axis=0)
    if initial_bias is not None:
        # Weighted average of the initial std and new std
        initial_std = np.array(initial_bias[2:])
        new_std = (new_std + initial_std) / 2

    # Combine new mean and std into the updated bias
    updated_bias = np.array([new_mean[0], new_mean[1], new_std[0], new_std[1]])

    return updated_bias


def sample_new_target(p_WG, p_WCs, target_bias=None, 
                    xmin=0.4, xmax=0.75, ymin=-0.35, ymax=0.35, 
                    grid_size=20,
                    search_radius=0.01,
                    object_r=0.01):
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
            object_r: float: radius of the object on the table
            motion_cost_radius: float: radius of the local neighborhood
        Returns:    
            new_target: np.array: 2D coordinates of the next target
    """
    # Create a grid over the table
    x = np.linspace(xmin, xmax, grid_size)
    y = np.linspace(ymin, ymax, grid_size)
    grid = np.array(np.meshgrid(x, y)).T.reshape(-1, 2)
    
    # Exclude points that are within object radius of previous contacts
    exclusion_zone = object_r + search_radius
    for contact in p_WCs:
        contact = contact[:2]  # Assuming contacts are 2D points
        distance = np.linalg.norm(grid - contact, axis=1)
        grid = grid[distance > exclusion_zone]

    # Apply target bias if available
    if target_bias is not None:
        mean_x, mean_y, std_x, std_y = target_bias
        weights = np.exp(-((grid[:, 0] - mean_x)**2 / (2 * std_x**2) + (grid[:, 1] - mean_y)**2 / (2 * std_y**2)))
    else:
        weights = np.ones(len(grid))

    # Prioritize points closer to the current gripper position
    distances_to_gripper = np.linalg.norm(grid - p_WG[:2], axis=1)
    weights /= distances_to_gripper

    # Normalize weights
    weights /= weights.sum()

    # Choose a new target point based on the weights
    new_target_idx = np.random.choice(len(grid), p=weights)
    new_target = grid[new_target_idx]

    return new_target, weights