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
                    xmin=0.45, xmax=0.75, ymin=-0.3, ymax=0.3, 
                    grid_size=20,
                    object_area=0.05):
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
    x_grid = np.linspace(xmin, xmax, grid_size)
    y_grid = np.linspace(ymin, ymax, grid_size)
    xx, yy = np.meshgrid(x_grid, y_grid)
    grid_points = np.vstack([xx.ravel(), yy.ravel()]).T

    # Apply Gaussian distribution if target_bias is given
    if target_bias is not None:
        mean = target_bias[:2]
        std = target_bias[2:]
        probabilities = np.exp(-0.5 * ((grid_points - mean)**2 / std**2).sum(axis=1))
        probabilities /= probabilities.sum()
    else:
        probabilities = np.ones(grid_size**2) / (grid_size**2)

    # Exclude previously visited coordinates
    for p_WC in p_WCs:
        distances = np.sqrt(((grid_points - p_WC[:2])**2).sum(axis=1))
        probabilities[distances < object_area] = 0

    # Normalize the probabilities after exclusion
    probabilities /= probabilities.sum()

    # Sample new target based on the probabilities
    new_target_index = np.random.choice(grid_size**2, p=probabilities)
    new_target = grid_points[new_target_index] * [1, -1]

    return new_target

