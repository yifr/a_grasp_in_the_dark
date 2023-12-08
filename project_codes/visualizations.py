import numpy as np
import matplotlib.pyplot as plt

def plot_2d_search(p_WG, p_WCs, new_target, target_bias, p_W_brick,
                    grid_size=20, xmin=0.4, xmax=0.75, ymin=-0.35, ymax=0.35,
                    search_radius=0.01, object_r=0.01):
    """
    Plots the 2D search space with the touched points

    Args:
        all_touched_points (list): list of all touched points
        x (list): x values of the search space
        y (list): y values of the search space
        probs (list): Probability of each point in the search space
    
    Returns:
        Plots the 2D search space with the touched points, returns nothing
    """
    # Generate the grid
    x = np.linspace(xmin, xmax, grid_size)
    y = np.linspace(ymin, ymax, grid_size)
    X, Y = np.meshgrid(x, y)
    grid = np.array([X.flatten(), Y.flatten()]).T

    # Calculate weights for each grid point
    exclusion_zone = object_r + search_radius
    weights = np.ones(grid.shape[0])
    for contact in p_WCs:
        distance = np.linalg.norm(grid - contact[:2], axis=1)
        weights[distance < exclusion_zone] = 0  # Exclude points too close to previous contacts

    if target_bias is not None:
        mean_x, mean_y, std_x, std_y = target_bias
        weights *= np.exp(-((grid[:, 0] - mean_x)**2 / (2 * std_x**2) + (grid[:, 1] - mean_y)**2 / (2 * std_y**2)))

    distances_to_gripper = np.linalg.norm(grid - p_WG[:2], axis=1)
    weights /= distances_to_gripper
    weights /= weights.sum()

    # Visualization
    plt.figure(figsize=(10, 6))
    plt.scatter(X, Y, c=weights, cmap='hot', alpha=0.5)  # Colormap for probabilities
    plt.colorbar(label='Probability')

    # Plot previous contacts and current gripper position
    plt.scatter(*zip(*p_WCs), marker='x', color='red', label='Previous Contacts')
    plt.scatter(p_WG[0], p_WG[1], marker='o', color='blue', label='Current Gripper Position')
    plt.scatter(new_target[0], new_target[1], marker='*', color='green', label='New Target', s=150)
    plt.scatter(p_W_brick[0], p_W_brick[1], marker='s', color='gold', label='Brick', s=200)

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Search Algorithm Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()