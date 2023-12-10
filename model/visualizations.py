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
    X, Y = np.meshgrid(y, x)
    grid = np.array([X.flatten(), Y.flatten()]).T

    # Calculate weights for each grid point
    exclusion_zone = object_r + search_radius
    weights = np.ones(grid.shape[0])
    for contact in p_WCs:
        distance = np.linalg.norm(grid - contact[:2], axis=1)
        weights[distance < exclusion_zone] = 0  # Exclude points too close to previous contacts

    if target_bias is not None:
        mean_y, mean_x, std_y, std_x = target_bias
        weights *= np.exp(-((grid[:, 0] - mean_x)**2 / (2 * std_x**2) + (grid[:, 1] - mean_y)**2 / (2 * std_y**2)))

    distances_to_gripper = np.linalg.norm(grid - p_WG[:2], axis=1)
    weights /= distances_to_gripper
    weights /= weights.sum()

    # Visualization
    plt.figure(figsize=(10, 6))
    plt.scatter(X, Y, c=weights, cmap='hot', alpha=0.5)  # Colormap for probabilities
    plt.colorbar(label='Probability')

    for p_WC in p_WCs:
        plt.scatter(p_WC[1], p_WC[0], marker='x', color='red', label="Touched Points", s=100)

    plt.scatter(p_WG[1], p_WG[0], marker='o', color='blue', label='Current Gripper Position')
    plt.scatter(new_target[1], new_target[0], marker='*', color='green', label='New Target', s=150)
    plt.scatter(p_W_brick[1], p_W_brick[0], marker='s', color='gold', label='Brick', s=200)

    # Zip legend
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc=(1.25, 0.5))

    plt.xlim(ymin, ymax)
    plt.ylim(xmin, xmax)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Search Algorithm Visualization')
    plt.grid(True)
    plt.show()