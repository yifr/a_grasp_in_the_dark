import numpy as np
import matplotlib.pyplot as plt

def plot_2d_search(all_touched_points, x, y, probs):
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
    fig = plt.figure(figsize=(10, 10))
    plt.scatter(x, y, c=probs)
    plt.colorbar()
    # show the touched point with a star
    for i, t in enumerate(all_touched_points):
        if i < len(all_touched_points) - 1:
            plt.scatter(t[0], t[1], marker="^", c="orange", label="previous", linewidths=5)
        else:
            plt.scatter(t[0], t[1], marker="*", c="red", label="current", linewidths=5)

    # legend the two star points
    # Zip legend values:
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    return fig