import numpy as np


def normalize(v):
    return v / np.sum(v)

def sample_new_target(prev_touch, p_grid, target_bias=None, 
                    xmin=0.3, xmax=0.6, ymin=-0.3, ymax=0.3, grid_size=15,
                    object_r=0.2):
    """
        Proposes a new location to reach with the hand.
        
        Args:
            prev_touch: np.array: 2D coordinates of the previous touch
            p_grid: dict: dictionary of probabilities for each location on the table
            target_bias: np.array: [mean_x, mean_y, std_x, std_y] for the target bias
            xmin: float: minimum x coordinate of the table
            xmax: float: maximum x coordinate of the table
            ymin: float: minimum y coordinate of the table
            ymax: float: maximum y coordinate of the table
            grid_size: int: how finely to discretize the table
            object_r: float: radius of the object on the table
        Returns:    
            next_touch: np.array: 2D coordinates of the next touch
            p_grid: dict: updated dictionary of probabilities for each location on the table
    """
    effort_ratio = 0.7  # ratio of effort to probability
    effort_c = 2.0  # effort cost coefficient
    p_target_c = 2.0

    table_x_bounds = [xmin, xmax]
    table_y_bounds = [ymin, ymax]

    if len(list(p_grid.keys())) == 0:
        p_grid = {}
        for x in np.linspace(table_x_bounds[0], table_x_bounds[1], grid_size):
            for y in np.linspace(table_y_bounds[0], table_y_bounds[1], grid_size):
                p_grid[(x, y)] = 0.5

    all_p = []
    all_effort = []
    all_dist = []
    table_grid = {}
    for coord, prob in p_grid.items():
        dist = np.linalg.norm(np.array(coord) - prev_touch)
        if dist < object_r:
            p_target = 0.0
        else:
            c_dist = dist - object_r
            p_target = 1 - np.exp(-p_target_c * c_dist)
        new_p_target = prob * p_target
        if new_p_target == 0.0:
            effort_cost = 0.0
        else:
            effort_cost = np.exp(-effort_c * c_dist)
        p_grid[coord] = new_p_target
        all_effort.append(effort_cost)
        all_p.append(new_p_target)
        all_dist.append(dist)
    all_effort_norm = normalize(all_effort)
    all_p_norm = normalize(all_p)
    for i, coord in enumerate(p_grid.keys()):
        table_grid[coord] = all_p_norm[i] * (1 - effort_ratio) + all_effort_norm[i] * effort_ratio
    table_grid_norm = normalize(list(table_grid.values()))
    next_touch_idx = np.random.choice(np.arange(len(list(table_grid.keys()))), p=table_grid_norm)
    next_touch = list(p_grid.keys())[next_touch_idx]
    
    return next_touch, p_grid
