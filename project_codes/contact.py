import numpy as np
import copy
from pydrake.all import (
    RigidTransform
)

def get_table_contact(X_robot_W, final_height = -0.2, num_knot_points=15):
    """
    Moves the arm down until it comes in contact with the table
    """
    final_rotation = X_robot_W.rotation()
    initial_position = X_robot_W.translation()
    final_position = copy.deepcopy(initial_position)
    final_position[2] = final_height

    t_list = np.arange(num_knot_points)
    positions = np.zeros((num_knot_points, 3))
    for i in range(3):
        positions[:, i] = np.linspace(
            initial_position[i], final_position[i], num_knot_points
        )
    p_list = []
    for p in positions:
        p_list.append(RigidTransform(final_rotation, p))

    return p_list


def get_contacts(station, context, X_robot_W):
    # simulator = Simulator(station)
    # simulator.AdvanceTo(0.1)
    robot_position = X_robot_W.translation()
    contacts = []
    contact = station.GetOutputPort("contact_results").Eval(context)
    num_contacts = contact.num_point_pair_contacts()
    for i in range(num_contacts):
        contacts.append(contact.point_pair_contact_info(i).contact_point() + robot_position)
    return np.array(contacts)


def evaluate_contact(current_contact, threshold=0.0001):
    """
    Determine if contact is likely an object or the table
    """
    # find the min difference between z values
    z_diff = np.max(current_contact[:, 2])
    # print(z_diff)
    if z_diff < threshold:
        touch = "table"
    else:
        touch = "object"
    return touch