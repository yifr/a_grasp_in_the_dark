import numpy as np
import copy
from pydrake.all import (
    RigidTransform
)

def get_table_contact(X_WG, final_height=-0.05, num_knot_points=15):
    """
    Moves the arm down until it comes in contact with the table
    """
    R_WG = X_WG.rotation()
    p_WG_pre = X_WG.translation()
    p_WG_post = copy.deepcopy(p_WG_pre)
    p_WG_post[2] = final_height

    t_list = np.arange(num_knot_points)
    positions = np.zeros((num_knot_points, 3))
    for i in range(3):
        positions[:, i] = np.linspace(
            p_WG_pre[i], p_WG_post[i], num_knot_points
        )

    p_list = []
    for p_WG in positions:
        p_list.append(RigidTransform(R_WG, p_WG))

    return p_list


def get_contacts(station, context):
    """
    Get contact points from the station
    Contact points are in World frame
    """
    contacts = []
    contact = station.GetOutputPort("contact_results").Eval(context)
    num_contacts = contact.num_point_pair_contacts()
    for i in range(num_contacts):
        p_WC = contact.point_pair_contact_info(i).contact_point()
        contacts.append(p_WC)

    return np.array(contacts)


def evaluate_contact(p_WC, threshold=0.0001):
    """
    Determine if contact is likely an object or the table
    """
    # find the min difference between z values
    z_diff = np.max(p_WC[:, 2])
    # print(z_diff)
    if z_diff < threshold:
        object_touched = "table"
    else:
        object_touched = "object"
        
    return object_touched