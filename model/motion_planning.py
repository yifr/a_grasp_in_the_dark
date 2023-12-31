import numpy as np
import copy
from pydrake.all import (
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    Solve,
)
from pydrake.trajectories import PiecewisePolynomial
from pydrake.multibody import inverse_kinematics
from scipy.interpolate import CubicSpline
import contact
from AllegroWrapper import STATION_MAP, LINK_IDS

def interpolate_locations(X_WG, p_WG_post, interp_steps=32, arc_height=0.25, rotate=None):
    """
    Linear interpolation between two 3D coordinates with adjustments for smoother motion

    Args:
        X_WG: RigidTransform of the robot before the move
        p_WG_post: 3D coordinate of the robot after the move
        interp_steps: number of interpolation steps
        arc_height: height of the arc to move the robot through

    Returns:
        coords: RigidTransform of the robot at each interpolation step
    """
    p_WG_pre = X_WG.translation()

    # Z-axis arc movement        
    if np.linalg.norm(p_WG_pre[2] - p_WG_post[2]) < 0.1:
        # Time vector
        t = np.linspace(0, 1, interp_steps)

        # Interpolate X and Y linearly
        x_spline = CubicSpline([0, 1], [p_WG_pre[0], p_WG_post[0]])
        y_spline = CubicSpline([0, 1], [p_WG_pre[1], p_WG_post[1]])
        z_spline = CubicSpline([0, 0.5, 1], [p_WG_pre[2], arc_height, p_WG_post[2]])

        # Apply splines to time vector
        xs = x_spline(t)
        ys = y_spline(t)
        zs = z_spline(t)
    else:
        xs = np.linspace(p_WG_pre[0], p_WG_post[0], interp_steps)
        ys = np.linspace(p_WG_pre[1], p_WG_post[1], interp_steps)
        zs = np.linspace(p_WG_pre[2], p_WG_post[2], interp_steps)
    
    # Generate list of RigidTransform objects
    pose_list = []
    for i in range(interp_steps):
        if rotate is None:
            rotation = X_WG.rotation()
        else:
            # Adjust rotation as needed
            rotation = RotationMatrix().MakeXRotation(np.pi / 2).MakeYRotation(np.pi / 2)
        
        pose = RigidTransform(rotation, np.array([xs[i], ys[i], zs[i]]))
        pose_list.append(pose)

    return pose_list

def add_finger_constraints(ik, station, context):
    plant = station.GetSubsystemByName("plant")
    plant_context = plant.GetMyContextFromRoot(context)

    gripper_frame = plant.GetFrameByName("hand_root")
    gripper_orientation = plant.EvalBodyPoseInWorld(plant_context,
                                plant.GetBodyByName("hand_root")).rotation()
    link_ids = LINK_IDS.values()
    for link_id in link_ids:
        ik.AddOrientationConstraint(
            frameAbar=gripper_frame,
            R_AbarA=gripper_orientation,
            frameBbar=plant.GetFrameByName(link_id),
            R_BbarB=RotationMatrix(),
            theta_bound=np.pi/2,
        )

def optimize_arm_movement(q_current, station, context, end_effector_poses, frame="iiwa_link_6",
                        freeze_fingers=False, position_tol=0.01):
    """Convert end-effector pose list to joint position list using series of
    InverseKinematics problems. Note that q is 9-dimensional because the last 2 dimensions
    contain gripper joints, but these should not matter to the constraints.

    @param: X_WG (numpy array): X_WG[i] contains joint position q at index i.
    @param: station (pydrake.systems.framework.DiagramBuilder): DiagramBuilder containing the robot model.
    @param: end_effector_poses (python_list): end_effector_poses[i] contains the desired pose of the end-effector at index i.
    @param: frame (string): name of the frame of the end-effector.

    @return: q_knots (python_list): q_knots[i] contains IK solution that will give f(q_knots[i]) \approx pose_lst[i].
    """
    q_knots = []
    plant = station.GetSubsystemByName("plant")
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName(frame)
    
    iiwa_initial = q_current[:7]
    gripper_initial = np.ones((23))
    q_nominal = q_current[:30]  # np.concatenate((iiwa_initial, gripper_initial))
    
    def AddPositionConstraint(ik, p_WG_lower, p_WG_upper):
        """Add position constraint to the ik problem. Implements an inequality
        constraint where f_p(q) must lie between p_WG_lower and p_WG_upper. Can be
        translated to
        ik.prog().AddBoundingBoxConstraint(f_p(q), p_WG_lower, p_WG_upper)
        """
        ik.AddPositionConstraint(
            frameA=world_frame,
            frameB=gripper_frame,
            p_BQ=np.zeros(3),
            p_AQ_lower=p_WG_lower,
            p_AQ_upper=p_WG_upper,
        )

    def AddOrientationConstraint(ik, R_WG, bounds):
        """Add orientation constraint to the ik problem. Implements an inequality
        constraint where the axis-angle difference between f_R(q) and R_WG must be
        within bounds. Can be translated to:
        ik.prog().AddBoundingBoxConstraint(angle_diff(f_R(q), R_WG), -bounds, bounds)
        """
        ik.AddOrientationConstraint(
            frameAbar=world_frame,
            R_AbarA=R_WG,
            frameBbar=gripper_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=bounds,
        )

    for i in range(len(end_effector_poses)):
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()  # Get variables for MathematicalProgram
        prog = ik.prog()  # Get MathematicalProgram

        pose = end_effector_poses[i]
        AddPositionConstraint(
                    ik,
                    pose.translation() - position_tol,
                    pose.translation() + position_tol
        )
        
        if freeze_fingers:
            add_finger_constraints(ik, station, context)

        if frame == "hand_root" and not freeze_fingers:
            AddOrientationConstraint(ik, pose.rotation(), 0.2)

        prog.AddQuadraticErrorCost(np.identity(len(q_variables)), q_nominal, q_variables)

        result_found = False
        for j in range(100):
            if i == 0:
                prog.SetInitialGuess(q_variables, q_nominal)
            else:
                prog.SetInitialGuess(q_variables, q_knots[i-1])

            result = Solve(prog)
            if result.is_success():
                result_found = True
                break

        if not result_found:
            raise RuntimeError("Inverse kinematics failed.")

        q_knots.append(result.GetSolution(q_variables))
    
    return np.array(q_knots)


def move_arm(p_WG_post, simulator, station, context, time_interval=0.5, frame="iiwa_link_6", 
                arc_height=0.25,
                stop_on_contact=False,
                rotate=None,
                poses=None,
                state_update_len=7,
                freeze_fingers=False,
                position_tol=0.01,
                simulator_steps=25):
    """
    Move the arm to a new location. If the arm is in contact with an object, stop moving.
    @param: X_WG (numpy array): Allegro wrapper with current robot state.
    @param: simulator (pydrake.systems.analysis.Simulator): Simulator object.
    @param: station (pydrake.systems.framework.DiagramBuilder): DiagramBuilder containing the robot model.
    @param: context (pydrake.systems.framework.Context): Context object.
    @param: end_effector_poses (python_list): end_effector_poses[i] contains the desired pose of the end-effector at index i.
    @param: time_interval (float): time interval for each step of the simulation.
    @param: frame (string): name of the frame of the end-effector.

    @return: The touched object and current contact params
    """    
    plant = station.GetSubsystemByName("plant")
    plant_context = plant.GetMyContextFromRoot(context)
    gripper = plant.GetBodyByName(frame)
    X_WG = plant.EvalBodyPoseInWorld(plant_context, gripper)

    if frame != "iiwa_link_6":
        rotate = True
    else:
        rotate = False
    if poses is None:
        end_effector_poses = interpolate_locations(X_WG, p_WG_post, interp_steps=16, arc_height=arc_height, rotate=rotate)
    else:
        end_effector_poses = poses
    q_current = station.GetOutputPort("iiwa+allegro.state_estimated").Eval(context)
    trajectory = optimize_arm_movement(q_current, station, context, end_effector_poses, frame=frame, 
                                    freeze_fingers=freeze_fingers,
                                    position_tol=position_tol)

    arm_trajectory = trajectory[:, :state_update_len]
    all_contacts = set()
    for state_update in arm_trajectory:
        new_state = station.GetOutputPort("iiwa+allegro.state_estimated").Eval(context)
        new_state[:len(state_update)] = state_update
        station.GetInputPort("iiwa+allegro.desired_state").FixValue(context, new_state)
        for i in range(simulator_steps):
            simulator.AdvanceTo(context.get_time() + time_interval / simulator_steps)
            # Check for contact
            current_contact = contact.get_contacts(station, context)
            if len(current_contact) > 0:
                for p_WC in current_contact:
                    all_contacts.add(tuple(p_WC))
                
                if i > simulator_steps // 10:
                    obj_touched, p_W_objcontact = contact.evaluate_contact(all_contacts)
                    if obj_touched == "object" and stop_on_contact:
                        return obj_touched, current_contact, p_W_objcontact

    if len(all_contacts) > 0:
        obj_touched, p_W_objcontact = contact.evaluate_contact(all_contacts)
        return obj_touched, current_contact, p_W_objcontact
    else:
        return None, all_contacts, None

