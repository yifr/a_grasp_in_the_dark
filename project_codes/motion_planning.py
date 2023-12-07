import numpy as np
import copy
from pydrake.all import (
    RigidTransform,
    Solve,
)
from pydrake.multibody import inverse_kinematics
import contact


def interpolate_locations(current_location, new_location, X_robot_W, interp_steps=16, arc_height=0.5):
    # NOTE: Convert positions to RigidTransforms
    """
    Linear interpolation between two 3D coordinates
    """
    coords = np.hstack([current_location, new_location])
    xs = np.linspace(current_location[0], new_location[0], interp_steps)
    ys = np.linspace(current_location[1], new_location[1], interp_steps)

    # If z-values are the same, move the arm up to avoid dragging along 
    if current_location[2] == new_location[2]:
        z0 = np.linspace(current_location[2], current_location[2] + arc_height, interp_steps // 2)
        z1 = np.linspace(current_location[2] + arc_height, new_location[2], interp_steps // 2)
        zs = np.concatenate([z0, z1])

    else:
        zs = np.linspace(current_location[2], new_location[2], interp_steps)
    
    interp_coords = np.vstack((xs, ys, zs)).T
    new_coords = np.vstack([interp_coords, new_location])

    pose_list = []
    for i in range(len(new_coords)):
        pose = RigidTransform(X_robot_W.rotation(), new_coords[i])
        pose_list.append(pose)
        
    return pose_list


def optimize_arm_movement(robot_state, station, end_effector_poses, frame="iiwa_link_6"):
    """Convert end-effector pose list to joint position list using series of
    InverseKinematics problems. Note that q is 9-dimensional because the last 2 dimensions
    contain gripper joints, but these should not matter to the constraints.

    @param: robot_state (numpy array): robot_state[i] contains joint position q at index i.
    @param: station (pydrake.systems.framework.DiagramBuilder): DiagramBuilder containing the robot model.
    @param: end_effector_poses (python_list): end_effector_poses[i] contains the desired pose of the end-effector at index i.
    @param: frame (string): name of the frame of the end-effector.

    @return: q_knots (python_list): q_knots[i] contains IK solution that will give f(q_knots[i]) \approx pose_lst[i].
    """
    q_knots = []
    plant = station.GetSubsystemByName("plant")
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName(frame)

    iiwa_initial = robot_state[:7]
    # gripper_initial = robot_state[7:30]
    # iiwa_initial = np.array([0.0, 0.1, 0.0, -1.5, 0.0, 0.0, 0.0])
    gripper_initial = np.ones((23))
    q_nominal = np.concatenate((iiwa_initial, gripper_initial))

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

    for i in range(len(end_effector_poses)):
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()  # Get variables for MathematicalProgram
        prog = ik.prog()  # Get MathematicalProgram

        pose = end_effector_poses[i]
        AddPositionConstraint(
                    ik,
                    pose.translation(),
                    pose.translation(),
        )
    
        prog.AddQuadraticErrorCost(np.identity(len(q_variables)), q_nominal, q_variables)
        if i == 0:
            prog.SetInitialGuess(q_variables, q_nominal)
        else:
            prog.SetInitialGuess(q_variables, q_knots[i-1])

        result = Solve(prog)

        assert result.is_success()
        q_knots.append(result.GetSolution(q_variables))

    return np.array(q_knots)
    


def move_arm(simulator, station, context, X_robot_W, end_effector_poses, time_interval=0.4, frame="iiwa_link_6"):
    """
    Move the arm to a new location. If the arm is in contact with an object, stop moving.
    @param: robot_state (numpy array): Allegro wrapper with current robot state.
    @param: simulator (pydrake.systems.analysis.Simulator): Simulator object.
    @param: station (pydrake.systems.framework.DiagramBuilder): DiagramBuilder containing the robot model.
    @param: context (pydrake.systems.framework.Context): Context object.
    @param: end_effector_poses (python_list): end_effector_poses[i] contains the desired pose of the end-effector at index i.
    @param: time_interval (float): time interval for each step of the simulation.
    @param: frame (string): name of the frame of the end-effector.

    @return: The touched object and current contact params
    """    
    robot_state = station.GetOutputPort("iiwa+allegro.state_estimated").Eval(context)
    trajectory = optimize_arm_movement(robot_state, station, end_effector_poses, frame=frame)
    arm_trajectory = trajectory[:, :7]
    for state_update in arm_trajectory:
        new_state = station.GetInputPort("iiwa+allegro.desired_state").Eval(context)
        new_state[:len(state_update)] = state_update
        station.GetInputPort("iiwa+allegro.desired_state").FixValue(context, new_state)
        simulator_steps = 500
        for i in range(simulator_steps):
            simulator.AdvanceTo(context.get_time() + time_interval / simulator_steps)
            # Check for contact
            current_contact = contact.get_contacts(station, context, X_robot_W)
            if len(current_contact) > 0:
                obj_touched = contact.evaluate_contact(current_contact)
                break
            else:
                obj_touched = False

    return obj_touched, current_contact