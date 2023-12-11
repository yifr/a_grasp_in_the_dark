import os
import numpy as np
from pydrake.all import (
    LeafSystem,
    Context,
    Meshcat,
    State,
    ContactResults,
    RigidTransform,
    RotationMatrix,
    AbstractValue,
    PointCloud,
    PiecewisePose,
    Solve,
    TrajectorySource,
    PiecewisePolynomial,
)

from scipy import signal
from enum import Enum
from pydrake.multibody import inverse_kinematics
from manipulation.meshcat_utils import AddMeshcatTriad
import copy


class PlannerState(Enum):
    """PlannerState is an enum that represents the state of the planner.
    """
    FIND_TABLE = 0
    SEARCHING = 1
    INVESTIGATE = 2
    GRASPING = 3
    GO_HOME = 4

class Planner(LeafSystem):
    """Planner is responsible for maintaining state and planning actions.
    """
    def __init__(
        self,
        plant,
        p_Wgoal_prior=None,
        R_Wgoal_prior=None,
        frame = "iiwa_link_6"
    ):
        super().__init__()
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._gripper_body_index = plant.GetBodyByName(frame).index()

        self._p_Wbrick_prior = p_Wgoal_prior
        self._R_Wbrick_prior = R_Wgoal_prior
        self._frame_name = frame

        #####################
        # # inputs from other systems
        # input[0] = body_poses
        self._body_poses_index = self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform()])).get_index()
        # input[1] = state_estimated
        self._estimated_state_index = self.DeclareVectorInputPort("iiwa+allegro.state_estimated", 46).get_index()
        # input[2] = contact_results
        # self._contact_results_index = self.DeclareAbstractInputPort("contact_results", AbstractValue.Make(ContactResults())).get_index()

        # # Planner state
        self._mode_index = self.DeclareAbstractState(AbstractValue.Make(PlannerState.FIND_TABLE))
        self._traj_X_G_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePose())
        )

        # # outputs
        self.DeclareAbstractOutputPort(
            "X_WG", 
            lambda: AbstractValue.Make(RigidTransform()), 
            self.CalcRobotPose
        )

        num_positions = 46
        self._q0_index = self.DeclareDiscreteState(num_positions)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)
        self._estimated_table_height = 0.0

    def Initialize(self, context, discrete_state):
        discrete_state.set_value(
            int(self._q0_index),
            self.get_input_port(int(self._estimated_state_index)).Eval(context),
        )


    def Find_table(self, context, state):
        """
        Initialize should make contact with table, and set the initial state of the planner.
        """
        # body_pose = self.get_input_port(self._body_poses_index).Eval(context)
        # gripper = self._plant.GetBodyByName(self._frame)
        # X_WG_pre = self._plant.EvalBodyPoseInWorld(self._plant_context, gripper)
        # p_WG_post = copy.deepcopy(X_WG_pre.translation())
        # p_WG_post[2] = self._estimated_table_height
        # X_WG_post = RigidTransform(X_WG_pre.rotation(), p_WG_post)
        # print(X_WG_post)
        # state.get_mutable_abstract_state(
        #     int(self._mode_index)
        # ).set_value(PlannerState.FIND_TABLE)
        # state.get_mutable_abstract_state(
        #     int(self._traj_X_G_index)
        # ).set_value(X_WG_post)


    def Update(self, context, state):
        """
        Update should update the state of the planner.
        """
        if self.EvaluateContactResults(context, state) == "table":
            mode = PlannerState.SEARCHING
        elif self.EvaluateContactResults(context, state) == "object":
            mode = PlannerState.INVESTIGATE 
        else:
            mode = context.get_abstract_state(int(self._mode_index)).get_value() 

        state.get_mutable_abstract_state(
            int(self._mode_index)
        ).set_value(mode)   

        if mode == PlannerState.FIND_TABLE:
            self.Find_table(context, state)
        elif mode == PlannerState.SEARCHING:
            self.Search(context, state)
        elif mode == PlannerState.INVESTIGATE:
            self.Investigate(context, state)
        elif mode == PlannerState.GRASPING:
            self.Pick(context, state)
        elif mode == PlannerState.GO_HOME:
            self.GoHome(context, state)
        else:
            raise RuntimeError("Invalid mode: {}".format(mode))
        

    def Search(self, context, output):
        pass


    def Investigate(self, context, output):
        pass


    def Pick(self, context, output):
        pass


    def GoHome(self, context, output):
        pass


    def CalcRobotPose(self, context, state):
        context.get_abstract_state(int(self._mode_index)).get_value()

        traj_X_G = context.get_abstract_state(
            int(self._traj_X_G_index)
        ).get_value()
        if traj_X_G.get_number_of_segments() > 0 and traj_X_G.is_time_in_range(
            context.get_time()
        ):
            # Evaluate the trajectory at the current time, and write it to the
            # output port.
            state.set_value(
                context.get_abstract_state(int(self._traj_X_G_index))
                .get_value()
                .GetPose(context.get_time())
            )
            return



    def EvaluateContactResults(self, context, output):
        """
        Get current contact results, filter out self collisions,
        update the point cloud, and return desired mode
        """
        # contact_results = self.get_input_port(int(self._contact_results_index)).Eval(context)
        return False
    

class Differential_IK(LeafSystem):
    def __init__(
            self,
            plant,
            frame = "iiwa_link_6"
    ):
        super().__init__()
        self._plant = plant
        self._frame = frame
        self._plant_context = plant.CreateDefaultContext()
        self._gripper_body_index = plant.GetBodyByName(frame).index()
        self.interp_steps = 16
        self.arc_height = 0.25

        #####################
        # # inputs from other systems
        # input[0] = body_poses
        self._X_WG_post_index = self.DeclareAbstractInputPort("X_WG_future", AbstractValue.Make(RigidTransform())).get_index()
        # input[1] = state_estimated
        self._body_poses_index = self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform()])).get_index()
        # input[2] = estimated_state
        self._estimated_state_index = self.DeclareVectorInputPort("iiwa+allegro.state_estimated", 46).get_index()
        self._desired_state_index = self.DeclareVectorOutputPort("iiwa+allegro.desired_state", 46, self.Calc_Traj).get_index()
        # input[2] = desired_state
        # self.desired_state_index = self.DeclareVectorInputPort("iiwa+allegro.desired_state", 46).get_index()

        # # outputs
        # self.DeclareAbstractOutputPort(
        #     "Trajectory", 
        #     lambda: AbstractValue.Make(TrajectorySource(self._desired_state_index)), 
        #     self.Calc_Traj
        # )

    def interpolate_location(self, context):
        body_poses = self.get_input_port(int(self._body_poses_index)).Eval(context)
        X_WG_pre = body_poses[self._gripper_body_index]
        gripper = self._plant.GetBodyByName(self._frame)
        X_WG_pre = self._plant.EvalBodyPoseInWorld(self._plant_context, gripper)
        p_WG_post = copy.deepcopy(X_WG_pre.translation())
        p_WG_post[2] = 0.0
        X_WG_post = RigidTransform(X_WG_pre.rotation(), p_WG_post)        
        p_WG_pre = X_WG_pre.translation()
        p_WG_post = X_WG_post.translation()
        R_WG_pre = X_WG_pre.rotation()
        R_WG_post = X_WG_post.rotation()
        t_list = np.arange(self.interp_steps)
        positions = np.zeros((self.interp_steps, 3))
        for i in range(3):
            positions[:, i] = np.linspace(
                p_WG_pre[i], p_WG_post[i], self.interp_steps
            )

        p_list = []
        for p_WG in positions:
            p_list.append(RigidTransform(R_WG_pre, p_WG))
        return p_list
        # xs = np.linspace(p_WG_pre[0], p_WG_post[0], self.interp_steps)[1:]
        # ys = np.linspace(p_WG_pre[1], p_WG_post[1], self.interp_steps)[1:]

        # # Check for same z-values and adjust if needed
        # if abs(p_WG_pre[2] - p_WG_post[2]) < 0.1:
        #     # If z-values are practically the same, use a half-circle arc
        #     z_center = (p_WG_pre[2] + p_WG_post[2]) / 2
        #     z0 = np.linspace(p_WG_pre[2], z_center + self.arc_height, self.interp_steps // 2)
        #     z1 = np.linspace(z_center + self.arc_height, p_WG_post[2], self.interp_steps // 2)
        #     zs = np.concatenate([z0, z1])[1:]
        # else:
        #     # Otherwise, use linear interpolation
        #     zs = np.linspace(p_WG_pre[2], p_WG_post[2], self.interp_steps)[1:]

        # # Combine interpolated coordinates
        # interp_coords = np.vstack((xs, ys, zs)).T


        # # Generate list of RigidTransform objects
        # pose_list = []
        # for i in range(len(interp_coords)):
        #     pose = RigidTransform(R_WG_pre, interp_coords[i])
        #     pose_list.append(pose)

        # return pose_list

    def optimize_arm_movement(self, context, state):
        pose_list = self.interpolate_location(context)
        # print(pose_list)
        arm_trajectory = []
        world_frame = self._plant.world_frame()
        gripper_frame = self._plant.GetFrameByName(self._frame)
        estimated_state = self.get_input_port(int(self._estimated_state_index)).Eval(context)
        iiwa_initial = estimated_state[:7]
        # print(estimated_state)
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

        end_effector_poses = pose_list
        for i in range(len(end_effector_poses)):
            ik = inverse_kinematics.InverseKinematics(self._plant)
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
                prog.SetInitialGuess(q_variables, arm_trajectory[i-1])

            result_found = False
            for i in range(100):
                result = Solve(prog)
                if result.is_success():
                    result_found = True
                    break

            if not result_found:
                raise RuntimeError("Inverse kinematics failed.")

            arm_trajectory.append(result.GetSolution(q_variables))

        q_knots = np.zeros((len(arm_trajectory), 46))
        arm_trajectory = np.array(arm_trajectory)
        q_knots[:, :arm_trajectory.shape[1]] = arm_trajectory
        return q_knots
        #q_traj = PiecewisePolynomial.CubicShapePreserving(range(len(q_knots)), q_knots.T)
        #self.q_traj_system = TrajectorySource(q_traj)

    def Calc_Traj(self, context, state):
        q_knots = self.optimize_arm_movement(context, state)
        state.set_value(
            q_knots[-1]
        )