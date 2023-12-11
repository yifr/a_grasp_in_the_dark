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
    PiecewisePose
)

from enum import Enum
from manipulation.meshcat_utils import AddMeshcatTriad

class PlannerState(Enum):
    """PlannerState is an enum that represents the state of the planner.
    """
    INITIAL = 0
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
    ):
        super().__init__()
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._gripper_body_index = plant.GetBodyByName("hand_root").index()

        self._p_Wbrick_prior = p_Wgoal_prior
        self._R_Wbrick_prior = R_Wgoal_prior
        
        # Input port for contact results
        self._point_cloud_index = self.DeclareAbstractInputPort(
            "point_cloud", 
            AbstractValue.Make(PointCloud(0))        
        ).get_index()

        self._contact_results_index = self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults(),
            self.EvaluateContactResults)
        ).get_index()

        self._mode_index = self.DeclareDiscreteState(PlannerState.INITIAL).get_index()
        self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform()]))
        self._estimated_state_index = self.DeclareAbstractInputPort("iiwa+allegro.state_estimated", AbstractValue.Make([0.0]*30))
        self._desired_state_index = self.DeclareAbstractInputPort("iiwa+allegro.desired_state", AbstractValue.Make([0.0]*30))

        self._traj_X_G_index = self.DeclareAbstractstate(
            AbstractValue.Make(PiecewisePose())
        )
        self._times_index = self.DeclareAbstractState(
            AbstractValue.Make({"initial": 0.0})
        )
        self._attempts_index = self.DeclareDiscreteState(1)

        self.DeclareAbstractOutputPort(
            "X_WG", 
            lambda: AbstractValue.Make(RigidTransform()), 
            self.CalcGripperPose
        )

        num_positions = 30
        self._q0_index = self.DeclareDiscreteState(num_positions)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)
        
        self._estimated_table_height = -0.02

    def Initialize(self, context, state):
        """
        Initialize should make contact with table, and set the initial state of the planner.
        """
        pass

    def Update(self, context, output):
        """
        Update should update the state of the planner.
        """
        mode = context.get_abstract_state(self._mode_index).get_value()
        if mode == PlannerState.SEARCHING:
            mode_update = self.get_input_port(self._contact_results_index).Eval(context)
            if mode_update == PlannerState.INVESTIGATE:
                self.Investigate(context, output)
            elif mode_update == PlannerState.GRASPING:
                self.Pick(context, output)
            elif mode_update == PlannerState.GO_HOME:
                self.GoHome(context, output)
            else:
                # We're still in search mode. Continue searching.
                self.Search(context, output)
                

    def Search(self, context, output):


    def Investigate(self, context, output):
        pass


    def Pick(self, context, output):
        pass


    def GoHome(self, context, output):
        pass


    def CalcGripperPose(self, context, output):
        pass


    def EvaluateContactResults(self, context, output):
        """
        Get current contact results, filter out self collisions,
        update the point cloud, and return desired mode
        """
        pass