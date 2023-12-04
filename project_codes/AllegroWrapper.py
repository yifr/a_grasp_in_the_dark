import numpy as np
import pydot
import os
from IPython.display import SVG, display
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    InverseDynamicsController,
    LeafSystem,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    Simulator,
    StartMeshcat,
    RotationMatrix,
    StateInterpolatorWithDiscreteDerivative,
    ConstantVectorSource,
    DiagramBuilder,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    MultibodyPlant,
    Parser,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    Solve,
    StartMeshcat,
    TrajectorySource,
)
from manipulation.station import MakeHardwareStation, load_scenario
from manipulation.scenarios import AddMultibodyTriad, MakeManipulationStation
from manipulation.meshcat_utils import AddMeshcatTriad
from pydrake.multibody import inverse_kinematics

class AllegroHand:
    def __init__(self, station, context):
        self.station = station
        self.plant = station.GetSubsystemByName("plant")
        self.context = context
        self.station_map = {
                            "index_revolute": 7,
                            "index_0": 8,
                            "index_1": 9,
                            "index_2": 10,
                            "thumb_": 11,
                            "thumb_1": 12, 
                            "thumb_revolute_z": 11,
                            "thumb_revolute_y": 12, # avoid
                            "thumb_joint_1": 13,
                            "thumb_joint_2": 14,
                            "middle_revolute": 15,
                            "middle_0": 16,
                            "middle_1": 17,
                            "middle_2": 18,
                            "pinky_revolute": 19,
                            "pinky_0": 20,
                            "pinky_1": 21,
                            "pinky_2": 22
                        }

    def get_state(self, context=None):
        if not context:
            context = self.context

        current_state = self.station.GetInputPort("iiwa+allegro.desired_state").Eval(context)
        return current_state

    def set_state(self, new_state, context=None):
        if not context:
            context = self.context
        self.station.GetInputPort("iiwa+allegro.desired_state").FixValue(context, new_state)
        return

    def get_limb_idx(self, limb):
        return self.allegro_map.get(limb, None)

    def get_limb_idxs(self, limbs):
        idxs = []
        for limb in limbs:
            idx = self.get_limb_idx(limb)
            if idx is not None:
                idxs.append(idx)
            else:
                print("Could not locate limb: " + limb)

        return idxs
        
    def get_positions(self, limbs, context=None):
        state = self.get_state(context)
        positions = []
        for limb in limbs:
            if limb not in self.allegro_map:
                print("Could not find limb titled: " + limb)
                continue
            limb_idx = self.allegro_map[limb]
            positions.append(state[limb_idx])
        
        return positions

    def set_positions(self, limbs, positions, context=None):
        new_state = self.get_state(context)
        limb_idxs = self.get_limb_idxs(limbs)
        for i, limb_idx in enumerate(limb_idxs):
            new_state[limb_idx] = positions[i]
        
        self.set_state(new_state, context)
        return