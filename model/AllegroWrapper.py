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
        self.link_ids = {
            "index_revolute_z": "link_0",
            "index_0": "link_1",
            "index_1": "link_2",
            "index_2": "link_3",
            "middle_revolute_z": "link_4",
            "middle_0": "link_5",
            "middle_1": "link_6",
            "middle_2": "link_7",
            "pinky_revolute_z": "link_8",
            "pinky_0": "link_9",
            "pinky_1": "link_10",
            "pinky_2": "link_11",
            "thumb_revolute_z": "link_12",
            "thumb_revolute_y": "link_13",
            "thumb_1": "link_14",
            "thumb_2": "link_15",
        }

        self.station_map = {
                            "index_revolute_z": 7,
                            "index_0": 8,
                            "index_1": 9,
                            "index_2": 10,
                            "thumb_revolute_z": 11,
                            "thumb_revolute_y": 12, 
                            "thumb_1": 13,
                            "thumb_2": 14,
                            "middle_revolute_z": 15,
                            "middle_0": 16,
                            "middle_1": 17,
                            "middle_2": 18,
                            "pinky_revolute_z": 19,
                            "pinky_0": 20,
                            "pinky_1": 21,
                            "pinky_2": 22
                        }

    def get_limbs(self):
        return self.station_map.keys()

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
        return self.station_map.get(limb, None)

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
            if limb not in self.station_map:
                print("Could not find limb titled: " + limb)
                continue
            limb_idx = self.station_map[limb]
            positions.append(state[limb_idx])
        
        return positions

    def set_positions(self, limbs, positions, context=None):
        new_state = self.get_state(context)
        limb_idxs = self.get_limb_idxs(limbs)
        for i, limb_idx in enumerate(limb_idxs):
            new_state[limb_idx] = positions[i]
        
        self.set_state(new_state, context)
        return

    def reset_arm(self):
        station = self.station
        context = self.context
        simulator = self.simulator
        station.GetInputPort("iiwa+allegro.desired_state").FixValue(context, x0)
        simulator.AdvanceTo(context.get_time() + 1)


    def close_hand(self):
        allegro_state = np.array([-0.1, 1.5, 1.5, -.4, 1.3, 1., 0.7, 0.7, 0, 1.5, 1.5, -.4, 0.1, 1.5, 1.5, -.4])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    def tighten_hand(self):
        allegro_state = np.array([-0.1, 1.6, 1.6, -.4, 1.4, 1.1, 0.8, 0.8, 0, 1.6, 1.6, -.4, 0.1, 1.6, 1.6, -.4])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    def set_explore_mode(self):
        allegro_state = np.array([0, 0, 0, 0, 0.7, 1.16, 1.6, 1.7, 0., 1.3, 1.3, 1.3, 0, 1.61, 1.71, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    
    def moonwalk_index_lead(self):
        # Index finger pushed back
        allegro_state = np.array([0, 0, 0, -.1, 0.7, 1.16, 1.6, 1.7, 0., 0, 0, 0, 0, 1.61, 1.71, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    def moonwalk_index_back(self):
        # Index finger pushed back
        allegro_state = np.array([0, 1.2, 1.1, .4, 0.7, 1.16, 1.6, 1.7, 0., 0, 0, 0, -.2, 1.61, 1.71, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)


    def moonwalk_pinky_lead(self):
        # Pinky finger pushed back
        allegro_state = np.array([0, 0, 0, 0, 0.7, 1.16, 1.6, 1.7, 0., 0, 0, 0, 0, 0, 0, -.1])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    def moonwalk_pinky_back(self):
        # Pinky finger pushed back
        allegro_state = np.array([0, 0, 0, 0, 0.7, 1.16, 1.6, 1.7, 0., 0, 0, 0, 0, 1.61, 1.71, 1.1])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)
        
    def moonwalk_middle_lead(self):
        # Middle finger pushed back
        allegro_state = np.array([0, 0, 0, 0, 0.7, 1.16, 1.6, 1.7, 0., 0, 0, 0, 0, 1.61, 1.71, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    def moonwalk_middle_back(self):
        # Middle finger pushed back
        allegro_state = np.array([0, 0, 0, 0, 0.7, 1., 1.6, 1.7, 0., 1.2, 1.2, 1.2, 0, 0, 0, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)

    def moonwalk_thumb_lead(self):
        # Thumb pushed back
        allegro_state = np.array([0, 0, 0, 0, 0.7, 0, 1.6, 1.7, 0., 0, 0, 0, 0, 1.61, 1.71, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)
    
    def moonwalk_thumb_back(self):
        # Thumb pushed back
        allegro_state = np.array([0, 0, 0, 0, 0, 1.0, 1.6, 1.7, 0., 0, 0, 0, 0, 1.61, 1.71, 0])
        new_state = self.get_state()
        new_state[7:len(allegro_state) + 7] = allegro_state
        self.set_state(new_state)