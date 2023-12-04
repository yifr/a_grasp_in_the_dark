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
import copy

def old_scenario():
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
import copy


def old_scenario():
    # Start the visualizer.
    meshcat = StartMeshcat()
    allegro_hand = "../gripper_sdfs/allegro_hand_description_right.sdf"

    gripper = os.path.join(os.getcwd(), allegro_hand)
    scenario_data = """
        directives:
        - add_model:
            name: iiwa
            file: package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf
            default_joint_positions:
                iiwa_joint_1: [0]
                iiwa_joint_2: [0.1]
                iiwa_joint_3: [0]
                iiwa_joint_4: [-1.5]
                iiwa_joint_5: [0]
                iiwa_joint_6: [0]
                iiwa_joint_7: [0]
        - add_weld:
            parent: world
            child: iiwa::iiwa_link_0
        - add_model:
            name: allegro
            file: file://""" + gripper + """
        - add_weld:
            parent: iiwa::iiwa_link_7
            child: allegro::hand_root
            X_PC:
                translation: [0, 0, 0.05]
                rotation: !Rpy { deg: [0, 0, 0]}
        # And now the environment:
        - add_model:
            name: foam_brick
            file: package://drake/examples/manipulation_station/models/061_foam_brick.sdf
            default_free_body_pose:
                base_link:
                    translation: [0.8, 0, 0]
        - add_model:
            name: robot_table
            file: package://drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf
        - add_weld:
            parent: world
            child: robot_table::link
            X_PC:
                translation: [0, 0, -0.7645]
        - add_model:
            name: work_table
            file: package://drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf
        - add_weld:
            parent: world
            child: work_table::link
            X_PC:
                translation: [0.75, 0, -0.7645]
        model_drivers:
            iiwa+allegro: !InverseDynamicsDriver {}
        """

    scenario = load_scenario(data=scenario_data)
    station = MakeHardwareStation(scenario, meshcat)

    simulator = Simulator(station)
    context = simulator.get_mutable_context()

    x0 = station.GetOutputPort("iiwa+allegro.state_estimated").Eval(context)
    station.GetInputPort("iiwa+allegro.desired_state").FixValue(context, x0)
    simulator.AdvanceTo(0.1)

    return meshcat, station, simulator, context


def build_scenario():
    # Start the visualizer.
    meshcat = StartMeshcat()
    allegro_hand = "../gripper_sdfs/allegro_hand_description_right.sdf"

    gripper = os.path.join(os.getcwd(), allegro_hand)
    foam_brick = os.path.join(os.getcwd(), "../drake_hydro_sdfs/061_foam_brick.sdf")
    table_top = os.path.join(os.getcwd(), "../drake_hydro_sdfs/extra_heavy_duty_table_surface_only_collision.sdf")
    scenario_data = """
        directives:
        - add_model:
            name: iiwa
            file: package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf
            default_joint_positions:
                iiwa_joint_1: [0]
                iiwa_joint_2: [0.1]
                iiwa_joint_3: [0]
                iiwa_joint_4: [-1.5]
                iiwa_joint_5: [0]
                iiwa_joint_6: [0]
                iiwa_joint_7: [0]
        - add_weld:
            parent: world
            child: iiwa::iiwa_link_0
        - add_model:
            name: allegro
            file: file://""" + gripper + """
        - add_weld:
            parent: iiwa::iiwa_link_7
            child: allegro::hand_root
            X_PC:
                translation: [0, 0, 0.05]
                rotation: !Rpy { deg: [0, -90, 0]}
        # And now the environment:
        - add_model:
            name: foam_brick
            file: file://""" + foam_brick + """
            default_free_body_pose:
                base_link:
                    translation: [0.8, 0, 0]
        - add_model:
            name: robot_table
            file: file://""" + table_top + """
        - add_weld:
            parent: world
            child: robot_table::link
            X_PC:
                translation: [0, 0, -0.7645]
        - add_model:
            name: work_table
            file: file://""" + table_top + """
        - add_weld:
            parent: world
            child: work_table::link
            X_PC:
                translation: [0.75, 0, -0.7645]
        model_drivers:
            iiwa+allegro: !InverseDynamicsDriver {}
        """

    scenario = load_scenario(data=scenario_data)
    station = MakeHardwareStation(scenario, meshcat)

    simulator = Simulator(station)
    context = simulator.get_mutable_context()

    x0 = station.GetOutputPort("iiwa+allegro.state_estimated").Eval(context)
    station.GetInputPort("iiwa+allegro.desired_state").FixValue(context, x0)
    simulator.AdvanceTo(0.1)

    return meshcat, station, simulator, context