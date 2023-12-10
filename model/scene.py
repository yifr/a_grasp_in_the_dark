import os
import pydot
import numpy as np
from pydrake.all import (
    Role,
    RevoluteJoint,
    RigidTransform,
    RollPitchYaw,
    Parser,
    Simulator,
    DiagramBuilder,
    StartMeshcat,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    MeshcatVisualizerParams
)
from IPython.display import SVG, display
from manipulation.station import MakeHardwareStation, load_scenario


def init_scenario(brick_location=None, brick_rotation=None, meshcat=None):
    if meshcat is None:
        meshcat = StartMeshcat()
    
    if brick_location is None:
        table_xmin = 0.55
        table_xmax = 0.75
        table_ymin = -0.3
        table_ymax = 0.3
        x = np.random.uniform(table_xmin, table_xmax)
        y = np.random.uniform(table_ymin, table_ymax)
        z = 0
        brick_location = [x, y, z] 

    if brick_rotation is None:
        brick_rotation = [0, 0, np.random.randint(0, 360)]

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
            X_PC:
                translation: [-0.1, 0, 0]
                rotation: !Rpy { deg: [0, 0, 0]}
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
            file: file://""" + foam_brick + """
            default_free_body_pose:
                base_link:
                    translation: """ + str(brick_location) + """
                    rotation: !Rpy { deg: """ + str(brick_rotation) + """}
        - add_model:
            name: robot_table
            file: file://""" + table_top + """
        - add_weld:
            parent: world
            child: robot_table::link
            X_PC:
                translation: [-0.1, 0, -0.7645]
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

    return meshcat, station, simulator, context, brick_location, brick_rotation


def render_station_diagram(station):
    display(
    SVG(
        pydot.graph_from_dot_data(station.GetGraphvizString(max_depth=1))[
            0
        ].create_svg()
    )
    )

def add_objects(plant, p_Wbrick=None, R_Wbrick=None):
    parser = Parser(plant)

    if p_Wbrick == None:
        table_xmin = 0.55
        table_xmax = 0.75
        table_ymin = -0.3
        table_ymax = 0.3
        x = np.random.uniform(table_xmin, table_xmax)
        y = np.random.uniform(table_ymin, table_ymax)
        z = 0
        p_Wbrick = [x, y, z] 
    if R_Wbrick == None:    
        R_Wbrick = [0, 0, np.random.randint(0, 360)]

    foam_brick = os.path.join(os.getcwd(), "../drake_hydro_sdfs/061_foam_brick.sdf")
    table_top = os.path.join(os.getcwd(), "../drake_hydro_sdfs/extra_heavy_duty_table_surface_only_collision.sdf")

    scenario_data = """
        directives:
        - add_model:
            name: foam_brick
            file: file://""" + foam_brick + """
            default_free_body_pose:
                base_link:
                    translation: """ + str(p_Wbrick) + """
                    rotation: !Rpy { deg: """ + str(R_Wbrick) + """}
        - add_model:
            name: robot_table
            file: file://""" + table_top + """
        - add_weld:
            parent: world
            child: robot_table::link
            X_PC:
                translation: [-0.1, 0, -0.7645]
        - add_model:
            name: work_table
            file: file://""" + table_top + """
        - add_weld:
            parent: world
            child: work_table::link
            X_PC:
                translation: [0.75, 0, -0.7645]
        """

    objects = parser.AddModelsFromString(scenario_data, ".dmd.yaml")[0]
    return objects, p_Wbrick, R_Wbrick

def add_allegro(plant, iiwa_instance):
    parser = Parser(plant)
    gripper_path = os.path.join(os.getcwd(), "../gripper_sdfs/allegro_hand_description_right.sdf")
    directives = f"""
    directives:
    - add_model:
            name: allegro
            file: file://{gripper_path} """ 
    gripper = parser.AddModelsFromString(directives, ".dmd.yaml")[0]
    
    X_7G = RigidTransform(RollPitchYaw(0, 0, np.pi), [0, 0, 0.05])
    plant.WeldFrames(
        plant.GetFrameByName("iiwa_link_7", iiwa_instance),
        plant.GetFrameByName("hand_root", gripper),
        X_7G,
    )
    return gripper

def add_iiwa(plant):
    parser = Parser(plant)
    directives = """
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
        X_PC:
            translation: [-0.1, 0, 0]
            rotation: !Rpy { deg: [0, 0, 0]}
    """
    print(directives)
    iiwa = parser.AddModelsFromString(directives, ".dmd.yaml")[0]
    return iiwa

def build_manipulation_station(meshcat=None, p_Wbrick=None, R_Wbrick=None):
    """
    Returns manipulation station, context, and simulator
    """
    if meshcat is None:
        meshcat = StartMeshcat()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)

    iiwa = add_iiwa(plant)
    gripper = add_allegro(plant, iiwa)
    objects, p_Wbrick, R_Wbrick = add_objects(plant, p_Wbrick, R_Wbrick)
    
    plant.Finalize()

    
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, 
        meshcat,
        MeshcatVisualizerParams(delete_prefix_initialization_event=False)
    )   

    collision = MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        MeshcatVisualizerParams(
            prefix="collision", role=Role.kProximity, visible_by_default=False
        ),
    )

    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    builder.ExportOutput(
        plant.get_contact_results_output_port(), "contact_results"
    )
    builder.ExportOutput(
        plant.get_state_output_port(), "plant_continuous_state"
    )
    builder.ExportOutput(plant.get_body_poses_output_port(), "body_poses")

    station = builder.Build()
    station.set_name("station")

    context = station.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    simulator = Simulator(station)
    context = simulator.get_mutable_context()

    q0 = plant.GetPositions(plant_context)
    
    simulator.AdvanceTo(0.1)
    gripper_frame = plant.GetFrameByName("hand_root", gripper)
    return meshcat, simulator, station, plant, context, q0, p_Wbrick, R_Wbrick