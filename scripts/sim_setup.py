# sim_setup.py
# Launches Isaac Sim, loads a Franka robot, and sets up the ROS 2 Bridge for JointTrajectory control.

# Start the simulation app
from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable the ROS 2 Bridge extension
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.core.utils.stage import add_reference_to_stage
import numpy as np

def create_ros2_bridge_graph():
    """
    Creates an OmniGraph to bridge ROS 2 JointTrajectory messages to the Articulation Controller.
    """
    keys = og.Controller.Keys
    (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("SubscribeJointState.inputs:topicName", "/joint_command"),
                ("ArticulationController.inputs:targetPrim", "/World/Franka"),
                ("PublishJointState.inputs:topicName", "/joint_states"),
                ("PublishJointState.inputs:targetPrim", "/World/Franka"),
            ],
        },
    )
    print("ROS 2 Bridge Graph created successfully.")

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add Franka Robot
    franka = world.scene.add(
        Franka(
            prim_path="/World/Franka",
            name="franka_robot",
            position=np.array([0.0, 0.0, 0.0])
        )
    )

    # Create the ROS 2 Bridge Graph
    create_ros2_bridge_graph()

    world.reset()
    print("Simulation is ready. Waiting for ROS 2 commands...")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()

if __name__ == "__main__":
    main()
