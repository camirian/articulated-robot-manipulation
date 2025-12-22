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
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.prims as prim_utils
import numpy as np
from isaacsim.core.utils.rotations import euler_angles_to_quat

def create_ros2_bridge_graph(render_product_path):
    """
    Creates an OmniGraph to bridge ROS 2 messages including Joint control and Camera data.
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
                ("CameraHelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelperRGB.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelperInfo.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelperDepth.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
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
                # Camera RGB
                ("CameraHelperRGB.inputs:frameId", "panda_hand_camera"),
                ("CameraHelperRGB.inputs:topicName", "/camera/image_raw"),
                ("CameraHelperRGB.inputs:type", "rgb"),
                ("CameraHelperRGB.inputs:renderProductPath", render_product_path),
                # Camera Info
                ("CameraHelperInfo.inputs:frameId", "panda_hand_camera"),
                ("CameraHelperInfo.inputs:topicName", "/camera/camera_info"),
                ("CameraHelperInfo.inputs:type", "camera_info"),
                ("CameraHelperInfo.inputs:renderProductPath", render_product_path),
                # Camera Depth
                ("CameraHelperDepth.inputs:frameId", "panda_hand_camera"),
                ("CameraHelperDepth.inputs:topicName", "/camera/depth"),
                ("CameraHelperDepth.inputs:type", "depth"),
                ("CameraHelperDepth.inputs:renderProductPath", render_product_path),
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

    # Add Target Cube (Random Position roughly in front of robot)
    # x: 0.3 to 0.6, y: -0.2 to 0.2
    cube_position = np.array([0.5, 0.0, 0.05]) # Fixed for now to verify
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/TargetCube",
            name="target_cube",
            position=cube_position,
            scale=np.array([0.05, 0.05, 0.05]),
            color=np.array([1.0, 0.0, 0.0]), # Red
        )
    )
    # Add Camera to Hand
    camera_path = "/World/Franka/panda_hand/geometry/camera"
    camera = Camera(
        prim_path=camera_path,
        name="hand_camera",
        frequency=30,
        resolution=(640, 480),
        orientation=euler_angles_to_quat(np.array([0, -90, -90]), degrees=True)
    )
    camera.initialize()
    
    # Create Render Product using Replicator
    import omni.replicator.core as rep
    render_product = rep.create.render_product(camera_path, (640, 480))
    render_product_path = render_product.path
    print(f"Render Product created at: {render_product_path}")

    # Create the ROS 2 Bridge Graph
    create_ros2_bridge_graph(render_product_path)

    world.reset()
    world.play() # Ensure timeline is playing
    print("Simulation is ready. Waiting for ROS 2 commands...")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()

if __name__ == "__main__":
    main()
