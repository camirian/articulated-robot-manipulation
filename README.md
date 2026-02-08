# Phase 3: Articulated Robot Manipulation

This repository documents **Phase 3** of my AI & Robotics Portfolio, focusing on advanced manipulation tasks with articulated robots. The goal is to move beyond basic control to implement complex, multi-stage manipulation sequences (e.g., Pick and Place) using ROS 2 and NVIDIA Isaac Sim.

For definitions of key terms, please see my central **[AI & Robotics Glossary](https://github.com/camirian/robotics-ontology/blob/main/GLOSSARY.md)**.

---

## ✅ Skills Demonstrated

-   **State Machine Design:** Implementing robust state machines to manage complex robot behaviors (Home -> Grasp -> Lift -> Place).
-   **Trajectory Generation:** Programmatically generating `JointTrajectory` messages to command smooth robot motion.
-   **Sim-to-Real Control:** Designing controllers that act on simulated hardware (Isaac Sim) via standard ROS 2 interfaces, ready for deployment on physical robots.
-   **Python for Robotics:** Utilizing `rclpy` to build modular and reusable ROS 2 nodes.

---

## 🚀 Projects

### Project 3.1: Sim-to-Real Pick and Place Controller

A ROS 2 package (`simple_manipulation`) that implements a state-machine-based controller for a Franka Emika Panda robot.

-   **Node:** `manipulation_controller`
-   **Logic:** Cycles through a predefined sequence of poses to simulate a pick-and-place operation.
-   **Interface:** Publishes to `/franka_joint_trajectory_controller/joint_trajectory`.
-   **[Video Demonstration](LINK_TO_PROJECT_3.1_VIDEO)**

---

### Project 3.2: MoveIt 2 Integration (Dynamic Planning)

Upgrade of the control system to use **MoveIt 2**, the industry standard for motion planning.

-   **Dynamic Planning:** Instead of hardcoded joint angles, we define target *poses* (e.g., "Move gripper to [x, y, z]"). MoveIt calculates the collision-free path.
-   **Bridge Node:** `simple_trajectory_server` translates MoveIt's `FollowJointTrajectory` actions into direct `JointState` commands for Isaac Sim.
-   **Integration:** Full MoveIt stack (MoveGroup, RViz) integrated with the simulation.
-   **[Video Demonstration](LINK_TO_PROJECT_3.2_VIDEO)**

### Project 3.3: Perception Pipeline (Visual Servoing)

Implementation of a closed-loop perception system allowing the robot to detect and interact with objects in the environment.

-   **RGB-D Processing:** Uses OpenCV to detect objects (Red Cube) based on color and depth data from a wrist-mounted camera.
-   **Projection:** Converts 2D pixel coordinates + Depth into precise 3D World Coordinates for the robot.
-   **Visual Servoing:** A coordinator node (`perform_pick`) dynamically commands the MoveIt interface to move the arm to the detected object's location.
-   **[Video Demonstration](PASTE_YOUR_NEW_YOUTUBE_LINK_HERE)**

---

## 🛠️ How to Build and Run

### 1. Build the Workspace
```bash
cd ~/dev/personal/ai-robotics-portfolio/articulated-robot-manipulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### **Terminal 1: The Simulation (Isaac Sim)**
**Crucial:** You must use the python wrapper provided by Isaac Sim to access its libraries.

```bash
# Start the Simulation and ROS 2 Bridge
~/isaac-sim-4.5.0/python.sh scripts/sim_setup.py
```
*Wait for Isaac Sim to load. If it doesn't auto-play, press the **PLAY** button in the UI.*
*(Verify "Status: Ok" for Robot and Image in RViz)*

**Open Terminal 3 (Action):**
```bash
source ros2_ws/install/setup.bash
ros2 run simple_manipulation perform_pick
```
*(The robot will detect the cube and verify alignment by hovering above it)*

---

## 📜 License

This project is licensed under the Apache 2.0 License.
