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

---

### Project 3.2: MoveIt 2 Integration (Dynamic Planning)

Upgrade of the control system to use **MoveIt 2**, the industry standard for motion planning.

-   **Dynamic Planning:** Instead of hardcoded joint angles, we define target *poses* (e.g., "Move gripper to [x, y, z]"). MoveIt calculates the collision-free path.
-   **Bridge Node:** `simple_trajectory_server` translates MoveIt's `FollowJointTrajectory` actions into direct `JointState` commands for Isaac Sim.
-   **Integration:** Full MoveIt stack (MoveGroup, RViz) integrated with the simulation.

---

## 🛠️ How to Build and Run

### 1. Build the Workspace
```bash
cd ~/dev/personal/ai-robotics-portfolio/articulated-robot-manipulation/ros2_ws
colcon build
source install/setup.bash
```

### 2. Launch the Simulation (Isaac Sim)
We use a custom Python script to automate the environment setup (loading robot + ROS 2 Bridge).
**Open Terminal 1:**
```bash
~/isaac-sim-4.5.0/python.sh scripts/sim_setup.py
```
*Wait for the robot to appear.*

### 3. Run a Project

**Option A: Simple State Machine (Project 3.1)**
**Open Terminal 2:**
```bash
source ros2_ws/install/setup.bash
ros2 run simple_manipulation manipulation_controller
```

**Option B: MoveIt 2 Demo (Project 3.2)**
**Open Terminal 2:**
```bash
source ros2_ws/install/setup.bash
ros2 launch simple_manipulation bringup_moveit.launch.py
```
*In RViz, drag the interactive marker and click "Plan & Execute" to move the robot in the simulator.*

---

## 📜 License

This project is licensed under the Apache 2.0 License.
