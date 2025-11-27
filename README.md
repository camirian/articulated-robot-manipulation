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

## 🛠️ How to Build and Run

1.  Navigate to the workspace:
    ```bash
    cd ~/dev/personal/ai-robotics-portfolio/articulated-robot-manipulation/ros2_ws
    ```
2.  Build the package:
    ```bash
    colcon build
    ```
3.  Source the overlay:
    ```bash
    source install/setup.bash
    ```
4.  Run the controller:
    ```bash
    ros2 run simple_manipulation manipulation_controller
    ```

---

## 📜 License

This project is licensed under the Apache 2.0 License.
