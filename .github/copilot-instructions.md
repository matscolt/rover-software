
# GORM Rover Project: AI Copilot Instructions

## 1. Purpose

This document provides guidelines for using AI coding assistants (e.g., GitHub Copilot, Gemini) for the GORM ERC 2025 Rover project. The goal is to maximize the benefits of AI-assisted coding while ensuring that all generated code adheres to our established software architecture and quality standards.

**All team members should read and follow these guidelines when writing or generating code.**

---

## 2. Core Architectural Principles (from ADR-001)

Before writing or generating any code, remember our foundational principles. Your prompts should be framed to produce code that respects these rules:

* **Modularity is Key:** Our system is built on loosely-coupled ROS 2 packages. Each package has a single, well-defined responsibility. **Never** write code that blends the responsibilities of different packages (e.g., do not put perception logic inside a control package).
* **Follow the Phased Approach:** We build simple, reliable systems first, then add complexity. Prioritize baseline functionality (Phase 1 & 2) before adding advanced features (Phase 3).
* **Prioritize Testability:** All code should be written with testing in mind. Generated code should be simple enough to be easily testable in isolation and in simulation.
* **Language Tenets:**
    * Use **C++** for performance-critical nodes (controllers, filters, low-level perception).
    * Use **Python** for high-level logic, state machines, and rapid prototyping.

---

## 3. The ROS 2 Package Architecture

This is the single source of truth for our software structure. All new code must live within one of these packages, according to its purpose.

* **`gorm_description`**
    * **Purpose:** Contains the rover's URDF model.
    * **Key Node:** `robot_state_publisher_node`.
* **`gorm_bringup`**
    * **Purpose:** Launches the entire rover system.
    * **Key Nodes:** `robot_state_publisher_node`, `ros2_control_node`,
    * **Key Launch Files:** `teleop.launch.py`, `simulation.launch.py`, `navigation.launch.py`.

* **`gorm_teleop`**
    * **Purpose:** Provides manual control of the rover.
    * **Key Node:** `teleop_twist_joy_node` or equivalent.

* **`gorm_sensors`**
    * **Purpose:** Manages all sensor drivers and launch files.
    * **Key Nodes:** Drivers for RGBD cameras, IMU, etc. Publishes raw sensor data.

* **`gorm_base_control`**
    * **Purpose:** The critical interface to our 6-wheel, 4-steer kinematics.
    * **Key Node:** A C++ node subscribing to `/cmd_vel` and publishing motor commands and `/odom`.

* **`gorm_perception`**
    * **Purpose:** Processes raw sensor data into meaningful information for navigation.
    * **Key Nodes:** `pointcloud_to_laserscan_node`, `costmap_assembler_node`, `aruco_detector_node`, and future AI inference clients.

* **`gorm_localization`**
    * **Purpose:** Provides an accurate estimate of the rover's position.
    * **Key Node:** `robot_localization` EKF node (`ekf_filter_node`).

* **`gorm_navigation`**
    * **Purpose:** Contains all configurations and launch files for the Nav2 stack.
    * **Key Nodes:** Standard Nav2 servers (Planner, Controller, etc.).

The above packages are placed in the `src/rover` directory of the repository. Also note that the `src/rover` directory is under development, and adherence to the aforementioned architecture is not fully enforced yet, but is critical for future development.
---

## 4. How to Prompt an AI Assistant

The quality of AI-generated code is directly proportional to the quality of your prompt. **Always provide context.**

### Prompt Template

Use this template as a starting point for your requests:

> **Goal:** [Describe your specific goal, e.g., "Create a ROS 2 node that subscribes to /cmd_vel..."]
>
> **Project Context:** We are building a 6-wheel, 4-steer rover using ROS 2. This node belongs in the `[package_name]` package. Its purpose is to [re-state purpose from the architecture list].
>
> **Constraints:**
> -   **Language:** [C++ or Python] but prefer Python for most implementations unless performance is critical.
> -   **Key Inputs:** [e.g., `geometry_msgs/Twist` on the `/cmd_vel` topic]
> -   **Key Outputs:** [e.g., `nav_msgs/Odometry` on the `/odom` topic]
> -   **Other:** [e.g., "This node should not perform any perception tasks."]

### Examples

#### GOOD Prompt (Specific and Context-Aware)

> **Goal:** "Generate the boilerplate for a C++ ROS 2 node named `GormBaseControlNode`."
>
> **Project Context:** "This node is the core of our `gorm_base_control` package for a 6-wheel, 4-steer rover. Its purpose is to translate velocity commands into motor commands and publish odometry."
>
> **Constraints:**
> -   **Language:** C++
> -   **Inputs:** Create a subscriber to the `/cmd_vel` topic (`geometry_msgs/Twist`).
> -   **Outputs:** Create a publisher for the `/odom` topic (`nav_msgs/Odometry`) and a `tf2` broadcaster for the `odom` -> `base_link` transform.
> -   **Other:** "Just create the class structure, member variables, and the subscriber/publisher/broadcaster setup in the constructor. Leave the callback logic empty for now."

#### BAD Prompt (Vague and Lacking Context)

> "Write a ROS 2 node to control a robot."

This bad prompt will produce generic code that doesn't fit our architecture, doesn't know about our topics, and is ultimately useless without significant rework.

---

By following these instructions, we can ensure that our use of AI accelerates our development while maintaining the integrity and quality of our rover's software.
