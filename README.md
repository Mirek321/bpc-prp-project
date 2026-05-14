# ROS2 Humble Autonomous Maze Robot

This repository contains a code, implemented in ROS2 Humble for a robotics project for the BPC-PRP subject on FEKT VUT, focused on autonomous maze navigation. The system integrates sensor feedback, a maze-solving algorithm, and **ArUco marker detection** for localization and decision-making.

## Project Structure

As shown in the repository layout:
* **`src/`**: Core C++ source files for navigation logic and ArUco detection.
* **`include/`**: Header files for the project.
* **`libs/`**: Custom libraries used within the navigation stack.
* **`tests/`**: Unit tests and temporary testing scripts.
* **`CMakeLists.txt` & `package.xml`**: ROS2 build and dependency configuration.

## Key Features

- **Autonomous Maze Navigation**: Implements a maze-solving loop to find the exit efficiently.
- **ArUco Marker Recognition**: Detects markers to determine the shortest way out of the maze.
- **ROS2 Humble Support**: Fully compatible with the Humble distribution.
- **Sensor Integration**: Reads line sensors and distance data for obstacle avoidance and movement.

## Prerequisites

Before running this project, ensure you have:
- **Ubuntu 22.04**
- **ROS2 Humble**
- **OpenCV** (required for ArUco marker processing)
- `cv_bridge` and `sensor_msgs` dependencies.

## Installation & Setup

1.  **Create a ROS2 Workspace** (if you haven't already):
    ```bash
    mkdir -p ~/maze_ws/src
    cd ~/maze_ws/src
    ```

2.  **Clone the Repository**:
    ```bash
    git clone -b maze-solver-with-aruco-code [https://github.com/Mirek321/bpc-prp-project.git](https://github.com/Mirek321/bpc-prp-project.git)
    ```

3.  **Install Dependencies**:
    ```bash
    cd ~/maze_ws
    rosdep install -i --from-path src --rosdistro humble -y
    ```

4.  **Build the Project**:
    ```bash
    colcon build
    ```

## Usage

1.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```

2.  **Launch the Maze Solver**:
    ```bash
    ros2 run prp_project prp_project
    ```
