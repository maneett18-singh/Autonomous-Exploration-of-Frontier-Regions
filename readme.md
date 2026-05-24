## **Videos**
- [camera.mp4]
<video controls width="100%" src="https://drive.google.com/file/d/1AaV7YE8Q1G4Fb7eU6TS2__y3EIl3stK9/view?usp=drive_link"></video>
- [exploration_final_video(2).mp4]
<video controls width="100%" src="https://drive.google.com/file/d/1n1p2rFSFI5GfZwu6ZfuZdyWLpFBoRnaI/view?usp=drive_link"></video>

# **Autonomous Exploration of Frontier Regions**

## **Overview**

This project focuses on autonomous exploration using frontier-based detection, mapping, and path planning. The robot navigates unexplored regions efficiently while updating an occupancy grid and dynamically planning its path.

## **Key Components**

### **1. Localization & Mapping**

- Uses log-odds representation to create an occupancy grid for mapping the environment.

### **2. Frontier Detection**

- Implements the approach described in [Frontier-Based Exploration](https://arxiv.org/pdf/1806.03581) to identify unexplored regions.

### **3. Path Planning**

- **Global Planner:** A\* algorithm with object inflation for obstacle avoidance.
- **Path Execution:**
  - The `follow_path` function adjusts the robot's speed and orientation based on the computed path.
  - It verifies if a goal has been explored, calculates yaw error, and publishes velocity commands for smooth movement.
  - If the robot reaches a goal or waypoint, it dynamically updates the path to continue exploration.

## **Running the Project**

To launch the project, use the following command:

```sh
source install/setup.bash
```

```sh
colcon build
```

```sh
ros2 launch turtlebot4_pkg launch_file.py
```

## **Demonstration Videos**

Videos demonstrating the project are available in the `videos` folder.

