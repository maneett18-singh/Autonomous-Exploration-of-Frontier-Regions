## **Videos**
- [camera.mp4]


https://github.com/user-attachments/assets/bfcaf1ca-c7cf-4a5b-9492-19b4aa81b13b


- [exploration_final_video(2).mp4]
<img width="800" height="450" alt="exploration_final_video2-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/71872979-d9cb-41d4-a403-af2aa7b890d7" />

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

