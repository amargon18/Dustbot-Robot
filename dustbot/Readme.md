DUSTBOT FINAL PROJECT.

The final goal of the project is to create a virtual robot (DustBot) that navigates an environment, locating and collecting trash. Using ROS 2 and custom services (`SetDirection` and `LoadGarbage`), the robot moves to random positions where trash is located and collects it. The system is designed to operate in a configurable grid size, with interaction between control and simulation nodes to manage movement and trash collection.

1. PROJECT STRUCTURE.
dustbot_ws
|
|__src
    |
    |__dustbot
    |        |
    |        |__dustbot
    |        |        |
    |        |        |__world_node.py
    |        |        |
    |        |        |__robot_node.py
    |        |    
    |        |___________ launch
    |                        |
    |                        |__dustbot_launch.py
    |    
    |
    |__dustbot_interfaces
                        |
                        |__srv
                            |
                            |__SetDirection.srv
                            |
                            |__LoadGarbage.srv


- world_node.py: This code defines the `WorldNode` in ROS 2, responsible for managing a robot on an NxN grid. It publishes the robot's position (`/dustbot/global_position`) and the garbage's position (`/dustbot/garbage_position`), while responding to services for moving the robot (`/dustbot/set_direction`) and collecting garbage (`/dustbot/load_garbage`). Once P garbage items are collected, the node automatically shuts down.

- robot_node.py: This code defines the `RobotNode` in ROS 2, responsible for controlling a robot that moves toward garbage positions published on `/dustbot/garbage_position`. The node calculates directions, updates the robot's position, and attempts to collect the garbage by interacting with services `/dustbot/set_direction` and `/dustbot/load_garbage`. It also ensures the robot stops once it has collected all designated garbage.

- SetDirection.srv: The `SetDirection` service allows the robot to receive a direction as a string (e.g., "Up", "Down") and move accordingly. The response indicates whether the direction was successfully set with a boolean `success`.

- LoadGarbage.srv: The `LoadGarbage` service attempts to collect garbage, returning a `success` boolean indicating whether the collection was successful. The `message` provides additional details about the result.

2. HOW TO RUN THE APPLICATION?
    1. Open the terminal
    2. Into the workspace, change the directory to dustbot_ws/src
    3. Compile the packages with colcon build.
    4. Configure the ROS2 environment after colcon build with source/install/setup.bash
    5. Change the directory to dustbot_ws/src/dustbot/launch
    6. Run the launch file with ros2 launch dustbot_launch.py N:__  P:__
        (N*N grid, P picks).
