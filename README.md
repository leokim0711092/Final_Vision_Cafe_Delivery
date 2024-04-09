# RoboCafé - Vision based cafe delivery

RoboCafé aims to revolutionize the café experience by introducing an automated, vision-based delivery system. The project employs cutting-edge technology to automate the delivery of beverages, utilizing a robotic arm to ensure precision and efficiency. The core idea is to streamline café operations and offer a novel, tech-driven service to customers.

## Workflow Overview:

1. Web Request Initiation: 
Customers initiate beverage orders through a user-friendly web interface. These requests are then forwarded to a Docker container, which acts as the central processing unit for the operation.

2. Perception and Analysis: 
The system employs an Intel D415 RGB camera to capture detailed visual data of the café environment. The Point Cloud Library (PCL) processes this data to understand the space and identify the precise location for cup placement.

3. Position Determination: 
A mathematical model, specifically the least square approximation, is used to slove the ideal circle equation. This calculation precisely determines where the robotic arm should place the beverage to ensure accuracy and stability.

4. Trajectory Planning and Execution: 
Utilizing MoveIt2, the system plans the trajectory for the UR3e robotic arm. This ensures the arm moves smoothly and accurately to place the coffee in the determined location, completing the delivery process.

## Requirements

- ROS2 (Robot Operating System 2)

- C++ Compiler

- Gazebo Simulation / Appropriate Robot Hardware
  
- PCL Library

- Moveit2

## Installation

1\. Clone the repository:

   ```
   git clone https://github.com/leokim0711092/Final_Vision_Cafe_Delivery.git
   ```

2\. Build the project:
   ```
   cd ~/ros2_ws
   colcon build && source install/setup.bash
   ```

## Usage for Simulation
1\. Gazebo launch:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
   ```
2\. Webserver launch:
   ```
   cd ~/ros2_ws/src/webpage && python3 -m http.server 7000
   ```
3\. Rosbridge server launch:
   ```
   source ~/ros2_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
4\. All service launch:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 launch moveit2_scripts All_Exec_Sim.launch.py  
   ```
5\. Service for receive request from webpage:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 run moveit2_scripts pick_service_node
   ```
6. Spawn coffee
   ```
   ros2 run gazebo_ros spawn_entity.py -file /home/user/ros2_ws/src/coffee_service_perception/urdf/portable_cup.urdf -entity coffee_1 -x 13.948 -y -18.106 -z 1.025 -R 1.57 -P 0 -Y 0
   ros2 run gazebo_ros spawn_entity.py -file /home/user/ros2_ws/src/coffee_service_perception/urdf/portable_cup.urdf -entity coffee_2 -x 13.948 -y -18.106 -z 1.025 -R 1.57 -P 0 -Y 0
   ros2 run gazebo_ros spawn_entity.py -file /home/user/ros2_ws/src/coffee_service_perception/urdf/portable_cup.urdf -entity coffee_3 -x 13.948 -y -18.106 -z 1.025 -R 1.57 -P 0 -Y 0
   ros2 run gazebo_ros spawn_entity.py -file /home/user/ros2_ws/src/coffee_service_perception/urdf/portable_cup.urdf -entity coffee_4 -x 13.948 -y -18.106 -z 1.025 -R 1.57 -P 0 -Y 0
   ```
## Usage for Real lab

1\. Webserver launch:

   ```
   cd ~/ros2_ws/src/webpage && python3 -m http.server 7000
   ```
2\. Rosbridge server launch:
   ```
   source ~/ros2_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
3\. All service launch:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 launch moveit2_scripts All_Exec_Real.launch.py 
   ```
4\. Service for receive request from webpage:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 run moveit2_scripts pick_service_node
   ```
## Demo

[RoboCafe' demo
](https://www.youtube.com/watch?v=-7of2jbOiUU)

## Authors

- [@leokim0711092](https://github.com/leokim0711092)

## License
This project was created under the supervision of the team at [The Construct](https://theconstructsim.com/)
