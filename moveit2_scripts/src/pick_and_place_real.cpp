#include <cstddef>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

/****************************************************
    *               Get current state                  *
    ***************************************************/
  moveit::core::RobotStatePtr current_state_arm = 
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;

/****************************************************
    *           Copy each joint current vlue           *
    ***************************************************/
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

/****************************************************
    *   Debug:: Get link name and joint value of arm   *
    ***************************************************/
    // RCLCPP_INFO(LOGGER, "Frame id: %s", move_group_arm.getPlanningFrame().c_str());
    // RCLCPP_INFO(LOGGER, "End Effector Type: %s", move_group_arm.getEndEffectorLink().c_str());
    // RCLCPP_INFO(LOGGER, "Get Pose reference: %s", move_group_arm.getPoseReferenceFrame().c_str());
    // for(size_t i = 0; i< joint_group_positions_arm.size(); i++){
    //     RCLCPP_INFO(LOGGER, "Pose i joint %zu: %f", i, joint_group_positions_arm[i]);
    // }

/****************************************************
    *           ROS Parameter define in config         *
    ***************************************************/
  std::vector<double> Approach_Retrive;
  std::vector<double> Initial_pose;
  std::vector<double> Hole_pose;
  double z_approach;

  move_group_node->get_parameter("Approach_Retrive", Approach_Retrive);
  move_group_node->get_parameter("intial_pose", Initial_pose);
  move_group_node->get_parameter("perception_pose", Hole_pose);
  move_group_node->get_parameter("z_approach", z_approach);

  double Approach = Approach_Retrive[0];
  double Retrieve = Approach_Retrive[1];

  double init_pose_x = Initial_pose[0];
  double init_pose_y = Initial_pose[1];
  double init_pose_z = Initial_pose[2];

  double hole_pose_x = Hole_pose[0];
  double hole_pose_y = Hole_pose[1];
  double hole_pose_z = Hole_pose[2];


/****************************************************
    *                Close gripper                     *
    ***************************************************/
  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("close");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);


/****************************************************
    *                 Pregrasp                         *
    ***************************************************/
  RCLCPP_INFO(LOGGER, "Pregrasp position");

  joint_group_positions_arm[0] = 1.06;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.07; // Shoulder Lift
  joint_group_positions_arm[2] = 0.700;  // Elbow
  joint_group_positions_arm[3] = -1.202; // Wrist 1
  joint_group_positions_arm[4] = -1.57; // Wrist 2
  joint_group_positions_arm[5] = -0.514;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);


/****************************************************
    *              Grasp pose define                   *
    ***************************************************/
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;

  target_pose1.position.x = 0.083;
  target_pose1.position.y = 0.414;
  target_pose1.position.z = 0.35;
  
/****************************************************
    *                Open gripper                     *
    ***************************************************/

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);


  /****************************************************
    *           Approach to object waypoint            *
    ***************************************************/
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= Approach;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= Approach;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);
  
  RCLCPP_INFO(LOGGER, "Attached Collision Object to the End Effector Link");

/****************************************************
    *                 Take coffee                      *
    ***************************************************/

  RCLCPP_INFO(LOGGER, "Take coffee");

  move_group_gripper.setNamedTarget("hold");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

/****************************************************
    *              Pick coffee up                      *
    ***************************************************/

  RCLCPP_INFO(LOGGER, "Pick coffee up");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += Retrieve;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += Retrieve;
  retreat_waypoints.push_back(target_pose1);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

/********************************************************
    * Pose X and Y from one hole set as reference to move  *
    *******************************************************/
  RCLCPP_INFO(LOGGER, "Position reference");

  geometry_msgs::msg::Pose target_pose2;

  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 0.0;
  target_pose2.orientation.z = 0.0;
  target_pose2.orientation.w = 0.0;

  target_pose2.position.x = init_pose_x;
  target_pose2.position.y = init_pose_y;
  target_pose2.position.z = 0.35;
 
  std::vector<geometry_msgs::msg::Pose> hole_waypoints;

  hole_waypoints.push_back(target_pose2);

  static const std::string END_EFFECTOR_LINK = move_group_arm.getEndEffectorLink();

  moveit_msgs::msg::OrientationConstraint ocm; // Orientation constraint set
  ocm.link_name = END_EFFECTOR_LINK; // Replace with your actual end-effector link name
  ocm.header.frame_id = "base_link"; // Replace with your reference frame
  ocm.orientation.x = 1;
  ocm.orientation.y = 0;
  ocm.orientation.z = 0;
  ocm.orientation.w = 0;
  ocm.absolute_x_axis_tolerance = 0.02; // Adjust tolerance as needed
  ocm.absolute_y_axis_tolerance = 0.02; // Adjust tolerance as needed
  ocm.absolute_z_axis_tolerance = 0.02; // Adjust tolerance as needed
  ocm.weight = 1.0; // Set how importance of this constraints in four constraints. 

  moveit_msgs::msg::Constraints test_constraints; // Add the orientation constraint to the motion plan request
  test_constraints.orientation_constraints.push_back(ocm);

  moveit_msgs::msg::RobotTrajectory hole_trajectory_approach;

  fraction = move_group_arm.computeCartesianPath(
      hole_waypoints, eef_step, jump_threshold, hole_trajectory_approach, test_constraints);

  move_group_arm.execute(hole_trajectory_approach);


/****************************************************
    *           Initail pose to desire place           *
    ***************************************************/

  if(init_pose_x != hole_pose_x || init_pose_y != hole_pose_y ){
    RCLCPP_INFO(LOGGER, "Initial to desire x, y");
    
    std::vector<geometry_msgs::msg::Pose> desire_waypoints;

    target_pose2.position.y += hole_pose_y - init_pose_y;
    target_pose2.position.x += hole_pose_x - init_pose_x;
    desire_waypoints.push_back(target_pose2);

    moveit_msgs::msg::RobotTrajectory desire_trajectory_approach;

    fraction = move_group_arm.computeCartesianPath(
        desire_waypoints, eef_step, jump_threshold, desire_trajectory_approach);
    
    RCLCPP_INFO(LOGGER, "Fraction = %f ", fraction);

    move_group_arm.execute(desire_trajectory_approach);

  }


/****************************************************
    *             Z pose to approach hole              *
    ***************************************************/

  RCLCPP_INFO(LOGGER, "Z pose to approach hole ");

  std::vector<geometry_msgs::msg::Pose> z_approach_waypoints;
  target_pose2.position.z -= z_approach/2.0;
  z_approach_waypoints.push_back(target_pose2);

  target_pose2.position.z -= z_approach/2.0;
  z_approach_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_z_approach;

  fraction = move_group_arm.computeCartesianPath(
      z_approach_waypoints, eef_step, jump_threshold, trajectory_z_approach);

  move_group_arm.execute(trajectory_z_approach);

/****************************************************
    *                Open gripper                     *
    ***************************************************/
  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

/****************************************************
    *               Z axis Retrieve                    *
    ***************************************************/
  RCLCPP_INFO(LOGGER, "Z Retrieve from hole position");

  std::vector<geometry_msgs::msg::Pose> z_retrieve_waypoints;
  target_pose2.position.z += z_approach/1.8;
  z_retrieve_waypoints.push_back(target_pose2);

  target_pose2.position.z += z_approach/1.8;
  z_retrieve_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_z_retrieve;

  fraction = move_group_arm.computeCartesianPath(
      z_retrieve_waypoints, eef_step, jump_threshold, trajectory_z_retrieve);

  move_group_arm.execute(trajectory_z_retrieve);


  
  rclcpp::shutdown();
  return 0;
}