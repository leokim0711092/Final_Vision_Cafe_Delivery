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

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();


RCLCPP_INFO(LOGGER, "Frame id: %s", move_group_arm.getPlanningFrame().c_str());
RCLCPP_INFO(LOGGER, "End Effector Type: %s", move_group_arm.getEndEffectorLink().c_str());

  // Create collision object for the robot to avoid
  auto const collision_object_cube = [frame_id =
                                          move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "coffee";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.10;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.02;


    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.46;
    box_pose.position.z = 0.05;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_3;

  planning_scene_interface_3.applyCollisionObject(collision_object_cube);

  RCLCPP_INFO(LOGGER, "Create Collision");

  //   Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("close");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  move_group_arm.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  //   Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 0.5003;
  target_pose1.orientation.y = 0.4996;
  target_pose1.orientation.z = 0.5007;
  target_pose1.orientation.w = -0.499;

  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.10;
  
  /*Test code to check the exact rpy we need */
//   double x = 0.3;  
//   double y = 0.3;  
//   double z = 0.20;

//   double roll = 1.57;  
//   double pitch = -1.57;  
//   double yaw = 3.14;
//   move_group_arm.setPositionTarget (x,y,z);
//   move_group_arm.setRPYTarget(roll,pitch,yaw);


  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

    //   Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("Open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);


  /* First, define the REMOVE object message*/
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "coffee";
  remove_object.header.frame_id = "world";
  remove_object.operation = remove_object.REMOVE;
  std::vector<std::string> object_ids;
  object_ids.push_back("coffee");

//   // Open Gripper

//   RCLCPP_INFO(LOGGER, "Open Gripper");

//   move_group_gripper.setNamedTarget("open");

//   success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                      moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_gripper.execute(my_plan_gripper);

//   // Approach
//   RCLCPP_INFO(LOGGER, "Approach to object!");

//   std::vector<geometry_msgs::msg::Pose> approach_waypoints;
//   target_pose1.position.z -= 0.14;
//   approach_waypoints.push_back(target_pose1);

//   target_pose1.position.z -= 0.14;
//   approach_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_approach;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;

//   double fraction = move_group_arm.computeCartesianPath(
//       approach_waypoints, eef_step, jump_threshold, trajectory_approach);

//   move_group_arm.execute(trajectory_approach);

  //   Remove  Collision Object
  planning_scene_interface_3.removeCollisionObjects(object_ids);

//   //   Close Gripper

//   RCLCPP_INFO(LOGGER, "Close Gripper!");

//   move_group_gripper.setNamedTarget("close");

//   success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                      moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_gripper.execute(my_plan_gripper);

//   //    Retreat

//   RCLCPP_INFO(LOGGER, "Retreat from object!");

//   std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
//   target_pose1.position.z += 0.20;
//   retreat_waypoints.push_back(target_pose1);

//   target_pose1.position.z += 0.20;
//   retreat_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_retreat;

//   fraction = move_group_arm.computeCartesianPath(
//       retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

//   move_group_arm.execute(trajectory_retreat);

  rclcpp::shutdown();
  return 0;
}