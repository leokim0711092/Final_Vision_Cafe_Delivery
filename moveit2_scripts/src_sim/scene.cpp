#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("scene");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node =
      rclcpp::Node::make_shared("scene", node_options);

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

  
  // Table
  auto const collision_object_table = [frame_id =
                                           move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object_1;
    collision_object_1.header.frame_id = frame_id;
    collision_object_1.id = "box2";
    shape_msgs::msg::SolidPrimitive primitive_1;

    // Define the size of the box in meters
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[primitive_1.BOX_X] = 0.79;
    primitive_1.dimensions[primitive_1.BOX_Y] = 1.81;
    primitive_1.dimensions[primitive_1.BOX_Z] = 0.05;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose_1;
    box_pose_1.orientation.w = 1.0;
    box_pose_1.position.x = 0.3;
    box_pose_1.position.y = 0.36;
    box_pose_1.position.z = -0.026; // add more than

    collision_object_1.primitives.push_back(primitive_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;

    return collision_object_1;
  }();

  // Wall
  auto const collision_object_wall = [frame_id =
                                           move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object_1;
    collision_object_1.header.frame_id = frame_id;
    collision_object_1.id = "wall";
    shape_msgs::msg::SolidPrimitive primitive_1;

    // Define the size of the box in meters
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[primitive_1.BOX_X] = 1.7;
    primitive_1.dimensions[primitive_1.BOX_Y] = 0.05;
    primitive_1.dimensions[primitive_1.BOX_Z] = 1.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose_1;
    box_pose_1.orientation.w = 1.0;
    box_pose_1.position.x = 0.0;
    box_pose_1.position.y = -0.3;
    box_pose_1.position.z = 0.3; // add more than

    collision_object_1.primitives.push_back(primitive_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;

    return collision_object_1;
  }();

// Coffee machine base
  auto const collision_object_coffee_machine_base = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "coffee base";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.48;
    primitive.dimensions[primitive.BOX_Y] = 0.23;
    primitive.dimensions[primitive.BOX_Z] = 0.47;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0; 
    box_pose.position.x = 0.34;
    box_pose.position.y = 0.86;
    box_pose.position.z = 0.24;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();


  // Coffee machine head
  auto const collision_object_coffee_machine_head = [frame_id =
                                          move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "coffee machine head";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.23;
    primitive.dimensions[primitive.BOX_Z] = 0.14;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    box_pose.position.x = 0.05;
    box_pose.position.y = 0.86;
    box_pose.position.z = 0.405;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

    // Coffee machine nozzle
  auto const collision_object_coffee_machine_nozzle = [frame_id =
                                          move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "coffee machine nozzle";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.11;
    primitive.dimensions[primitive.BOX_Z] = 0.07;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    box_pose.position.x = 0.05;
    box_pose.position.y = 0.86;
    box_pose.position.z = 0.3;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Create collision object for the robot to avoid
  auto const collision_object_cube = [frame_id =
                                          move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    // collision_object.header.frame_id = "camera_camera_depth_optical_frame";
    collision_object.id = "coffee";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    // primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.09;

    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.03;


    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.33;
    box_pose.position.z = 0.05;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

auto const collision_object_plate = [frame_id =
                                          move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "plate";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    // primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;

    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.175;


    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    box_pose.position.x = -0.38;
    box_pose.position.y = 0.05;
    box_pose.position.z = -0.66;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

// Wall
  auto const collision_object_jaw_1 = [frame_id =
                                           move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object_1;
    collision_object_1.header.frame_id = frame_id;
    collision_object_1.id = "jaw_1";
    shape_msgs::msg::SolidPrimitive primitive_1;

    // Define the size of the box in meters
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[primitive_1.BOX_X] = 0.13;
    primitive_1.dimensions[primitive_1.BOX_Y] = 0.01;
    primitive_1.dimensions[primitive_1.BOX_Z] = 0.03;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose_1;
    box_pose_1.orientation.w = 1.0;
    box_pose_1.position.x = -0.065;
    box_pose_1.position.y = 0.18;
    box_pose_1.position.z = 0.015; // add more than

    collision_object_1.primitives.push_back(primitive_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;

    return collision_object_1;
  }();

  // Wall
  auto const collision_object_jaw_2 = [frame_id =
                                           move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object_1;
    collision_object_1.header.frame_id = frame_id;
    collision_object_1.id = "jaw_2";
    shape_msgs::msg::SolidPrimitive primitive_1;

    // Define the size of the box in meters
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[primitive_1.BOX_X] = 0.13;
    primitive_1.dimensions[primitive_1.BOX_Y] = 0.01;
    primitive_1.dimensions[primitive_1.BOX_Z] = 0.03;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose_1;
    box_pose_1.orientation.w = 1.0;
    box_pose_1.position.x = -0.065;
    box_pose_1.position.y = 0.50;
    box_pose_1.position.z = 0.015; // add more than

    collision_object_1.primitives.push_back(primitive_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;

    return collision_object_1;
  }();
  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   moveit::planning_interface::PlanningSceneInterface coffee;

  planning_scene_interface.applyCollisionObject(collision_object_table);
  planning_scene_interface.applyCollisionObject(collision_object_wall);
  planning_scene_interface.applyCollisionObject(collision_object_coffee_machine_base);
  planning_scene_interface.applyCollisionObject(collision_object_coffee_machine_head);
  planning_scene_interface.applyCollisionObject(collision_object_coffee_machine_nozzle);
  planning_scene_interface.applyCollisionObject(collision_object_jaw_1);
  planning_scene_interface.applyCollisionObject(collision_object_jaw_2);

//   planning_scene_interface_5.applyCollisionObject(collision_object_plate);

//   coffee.applyCollisionObject(collision_object_cube);

  RCLCPP_INFO(LOGGER, "Create Scene");

  rclcpp::shutdown();
  return 0;
}