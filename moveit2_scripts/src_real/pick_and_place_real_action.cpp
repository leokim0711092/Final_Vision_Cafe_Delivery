#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <sstream>
#include "custom_interfaces/action/pick_and_place.hpp"
#include "custom_interfaces/action/find_hole_position.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <chrono>
namespace Pick_And_Place {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick and place action");
static const rclcpp::Logger LOGGER2 = rclcpp::get_logger("Perception action in moveit action");

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
class MoveitAction : public rclcpp::Node 
{
  using PickAndPlaceAction =
      custom_interfaces::action::PickAndPlace;
  using PickAndPlaceActionGoal =
      rclcpp_action::ServerGoalHandle<PickAndPlaceAction>;
  using FindHolesAction =
      custom_interfaces::action::FindHolePosition;
  using FindHoleGoalHandle =
      rclcpp_action::ClientGoalHandle<FindHolesAction>;

public:
  explicit MoveitAction(const rclcpp::NodeOptions &options)
      : rclcpp::Node("pick_and_place_action", options), goal_done_(false){   

    // Using a very short timer to delay initialization
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                            std::bind(&MoveitAction::timer_callback, this));
    receive = false; // blocking the moveit motion 

    /****************************************************
    *           ROS Parameter define in config         *
    ***************************************************/
    std::vector<double> Approach_Retrive;
    std::vector<double> object_pose;

    Approach_Retrive = this->declare_parameter<std::vector<double>>("Approach_Retrive", {});
    z_approach = this->declare_parameter<double>("z_approach", 0.0);
    deviation = this->declare_parameter<double>("deviation", 0.0);

    object_dimensions = this->declare_parameter<std::vector<double>>("object_dimensions", {});
    object_pose = this->declare_parameter<std::vector<double>>("object_pose", {});

    RCLCPP_INFO(this->get_logger(), "Z approach is %f", z_approach);
    RCLCPP_INFO(this->get_logger(), "Deviation is %f", deviation);

    Approach = Approach_Retrive[0];
    Retrieve = Approach_Retrive[1];
    cup_pose.x = object_pose[0];
    cup_pose.y = object_pose[1];
    cup_pose.z = object_pose[2];

    // Setup actionlib server
    server_ = rclcpp_action::create_server<PickAndPlaceAction>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "pick_and_place",
        std::bind(&MoveitAction::handle_goal, this, _1, _2),
        std::bind(&MoveitAction::handle_cancel, this, _1),
        std::bind(&MoveitAction::handle_accepted, this, _1));
    
    client_ptr_ = rclcpp_action::create_client<FindHolesAction>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_hole");

}

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = FindHolesAction::Goal();
    goal_msg.find_hole = true;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FindHolesAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MoveitAction::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MoveitAction::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MoveitAction::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
    rclcpp_action::Server<PickAndPlaceAction>::SharedPtr server_;
    rclcpp_action::Client<FindHolesAction>::SharedPtr client_ptr_;
    bool goal_done_;
    bool receive;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper;
    rclcpp::TimerBase::SharedPtr timer_;

    double init_pose_x;
    double init_pose_y;
    double init_pose_z;

    /****************************************************
    *        Parameter get from perception action       *
    ***************************************************/
    std::vector<geometry_msgs::msg::Point> hole_position;
    geometry_msgs::msg::Point plate_center;
    /****************************************************
    *           ROS Parameter get         *
    ***************************************************/
    geometry_msgs::msg::Point cup_pose;
    std::vector<double> object_dimensions;
    double z_approach;
    double deviation;

    double Approach;
    double Retrieve;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const PickAndPlaceAction::Goal> goal_handle) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<PickAndPlaceActionGoal> goal_handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<PickAndPlaceActionGoal> goal_handle) {

        // Break off a thread
        std::thread{std::bind(&MoveitAction::execute, this, _1), goal_handle}.detach();
    }

    void initialize() {

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        
    }
    void timer_callback() {

        this->timer_->cancel();
        initialize();
    }
    void execute(const std::shared_ptr<PickAndPlaceActionGoal> goal_handle) {
        
        auto result = std::make_shared<PickAndPlaceAction::Result>();
        auto feedback = std::make_shared<PickAndPlaceAction::Feedback>();

        const auto goal = goal_handle->get_goal();

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // Get objects
        send_goal();
        while (!receive) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            feedback->state = "Perception: finding places to put coffee";
            goal_handle->publish_feedback(feedback);
        }
        int cup_number = goal->cup_number;

        for(int i =0; i< cup_number;i++){      
            arm_execute(goal_handle, i, hole_position[i], plate_center);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }

        result->result = "Succeed";
        goal_handle->succeed(result);
        RCLCPP_INFO(LOGGER, "After succeed");
        receive = false;

    }
    void arm_execute(std::shared_ptr<PickAndPlaceActionGoal> goal_handle, int i, geometry_msgs::msg::Point hole_position, geometry_msgs::msg::Point plate_center ){
        auto feedback = std::make_shared<PickAndPlaceAction::Feedback>();

        feedback->state = "Grasping number " + std::to_string(i+1)+ " cup";
        goal_handle->publish_feedback(feedback);

        const moveit::core::JointModelGroup *joint_model_group_arm =
            move_group_arm->getCurrentState()->getJointModelGroup("ur_manipulator");

        const moveit::core::JointModelGroup *joint_model_group_gripper =
            move_group_gripper->getCurrentState()->getJointModelGroup(
                "gripper");

        /****************************************************
        *               Get current state                  *
        ***************************************************/
        moveit::core::RobotStatePtr current_state_arm = 
            move_group_arm->getCurrentState(10);
        moveit::core::RobotStatePtr current_state_gripper =
            move_group_gripper->getCurrentState(10);

        std::vector<double> joint_group_positions_arm;
        std::vector<double> joint_group_positions_gripper;

        /****************************************************
        *           Copy each joint current vlue           *
        ***************************************************/
        current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                    joint_group_positions_arm);
        current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                        joint_group_positions_gripper);

        move_group_arm->setStartStateToCurrentState();
        move_group_gripper->setStartStateToCurrentState();

        /****************************************************
        *           Parameter from perception action         *
        ***************************************************/
        if (i == 0) {
            // init_pose_x = hole_position.x-0.01;
            init_pose_x = hole_position.x - deviation;
            init_pose_y = hole_position.y;
            init_pose_z = hole_position.z;
        }
        // double hole_pose_x = hole_position.x-0.01;
        double hole_pose_x = hole_position.x + deviation;
        double hole_pose_y = hole_position.y;
        double hole_pose_z = hole_position.z;

        /****************************************************
        *      Add the collision object to the scene       *
        ***************************************************/
        auto const collision_object_cube = create_collision(cup_pose, "coffee_" + std::to_string(i), object_dimensions[0], object_dimensions[1]);
        auto const collision_object_plate = create_collision(plate_center, "plate", 0.1, 0.175);

        moveit::planning_interface::PlanningSceneInterface coffee;
        moveit::planning_interface::PlanningSceneInterface robot_plate;

        coffee.applyCollisionObject(collision_object_cube);
        robot_plate.applyCollisionObject(collision_object_plate);

        RCLCPP_INFO(LOGGER, "Create Collision");
        feedback->state = "Create Collision";
        goal_handle->publish_feedback(feedback);
        /****************************************************
        *                Close gripper                     *
        ***************************************************/
        RCLCPP_INFO(LOGGER, "Close Gripper!");
        feedback->state = "Close Gripper!";
        goal_handle->publish_feedback(feedback);

        move_group_gripper->setNamedTarget("close");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        bool success_gripper = (move_group_gripper->plan(my_plan_gripper) ==
                                moveit::core::MoveItErrorCode::SUCCESS);

        move_group_gripper->execute(my_plan_gripper);

        /****************************************************
        *                 Pregrasp                         *
        ***************************************************/
        RCLCPP_INFO(LOGGER, "Pregrasp position");
        feedback->state = "Pregrasp position";
        goal_handle->publish_feedback(feedback);

        joint_group_positions_arm[0] = 1.06;  // Shoulder Pan
        joint_group_positions_arm[1] = -1.07; // Shoulder Lift
        joint_group_positions_arm[2] = 0.700;  // Elbow
        joint_group_positions_arm[3] = -1.202; // Wrist 1
        joint_group_positions_arm[4] = -1.57; // Wrist 2
        joint_group_positions_arm[5] = -0.514;  // Wrist 3

        move_group_arm->setJointValueTarget(joint_group_positions_arm);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group_arm->plan(my_plan_arm) ==
                            moveit::core::MoveItErrorCode::SUCCESS);

        move_group_arm->execute(my_plan_arm);


        /****************************************************
        *              Grasp pose define                   *
        ***************************************************/
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.orientation.x = 1.0;
        target_pose1.orientation.y = 0.0;
        target_pose1.orientation.z = 0.0;
        target_pose1.orientation.w = 0.0;

        target_pose1.position.x = cup_pose.x;
        target_pose1.position.y = cup_pose.y;
        target_pose1.position.z = 0.35;
        
        current_state_arm = move_group_arm->getCurrentState(10);
        current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                    joint_group_positions_arm);

        /****************************************************
        *                Open gripper                     *
        ***************************************************/
        feedback->state = "Open Gripper!";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(LOGGER, "Open Gripper!");

        move_group_gripper->setNamedTarget("open");

        success_gripper = (move_group_gripper->plan(my_plan_gripper) ==
                                moveit::core::MoveItErrorCode::SUCCESS);

        move_group_gripper->execute(my_plan_gripper);

        /****************************************************
        *           Approach to object waypoint            *
        ***************************************************/
        RCLCPP_INFO(LOGGER, "Approach to object!");
        feedback->state = "Approach to object!";
        goal_handle->publish_feedback(feedback);

        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
        target_pose1.position.z -= Approach;
        approach_waypoints.push_back(target_pose1);

        target_pose1.position.z -= Approach;
        approach_waypoints.push_back(target_pose1);

        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = move_group_arm->computeCartesianPath(
            approach_waypoints, eef_step, jump_threshold, trajectory_approach);

        move_group_arm->execute(trajectory_approach);
        
        /*******************************************************
        *  Attached Collision Object to the End Effector Link *
        ******************************************************/
        RCLCPP_INFO(LOGGER, "Attached Collision Object to the End Effector Link");
        feedback->state = "Attached Collision Object to the End Effector Link";
        goal_handle->publish_feedback(feedback);

        static const std::string END_EFFECTOR_LINK = move_group_arm->getEndEffectorLink();

        moveit_msgs::msg::AttachedCollisionObject attached_object;  // attach object
        attached_object.link_name = END_EFFECTOR_LINK;
        attached_object.object = collision_object_cube;  
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        attached_object.touch_links = std::vector<std::string>{"rg2_gripper_left_finger", "rg2_gripper_left_thumb", "rg2_gripper_right_finger", "rg2_gripper_right_thumb"}; // Specify touch links

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //add the attached object to the scene
        
        bool result = planning_scene_interface.applyAttachedCollisionObject(attached_object);
            if (!result)
        {
            RCLCPP_ERROR(LOGGER, "Failed to add AttachCollisionObject: %s", attached_object.object.id.c_str());

            //action feedback
            std::stringstream ss;
            ss << "Failed to add AttachCollisionObject: " << attached_object.object.id.c_str();
            std::string s = ss.str();
            feedback->state = s;
            goal_handle->publish_feedback(feedback);
        }

        /****************************************************
        *                 Take coffee                      *
        ***************************************************/

        RCLCPP_INFO(LOGGER, "Take coffee");
        feedback->state = "Take coffee";
        goal_handle->publish_feedback(feedback);

        move_group_gripper->setNamedTarget("hold");
        success_gripper = (move_group_gripper->plan(my_plan_gripper) ==
                                moveit::core::MoveItErrorCode::SUCCESS);

        move_group_gripper->execute(my_plan_gripper);

        /****************************************************
        *              Pick coffee up                      *
        ***************************************************/

        RCLCPP_INFO(LOGGER, "Pick coffee up");
        feedback->state = "Pick coffee up";
        goal_handle->publish_feedback(feedback);

        std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
        target_pose1.position.z += Retrieve;
        retreat_waypoints.push_back(target_pose1);

        target_pose1.position.z += Retrieve;
        retreat_waypoints.push_back(target_pose1);
        moveit_msgs::msg::RobotTrajectory trajectory_retreat;

        fraction = move_group_arm->computeCartesianPath(
            retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

        move_group_arm->execute(trajectory_retreat);

        /********************************************************
        * Pose X and Y from one hole set as reference to move  *
        *******************************************************/
        RCLCPP_INFO(LOGGER, "Position reference");
        feedback->state = "To initial hole X Y";
        goal_handle->publish_feedback(feedback);

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

        fraction = move_group_arm->computeCartesianPath(
            hole_waypoints, eef_step, jump_threshold, hole_trajectory_approach, test_constraints);

        move_group_arm->execute(hole_trajectory_approach);


        /****************************************************
        *           Initail pose to desire place           *
        ***************************************************/

        if(init_pose_x != hole_pose_x || init_pose_y != hole_pose_y ){
            RCLCPP_INFO(LOGGER, "Initial to desire x, y");
            feedback->state = "From intial hole X Y to desire hole";
            goal_handle->publish_feedback(feedback);
            
            std::vector<geometry_msgs::msg::Pose> desire_waypoints;

            target_pose2.position.y += hole_pose_y - init_pose_y;
            target_pose2.position.x += hole_pose_x - init_pose_x;
            desire_waypoints.push_back(target_pose2);

            moveit_msgs::msg::RobotTrajectory desire_trajectory_approach;

            fraction = move_group_arm->computeCartesianPath(
                desire_waypoints, eef_step, jump_threshold, desire_trajectory_approach);
            
            RCLCPP_INFO(LOGGER, "Fraction = %f ", fraction);

            move_group_arm->execute(desire_trajectory_approach);

        }

        /****************************************************
        *             Z pose to approach hole              *
        ***************************************************/

        RCLCPP_INFO(LOGGER, "Z pose to approach hole ");
        feedback->state = "Z pose to approach hole";
        goal_handle->publish_feedback(feedback);

        std::vector<geometry_msgs::msg::Pose> z_approach_waypoints;
        target_pose2.position.z -= z_approach/2.0;
        z_approach_waypoints.push_back(target_pose2);

        target_pose2.position.z -= z_approach/2.0;
        z_approach_waypoints.push_back(target_pose2);

        moveit_msgs::msg::RobotTrajectory trajectory_z_approach;

        fraction = move_group_arm->computeCartesianPath(
            z_approach_waypoints, eef_step, jump_threshold, trajectory_z_approach);

        move_group_arm->execute(trajectory_z_approach);


        /****************************************************
        *                Open gripper                     *
        ***************************************************/
        RCLCPP_INFO(LOGGER, "Open Gripper!");
        feedback->state = "Open Gripper!";
        goal_handle->publish_feedback(feedback);

        move_group_gripper->setNamedTarget("open");

        success_gripper = (move_group_gripper->plan(my_plan_gripper) ==
                                moveit::core::MoveItErrorCode::SUCCESS);

        move_group_gripper->execute(my_plan_gripper);

        /****************************************************
        *                Detach object                     *
        ***************************************************/
        RCLCPP_INFO(LOGGER, "Detach Collision Object ");
        feedback->state = "Detach Collision Object";
        goal_handle->publish_feedback(feedback);

        moveit_msgs::msg::AttachedCollisionObject detach_object;
        detach_object.link_name = END_EFFECTOR_LINK;
        detach_object.object.id = collision_object_cube.id;
        detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        result = planning_scene_interface.applyAttachedCollisionObject(detach_object);
        if (!result)
        {
            RCLCPP_ERROR(LOGGER, "Failed to detach AttachCollisionObject object: %s", detach_object.object.id.c_str());
                        //action feedback
            std::stringstream ss;
            ss << "Failed to detach AttachCollisionObject object:" <<detach_object.object.id.c_str();
            std::string s = ss.str();
            feedback->state = s;
            goal_handle->publish_feedback(feedback);
        }

        /****************************************************
        *               Z axis Retrieve                    *
        ***************************************************/
        RCLCPP_INFO(LOGGER, "Z Retrieve from hole position");
        feedback->state = "Z Retrieve from hole position";
        goal_handle->publish_feedback(feedback);

        std::vector<geometry_msgs::msg::Pose> z_retrieve_waypoints;
        target_pose2.position.z += z_approach/2.0;
        z_retrieve_waypoints.push_back(target_pose2);

        target_pose2.position.z += z_approach/2.0;
        z_retrieve_waypoints.push_back(target_pose2);

        moveit_msgs::msg::RobotTrajectory trajectory_z_retrieve;

        fraction = move_group_arm->computeCartesianPath(
            z_retrieve_waypoints, eef_step, jump_threshold, trajectory_z_retrieve);

        move_group_arm->execute(trajectory_z_retrieve);  
    }

    moveit_msgs::msg::CollisionObject create_collision(geometry_msgs::msg::Point pose, std::string id,  float height, float radius){
            
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = move_group_arm->getPlanningFrame();
            collision_object.id = id;
            shape_msgs::msg::SolidPrimitive primitive;

            // Define the size of the box in meters
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.CYLINDER_HEIGHT] = height;
            primitive.dimensions[primitive.CYLINDER_RADIUS] = radius;

            // Define the pose of the box (relative to the frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0; // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
            box_pose.position.x = pose.x;
            box_pose.position.y = pose.y;
            box_pose.position.z = pose.z;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            return collision_object;
    
    }

 void goal_response_callback(const FindHoleGoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER2, "Goal was rejected by server");
    } else {
      RCLCPP_INFO(LOGGER2,
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(FindHoleGoalHandle::SharedPtr,
                         const std::shared_ptr<const FindHolesAction::Feedback> feedback) {
        if (feedback && !feedback->process_description.empty()) {
        // Log or process each feedback process description
        for (const auto& description : feedback->process_description) {
            RCLCPP_INFO(LOGGER2, "Feedback: %s", description.c_str());
        }
        } else {
            RCLCPP_INFO(LOGGER2, "Feedback received but it's empty or null");
        }
  }

  void result_callback(const FindHoleGoalHandle::WrappedResult &result) {
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(LOGGER2, "Goal was aborted");
        return;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(LOGGER2, "Goal was canceled");
        return;
        default:
        RCLCPP_ERROR(LOGGER2, "Unknown result code");
        return;
        }
        for (size_t i = 0; i < result.result->hole_position.size(); ++i) {
            hole_position.push_back(result.result->hole_position[i]);
        }
        plate_center = result.result->plate_center;
        RCLCPP_INFO(LOGGER2, "Result received");
        receive = true;
  }

};
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Pick_And_Place::MoveitAction)

