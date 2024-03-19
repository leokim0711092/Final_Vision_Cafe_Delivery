#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene/planning_scene.h>
#include <map>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_task");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  Eigen::Isometry3d vectorToEigen(const std::vector<double>& values); 
  geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values); 

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  std::map<std::string, std::string> params;
  std::vector<double> grasp_frame_transform;
  std::vector<double> fill_pose;

  rclcpp::Node::SharedPtr node_;
};


MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
    std::vector<std::string> param_names = {
        "arm_group_name",
        "hand_name",
        "eef_name",
        "grasp_frame_name",
    };

    node_->declare_parameter<std::vector<double>>("grasp_frame_transform", std::vector<double>());
    node_->get_parameter("grasp_frame_transform", grasp_frame_transform);
    
    node_->declare_parameter<std::vector<double>>("fill_pose", std::vector<double>());
    node_->get_parameter("fill_pose", fill_pose);

    // // Fetch all parameters
    for (const auto& name : param_names) {
        node_->declare_parameter<std::string>(name);
        params[name] = node_->get_parameter(name).as_string();
    }

    for (const auto& param : params) {
        RCLCPP_INFO(LOGGER, "Param %s: %s", param.first.c_str(), param.second.c_str());
    }


}

Eigen::Isometry3d MTCTaskNode::vectorToEigen(const std::vector<double>& values){
    return Eigen::Translation3d(values[0], values[1], values[2]) *
        Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

geometry_msgs::msg::Pose MTCTaskNode::vectorToPose(const std::vector<double>& values){
    return tf2::toMsg(vectorToEigen(values));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::doTask()
{
  auto task = createTask();

  try
  {
    task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task.plan(5))
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed");
    return;
  }
  task.introspection().publishSolution(*task.solutions().front());

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Task execution failed");
    return;
  }
}

mtc::Task MTCTaskNode::createTask()
{

   mtc::Task task;

    task.stages()->setName("Coffee PnP");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "ur_manipulator";
    const auto& gripper_group_name = "gripper";
    const auto& gripper_frame = "tool0"; 

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", gripper_group_name); 
    task.setProperty("hand", gripper_group_name);
	task.setProperty("hand_grasping_frame", gripper_frame);
    task.setProperty("ik_frame", gripper_frame); 

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-4);
    //sampling_planner->setProperty("multi_query_planning_enabled", true);
    //sampling_planner->setPlannerId("geometric::LBKPIECE");
    
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.01);
    cartesian_planner->setJumpThreshold(1.5);
    
    // Constraints 
    moveit_msgs::msg::OrientationConstraint gc;
    gc.link_name = "tool0"; 
    gc.header.frame_id = "world";
    gc.orientation.x = 0.0; // 
    gc.orientation.y = 0.0;
    gc.orientation.z = 0.0;
    gc.orientation.w = 1.0; 
    gc.absolute_x_axis_tolerance = 0.68;
    gc.absolute_y_axis_tolerance = 0.68;
    gc.absolute_z_axis_tolerance = 3.15;
    gc.weight = 1.0;

    moveit_msgs::msg::Constraints grasp_constraint;
    grasp_constraint.orientation_constraints.push_back(gc);

    // ***STAGES***
    // -> Current State Pointer ->
    mtc::Stage* current_state_ = nullptr;
    {
		auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object = "coffee"](const mtc::SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		task.add(std::move(applicability_filter));
	}
    // ADD TO READY POSITION?
    // ***Close Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("close");       
        task.add(std::move(stage));    
    }

    // ***Open Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("open"); 
        // -> Set Current State Pointer ->
        current_state_ = stage.get();
        task.add(std::move(stage));
    }

    // ***Move to Pre-Grasp Position <Connector>***
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
        "pre-grasp position", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }
    // -> Grasp Stage Pointer ->
    mtc::Stage* grasp_stage_ = nullptr;
    
    // ***Initial Grasp Container***
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("initial grasp");
        // Pass Properties from Task to Container
        task.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
        
        // ***Approach Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach cup");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.03, 0.1);

             // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // ***Grasp Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate initial grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "initial_grasp_pose"); 
            stage->setPreGraspPose("open");
            stage->setObject("coffee");
            stage->setAngleDelta(M_PI / 16); 
            stage->setMonitoredStage(current_state_);
            // stage->setPathConstraints(grasp_constraint);
            
            // ***Compute IK <Wrapper>***
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("initial grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame); // Pose and Frame
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }

        // ***Allow Collision <PlanningScene>*** 
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
            stage->allowCollisions(
                "coffee", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }    
        


        // ***Attach Cup <PlanningScene>***  
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
            stage->attachObject("coffee", gripper_frame);
            grasp->insert(std::move(stage));
        }

        // *** Lift Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.02, 0.1); 
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "lift_cup");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
    
            grasp->insert(std::move(stage));
        }
        // ***Close Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("close");       
            grasp->insert(std::move(stage));    
        }
        // -> Set Grasp Stage Pointer ->
        grasp_stage_ = grasp.get();
        task.add(std::move(grasp));
    } // END INITIAL GRASP CONTAINER

    // *** Move to Coffee Machine <Connector>***
    {
        auto stage = std::make_unique<mtc::stages::Connect>("move to coffee machine", 
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
                                                    //{gripper_group_name, sampling_planner}
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }   
    // ***Fill Coffee Container***
    // -> Fill Stage Pointer ->
    mtc::Stage* fill_stage_ = nullptr;
    {
        auto fill = std::make_unique<mtc::SerialContainer>("fill coffee");
        task.properties().exposeTo(fill->properties(), {"eef", "hand", "group", "ik_frame"});
        fill->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
        // *** Move to Coffee Machine <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("fill cup", cartesian_planner);
            stage->properties().set("marker_ns", "fill_cup");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.01, 0.15);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = 1.0; // Set to 1.0 to use range
            stage->setDirection(vec);
            fill->insert(std::move(stage));
        }
        // *** Fill Coffee Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate fill pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "fill_pose");
            stage->setObject("coffee");
            // Define Pose 
            geometry_msgs::msg::PoseStamped fill_pose_msg;
            fill_pose_msg.header.frame_id = "world";
            fill_pose_msg.pose =vectorToPose(fill_pose);
            stage->setPose(fill_pose_msg);
            stage->setMonitoredStage(grasp_stage_);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("fill pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("coffee");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            fill->insert(std::move(wrapper));
        }

        // ***Retract from Coffee Machine <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retract coffee1", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.2);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "retract_coffee1");
            // Set Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.x = -0.05;
            vec.vector.y = -0.1;
            //vec.vector.z = fill_retract;
            stage->setDirection(vec);
            // Get Pose for next stage TODO: FIX FOR USE
            //current_pose = moveit::planning_interface::MoveGroupInterface::getCurrentPose("tool0");
            fill->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retract coffee2", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.2);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "retract_coffee2");
            // Set Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.x = 0.05;
            vec.vector.y = -0.06;
            //vec.vector.z = fill_retract;
            stage->setDirection(vec);
            // Get Pose for next stage TODO: FIX FOR USE
            //current_pose = moveit::planning_interface::MoveGroupInterface::getCurrentPose("tool0");
            fill->insert(std::move(stage));
        }
        
        
        // ***Lower Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cup", cartesian_planner);
            stage->properties().set("marker_ns", "lower_cup");
            //stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            //stage->setMinMaxDistance(0.05, 0.15);

            // Set down direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = -0.09; // ADJUST IF NEEDED 
            stage->setDirection(vec);
            fill->insert(std::move(stage));
        }

        // ***Open Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open");
            fill->insert(std::move(stage));
        }

        // ***Disable Collision <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collision");
            stage->allowCollisions("coffee", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                false); 
            fill->insert(std::move(stage));
        }

        // ***Detach Cup <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cup");
            stage->detachObject("coffee", gripper_frame);
            fill->insert(std::move(stage));
        }

        // ***Retreat from Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from cup", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.15);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "retreat");

            //Set Retreat Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 0.1;
            stage->setDirection(vec);
            fill->insert(std::move(stage));
        }
        // -> Set Fill Stage Pointer ->
        fill_stage_ = fill.get();
        task.add(std::move(fill));
    } // END FILL COFFEE CONTAINER

    // // ***Move to Regrasp <Connector>***
    // {
    //     auto stage = std::make_unique<mtc::stages::Connect>(
    //     "move to regrasp", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
    //     stage->setTimeout(15.0);
    //     stage->properties().configureInitFrom(mtc::Stage::PARENT);
    //     task.add(std::move(stage));
    // }
    
    // // -> Regrasp Stage Pointer ->
    // // mtc::Stage* regrasp_stage_ = nullptr;
    // // ***Regrasp Container***
    // {
    //     auto regrasp = std::make_unique<mtc::SerialContainer>("regrasp cup");
    //     task.properties().exposeTo(regrasp->properties(), {"eef", "hand", "group", "ik_frame"});
    //     regrasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

    //     // ***Approach Cup <MoveRelative>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveRelative>("approach regrasp", cartesian_planner);
    //         stage->properties().set("marker_ns", "approach regrasp");
    //         stage->properties().set("link", gripper_frame);
    //         stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    //         stage->setMinMaxDistance(0.02, 0.15);

    //         // Set Direction
    //         geometry_msgs::msg::Vector3Stamped vec;
    //         vec.header.frame_id = "world";
    //         vec.vector.z = regrasp_approach; // ADJUST
    //         stage->setDirection(vec);
    //         regrasp->insert(std::move(stage));
    //     }

    //     // ***Generate Grasp Pose <Generator>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate regrasp pose");
    //         stage->properties().configureInitFrom(mtc::Stage::PARENT);
	// 		stage->properties().set("marker_ns", "regrasp_pose");
	// 		stage->setPreGraspPose("open");
	// 		stage->setObject("cup");
	// 		stage->setAngleDelta(M_PI / 16);
	// 		stage->setMonitoredStage(fill_stage_);

    //         // ***Compute IK***
    //         auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp pose IK", std::move(stage));
	// 		wrapper->setMaxIKSolutions(20);
	// 		wrapper->setMinSolutionDistance(1.0);
	// 		wrapper->setIKFrame(toEigen(regrasp_frame_transform), gripper_frame);
	// 		wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
	// 		wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
	// 		regrasp->insert(std::move(wrapper)); 
    //     }

    //     // *** Allow Collision <PlanningScene>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
	// 		stage->allowCollisions(
	// 		    "cup",
	// 		    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
	// 		    true);
	// 		regrasp->insert(std::move(stage));
    //     }
       
    //     // ***Close Gripper <MoveTo>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
	// 		stage->setGroup(gripper_group_name);
	// 		stage->setGoal("close");
	// 		regrasp->insert(std::move(stage));    
    //     }

    //     // ***Attach Cup <PlanningScene>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
	// 		stage->attachObject("cup", gripper_frame);
	// 		regrasp->insert(std::move(stage));
    //     }


    //     // ***Lift Cup <MoveRelative>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
	// 		stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
	// 		//stage->setMinMaxDistance(0.05, 0.15);
	// 		stage->setIKFrame(gripper_frame);
	// 		stage->properties().set("marker_ns", "lift_cup");

	// 		// Set direction
	// 		geometry_msgs::msg::Vector3Stamped vec;
	// 		vec.header.frame_id = "world";
	// 		vec.vector.z = regrasp_retract;
	// 		stage->setDirection(vec);
	// 		regrasp->insert(std::move(stage));
    //     }
        
    //     // ***Move to Pre-Serve Position <MoveRelative>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveRelative>("pre-serve", cartesian_planner);
	// 		stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
	// 		//stage->setMinMaxDistance(0.05, 0.15);
	// 		stage->setIKFrame(gripper_frame);
	// 		stage->properties().set("marker_ns", "lift_cup");

	// 		// Set direction
	// 		geometry_msgs::msg::Vector3Stamped vec;
	// 		vec.header.frame_id = "world";
	// 		vec.vector.x = preserveX;
    //         vec.vector.y = preserveY;
	// 		stage->setDirection(vec);
	// 		regrasp->insert(std::move(stage));
    //     }

    //     // ***Serve to Baristabot <MoveRelative>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
	// 		stage->properties().set("marker_ns", "serve_cup");
    //         stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
	// 		//stage->setMinMaxDistance(0.05, 0.15);
			
	// 		// Set direction
	// 		geometry_msgs::msg::Vector3Stamped vec;
	// 		vec.header.frame_id = "world";
	// 		vec.vector.z = serve_approach; 
	// 		stage->setDirection(vec);
	// 		regrasp->insert(std::move(stage));
    //     }   

    //     //  ***Open Gripper <MoveTo>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    //         stage->setGroup(gripper_group_name);
    //         stage->setGoal("open");       
    //         regrasp->insert(std::move(stage));    
    //     }

    //     // ***Disable Collision <PlanningScene>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collision");
    //         stage->allowCollisions("cup", 
    //             task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
    //             false); 
    //         regrasp->insert(std::move(stage));
    //     }
        
    //     // ***Detach Cup <PlanningScene>***
    //     {
    //         auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cup");
    //         stage->detachObject("cup", gripper_frame);
    //         regrasp->insert(std::move(stage));
    //     }

    //     // ***Retract from Serve Pose
    //     {
    //         auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
	// 		stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
	// 		//stage->setMinMaxDistance(0.05, 0.15);
	// 		stage->setIKFrame(gripper_frame);
	// 		stage->properties().set("marker_ns", "serve_cup");

	// 		// Set direction
	// 		geometry_msgs::msg::Vector3Stamped vec;
	// 		vec.header.frame_id = "world";
	// 		vec.vector.z = serve_retract; 
	// 		stage->setDirection(vec);
	// 		regrasp->insert(std::move(stage));
    //     }  
    //     // -> Set Regrasp Stage Pointer ->
    //     //regrasp_stage_ = regrasp.get(); 

    //     task.add(std::move(regrasp));
    // } // END REGRASP CONTAINER

    return task;
}

//   mtc::Task task;
//   task.stages()->setName("open_gripper_task");
//   task.loadRobotModel(node_);

//   // Sampling planner
//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  
//   auto joint_interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   // Cartesian planner
//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(1.0);
//   cartesian_planner->setMaxAccelerationScalingFactor(1.0);
//   cartesian_planner->setStepSize(.1);
//   cartesian_planner->setMinFraction(0.00);
  
//   // Configure the gripper group name according to your setup
//   const std::string arm_group_name = params["arm_group_name"];
//   const std::string gripper_group_name = params["hand_name"];

//   const std::string eef_name = params["eef_name"];
  
//   const std::string grasp_frame_name = params["grasp_frame_name"];


//   // Set task properties
//   task.setProperty("group", arm_group_name);
//   task.setProperty("hand", gripper_group_name);
//   task.setProperty("eef", eef_name );
//   task.setProperty("hand_grasping_frame", grasp_frame_name);
//   task.setProperty("ik_frame", grasp_frame_name);
  // Start state
//   auto start_state = std::make_unique<mtc::stages::CurrentState>("current state");
//   task.add(std::move(start_state));

	// auto scene = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
	// {
	// 	auto& state = scene->getCurrentStateNonConst();
	// 	state.setToDefaultValues(state.getJointModelGroup(arm_group_name), "ready");

	// 	auto fixed = std::make_unique<mtc::stages::FixedState>("initial state");
	// 	fixed->setState(scene);
	// 	task.add(std::move(fixed));
	// }

	// {
	// 	auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

	// 	// Verify that object is not attached
	// 	auto applicability_filter =
	// 	    std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
	// 	applicability_filter->setPredicate([object = "coffee"](const mtc::SolutionBase& s, std::string& comment) {
	// 		if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
    //             comment = std::string("object with id '") + object + "' is already attached and cannot be picked";
	// 			return false;
	// 		}
	// 		return true;
	// 	});
	// 	task.add(std::move(applicability_filter));
	// }

    // /****************************************************
	//  *                                                  *
	//  *               Close gripper                       *
	//  *                                                  *
	//  ***************************************************/
    // mtc::Stage* initial_state_ptr = nullptr;
    // {
    //     // close gripper stage
    //     auto close_gripper = std::make_unique<mtc::stages::MoveTo>("close gripper", std::make_shared<mtc::solvers::JointInterpolationPlanner>());
    //     close_gripper->setGroup(gripper_group_name);
    //     close_gripper->setGoal("close");
    //     initial_state_ptr = close_gripper.get();  // remember start state for monitoring grasp pose generator
    //     task.add(std::move(close_gripper));
    // }
    
    // /****************************************************
	//  *                                                  *
	//  *               Move to Pick                       *
	//  *                                                  *
	//  ***************************************************/
	// {  // Move-to pre-grasp
	// 	auto stage = std::make_unique<mtc::stages::Connect>(
	// 	    "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
	// 	stage->setTimeout(5.0);
	// 	stage->properties().configureInitFrom(mtc::Stage::PARENT);
	// 	task.add(std::move(stage));
	// }

// 	/****************************************************
// 	 *                                                  *
// 	 *               Pick Object                        *
// 	 *                                                  *
// 	 ***************************************************/
// 	mtc::Stage* pick_stage_ptr = nullptr;
// 	{
// 		auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
// 		task.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
// 		grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "hand", "group", "ik_frame" });

// 		/****************************************************
//   ---- *               Approach Object                    *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
// 			stage->properties().set("marker_ns", "approach_object");
// 			stage->properties().set("link", grasp_frame_name);
// 			stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
// 			stage->setMinMaxDistance(0.2, 0.5);

// 			// Set hand forward direction
// 			geometry_msgs::msg::Vector3Stamped vec;
// 			vec.header.frame_id = grasp_frame_name;
// 			vec.vector.z = 1.0;
// 			stage->setDirection(vec);
// 			grasp->insert(std::move(stage));
// 		}

// 		/****************************************************
//   ---- *               Generate Grasp Pose                *
// 		 ***************************************************/
// 		{
// 			// Sample grasp pose
// 			auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
// 			stage->properties().configureInitFrom(mtc::Stage::PARENT);
// 			stage->properties().set("marker_ns", "grasp_pose");
// 			stage->setPreGraspPose("open"); ///config
// 			stage->setObject("coffee"); ///config
// 			stage->setAngleDelta(M_PI / 12);
// 			stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

// 			// Compute IK
// 			auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
// 			wrapper->setMaxIKSolutions(8);
// 			wrapper->setMinSolutionDistance(1.0);
// 			wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), grasp_frame_name ); /// config
// 			wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
// 			wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
// 			grasp->insert(std::move(wrapper));
// 		}

// 		/****************************************************
//   ---- *               Allow Collision (hand object)   *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
// 			stage->allowCollisions(
// 			    "coffee",
// 			    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
// 			    true);    /// config
// 			grasp->insert(std::move(stage));
// 		}

// 		/****************************************************
//   ---- *               Hold Hand                      *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
// 			stage->setGroup(gripper_group_name);
// 			stage->setGoal("close"); ///config
// 			grasp->insert(std::move(stage));
// 		}

// 		/****************************************************
//   .... *               Attach Object                      *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
// 			stage->attachObject("coffee", grasp_frame_name);  ///config
// 			grasp->insert(std::move(stage));
// 		}

// 		/****************************************************
//   .... *               Allow collision (object support)   *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
// 			stage->allowCollisions("coffee" , "world", true);
// 			grasp->insert(std::move(stage));
// 		}

// 		/****************************************************
//   .... *               Lift object                        *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
// 			stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
// 			stage->setMinMaxDistance(0.1, 0.15);
// 			stage->setIKFrame(grasp_frame_name);
// 			stage->properties().set("marker_ns", "lift_object");

// 			// Set upward direction
// 			geometry_msgs::msg::Vector3Stamped vec;
// 			vec.header.frame_id = "world";
// 			vec.vector.z = 1.0;
// 			stage->setDirection(vec);
// 			grasp->insert(std::move(stage));
// 		}

// 		/****************************************************
//   .... *               Forbid collision (object support)  *
// 		 ***************************************************/
// 		{
// 			auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
// 			stage->allowCollisions("coffee" , "world" , false);
// 			grasp->insert(std::move(stage));
// 		}

	// 	pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

	// 	// Add grasp container to task
	// 	task.add(std::move(grasp));
	// }

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
// 	{
// 		auto stage = std::make_unique<stages::Connect>(
// 		    "move to place", stages::Connect::GroupPlannerVector{ { params.arm_group_name, sampling_planner } });
// 		stage->setTimeout(5.0);
// 		stage->properties().configureInitFrom(Stage::PARENT);
// 		t.add(std::move(stage));
// 	}

// 	/******************************************************
// 	 *                                                    *
// 	 *          Place Object                              *
// 	 *                                                    *
// 	 *****************************************************/
// 	{
// 		auto place = std::make_unique<SerialContainer>("place object");
// 		t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
// 		place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

// 		/******************************************************
//   ---- *          Lower Object                              *
// 		 *****************************************************/
// 		{
// 			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
// 			stage->properties().set("marker_ns", "lower_object");
// 			stage->properties().set("link", params.hand_frame);
// 			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 			stage->setMinMaxDistance(.03, .13);

// 			// Set downward direction
// 			geometry_msgs::msg::Vector3Stamped vec;
// 			vec.header.frame_id = params.world_frame;
// 			vec.vector.z = -1.0;
// 			stage->setDirection(vec);
// 			place->insert(std::move(stage));
// 		}

// 		/******************************************************
//   ---- *          Generate Place Pose                       *
// 		 *****************************************************/
// 		{
// 			// Generate Place Pose
// 			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
// 			stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
// 			stage->properties().set("marker_ns", "place_pose");
// 			stage->setObject(params.object_name);

// 			// Set target pose
// 			geometry_msgs::msg::PoseStamped p;
// 			p.header.frame_id = params.object_reference_frame;
// 			p.pose = vectorToPose(params.place_pose);
// 			p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
// 			stage->setPose(p);
// 			stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

// 			// Compute IK
// 			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
// 			wrapper->setMaxIKSolutions(2);
// 			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
// 			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
// 			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
// 			place->insert(std::move(wrapper));
// 		}

// 		/******************************************************
//   ---- *          Open Hand                              *
// 		 *****************************************************/
// 		{
// 			auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
// 			stage->setGroup(params.hand_group_name);
// 			stage->setGoal(params.hand_open_pose);
// 			place->insert(std::move(stage));
// 		}

// 		/******************************************************
//   ---- *          Forbid collision (hand, object)        *
// 		 *****************************************************/
// 		{
// 			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
// 			stage->allowCollisions(params.object_name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name),
// 			                       false);
// 			place->insert(std::move(stage));
// 		}

// 		/******************************************************
//   ---- *          Detach Object                             *
// 		 *****************************************************/
// 		{
// 			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
// 			stage->detachObject(params.object_name, params.hand_frame);
// 			place->insert(std::move(stage));
// 		}

// 		/******************************************************
//   ---- *          Retreat Motion                            *
// 		 *****************************************************/
// 		{
// 			auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
// 			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 			stage->setMinMaxDistance(.12, .25);
// 			stage->setIKFrame(params.hand_frame);
// 			stage->properties().set("marker_ns", "retreat");
// 			geometry_msgs::msg::Vector3Stamped vec;
// 			vec.header.frame_id = params.hand_frame;
// 			vec.vector.z = -1.0;
// 			stage->setDirection(vec);
// 			place->insert(std::move(stage));
// 		}

// 		// Add place container to task
// 		t.add(std::move(place));
// 	}

    // // /****************************************************
    // //  *                                                  *
    // //  *               Pregrasp Position                  *
    // //  *                                                  *
    // //  ***************************************************/
	// {
	// 	auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("x +0.2", cartesian_planner);
	// 	stage->setGroup(arm_group_name);
	// 	geometry_msgs::msg::Vector3Stamped direction;
	// 	direction.header.frame_id = "world";
	// 	// direction.vector.x = 0.2;
    //     // direction.vector.y = 0.3;
    //     direction.vector.z = 0.02;
	// 	stage->setDirection(direction);
	// 	task.add(std::move(stage));
	// }
    // // /****************************************************
	// //  *                                                  *
	// //  *               Open gripper                       *
	// //  *                                                  *
	// //  ***************************************************/
    // {
    //     // Open gripper stage
    //     auto open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", std::make_shared<mtc::solvers::JointInterpolationPlanner>());
    //     open_gripper->setGroup(gripper_group_name);
    //     open_gripper->setGoal("open");
    //     task.add(std::move(open_gripper));
    // }

//   return task;
// }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  MTCTaskNode mtc_task_node(options); 

  mtc_task_node.doTask();
  // Keep introspection alive
  
  rclcpp::shutdown();
  return 0;
}
