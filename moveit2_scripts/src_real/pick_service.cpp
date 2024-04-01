#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <sstream>
#include "custom_interfaces/action/pick_and_place.hpp"
#include "custom_interfaces/srv/activate_action.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <chrono>

namespace Pick_And_Place {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick and place service");
static const rclcpp::Logger LOGGER2 = rclcpp::get_logger("Perception action in moveit action");

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class MoveitService : public rclcpp::Node 
{
  using PickAndPlaceAction =
      custom_interfaces::action::PickAndPlace;
  using PickAndPlaceGoalHandle =
      rclcpp_action::ClientGoalHandle<PickAndPlaceAction>;
public:
  explicit MoveitService(const rclcpp::NodeOptions &options)
      : rclcpp::Node("pick_and_place_service", options), goal_done_(false){   
    srv_ = create_service<custom_interfaces::srv::ActivateAction>("/activate_action",std::bind(&MoveitService::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    pub_ = this->create_publisher<std_msgs::msg::String>("Action_feedback",10);
    pub_result = this->create_publisher<std_msgs::msg::String>("Action_result",10);

    client_ptr_ = rclcpp_action::create_client<PickAndPlaceAction>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "pick_and_place");

}

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal(int cup_number) {
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

    auto goal_msg = PickAndPlaceAction::Goal();
    goal_msg.cup_number= cup_number;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<PickAndPlaceAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MoveitService::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MoveitService::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MoveitService::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

  }

private:
    rclcpp_action::Client<PickAndPlaceAction>::SharedPtr client_ptr_;
    rclcpp::Service<custom_interfaces::srv::ActivateAction>::SharedPtr srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_result;

    rclcpp::TimerBase::SharedPtr response_timer_;
    bool goal_done_;

 void service_callback(const std::shared_ptr<custom_interfaces::srv::ActivateAction::Request> req, 
        const std::shared_ptr<custom_interfaces::srv::ActivateAction::Response> res){

        send_goal(req->cup_number);
        res->result = "processing";
}
   
 void goal_response_callback(const PickAndPlaceGoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER2, "Goal was rejected by server");
    } else {
      RCLCPP_INFO(LOGGER2,
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(PickAndPlaceGoalHandle::SharedPtr,
                         const std::shared_ptr<const PickAndPlaceAction::Feedback> feedback) {
        if (feedback && !feedback->state.empty()) {
        // Log or process each feedback process description
            RCLCPP_INFO(LOGGER2, "Feedback: %s", feedback->state.c_str());
            std_msgs::msg::String msg;
            msg.data = feedback->state;
            pub_->publish(msg);

        } else {
            RCLCPP_INFO(LOGGER2, "Feedback received but it's empty or null");
            std_msgs::msg::String msg;
            msg.data = "Feedback received but it's empty or null";
            pub_->publish(msg);        
        }
  }

  void result_callback(const PickAndPlaceGoalHandle::WrappedResult &result) {
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
        RCLCPP_INFO(LOGGER2, "Result received");
        std_msgs::msg::String msg;
        msg.data = result.result->result;
        pub_result->publish(msg);        
  }

};
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Pick_And_Place::MoveitService)

