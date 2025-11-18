
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>

#include <fstream>
#include <sstream>
#include <chrono>
#include <string>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;
using NavigateAction = nav2_msgs::action::NavigateToPose;
using SpawnSrv = gazebo_msgs::srv::SpawnEntity;
using DeleteSrv = gazebo_msgs::srv::DeleteEntity;

enum class MissionState {
  SPAWN_BOX,
  NAV_TO_PICK,
  MONITOR_NAV_PICK,  
  MOCK_PICK,
  SPAWN_ATTACHED,
  NAV_TO_DROP,
  MONITOR_NAV_DROP,
  MOCK_DROP,
  SPAWN_DROP,
  NAV_HOME,
  MONITOR_NAV_HOME,
  DONE,
  FAILED,
  IDLE
};

class MissionPlanner : public rclcpp::Node
{
public:
  MissionPlanner()
  : Node("mission_planner_fake_pick"), state_(MissionState::SPAWN_BOX)
  {
    // Nav2 client
    nav_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");

    // Gazebo spawn/delete clients
    spawn_client_ = this->create_client<SpawnSrv>("/spawn_entity");
    delete_client_ = this->create_client<DeleteSrv>("/delete_entity");

    // Odom subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&MissionPlanner::odom_cb, this, std::placeholders::_1)
    );

    // Parameters
    this->declare_parameter<std::string>("box_sdf_path", "/home/nokia/sakar_bot/src/sakar_bot/models/box.sdf");
    this->declare_parameter<std::string>("box_model_name", "cargo_parcel");
    this->declare_parameter<std::string>("attached_model_name", "cargo_parcel_attached");
    this->declare_parameter<double>("pick_x", 4.4774699211120605);
    this->declare_parameter<double>("pick_y", 1.5526522397994995);
    this->declare_parameter<double>("drop_x", -3.905107259750366);
    this->declare_parameter<double>("drop_y", 0.8089179992675781);
    this->declare_parameter<double>("home_x", 0.0);
    this->declare_parameter<double>("home_y", 0.0);
    this->declare_parameter<double>("goal_tolerance", 0.5);  
    this->declare_parameter<double>("nav_timeout", 60.0);    

    box_sdf_path_ = this->get_parameter("box_sdf_path").as_string();
    box_model_name_ = this->get_parameter("box_model_name").as_string();
    attached_model_name_ = this->get_parameter("attached_model_name").as_string();
    pick_x_ = this->get_parameter("pick_x").as_double();
    pick_y_ = this->get_parameter("pick_y").as_double();
    drop_x_ = this->get_parameter("drop_x").as_double();
    drop_y_ = this->get_parameter("drop_y").as_double();
    home_x_ = this->get_parameter("home_x").as_double();
    home_y_ = this->get_parameter("home_y").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    nav_timeout_ = this->get_parameter("nav_timeout").as_double();

    RCLCPP_INFO(this->get_logger(), "MissionPlanner: box_sdf=%s", box_sdf_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pick:(%.2f,%.2f) Drop:(%.2f,%.2f) Home:(%.2f,%.2f)",
                pick_x_, pick_y_, drop_x_, drop_y_, home_x_, home_y_);
    RCLCPP_INFO(this->get_logger(), "Goal tolerance: %.2fm, Timeout: %.1fs", 
                goal_tolerance_, nav_timeout_);

    // Read SDF
    std::ifstream f(box_sdf_path_);
    if (f) {
      std::stringstream buf;
      buf << f.rdbuf();
      box_sdf_xml_ = buf.str();
      RCLCPP_INFO(this->get_logger(), "Loaded box sdf (%zu bytes)", box_sdf_xml_.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not open SDF at %s", box_sdf_path_.c_str());
    }

    // Start FSM loop
    timer_ = this->create_wall_timer(1000ms, std::bind(&MissionPlanner::step, this)); 
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<NavigateAction>::SharedPtr nav_client_;
  rclcpp::Client<SpawnSrv>::SharedPtr spawn_client_;
  rclcpp::Client<DeleteSrv>::SharedPtr delete_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_{nullptr};
  std::mutex odom_mutex_;

  MissionState state_;
  std::string box_sdf_path_;
  std::string box_sdf_xml_;
  std::string box_model_name_;
  std::string attached_model_name_;
  double pick_x_, pick_y_, drop_x_, drop_y_, home_x_, home_y_;
  double goal_tolerance_;
  double nav_timeout_;
  
  // Navigation state tracking
  rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr nav_goal_handle_;
  bool nav_in_progress_ = false;
  rclcpp::Time nav_start_time_;
  double current_goal_x_, current_goal_y_;

  geometry_msgs::msg::PoseStamped makePose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now();
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.w = 1.0;
    return p;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    latest_odom_ = msg;
  }

  double distanceToGoal(double goal_x, double goal_y)
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!latest_odom_) {
      return std::numeric_limits<double>::max();
    }
    
    double dx = goal_x - latest_odom_->pose.pose.position.x;
    double dy = goal_y - latest_odom_->pose.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
  }

  bool isGoalReached(double goal_x, double goal_y)
  {
    return distanceToGoal(goal_x, goal_y) <= goal_tolerance_;
  }

  void cancelCurrentNavigation()
  {
    if (nav_in_progress_ && nav_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling current navigation");
      auto cancel_future = nav_client_->async_cancel_goal(nav_goal_handle_);
      nav_in_progress_ = false;
    }
  }

  void startNavigation(double x, double y)
  {
    if (!nav_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Navigate action server not ready");
      return;
    }

    NavigateAction::Goal goal_msg;
    goal_msg.pose = makePose(x, y);
    current_goal_x_ = x;
    current_goal_y_ = y;

    auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](auto goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          nav_in_progress_ = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
          nav_goal_handle_ = goal_handle;
          nav_start_time_ = this->now();
        }
      };

    send_goal_options.result_callback =
      [this](const auto & result) {
        nav_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "Navigation action completed with result code: %d", 
                   static_cast<int>(result.code));
      };

    RCLCPP_INFO(this->get_logger(), "Sending navigation goal to (%.2f, %.2f)", x, y);
    nav_client_->async_send_goal(goal_msg, send_goal_options);
    nav_in_progress_ = true;
  }

  void spawnEntity(const std::string &name, double x, double y, double z, MissionState next_state)
  {
    if (!spawn_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "Spawn service not ready");
      return;
    }

    if (box_sdf_xml_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Box SDF not loaded");
      state_ = MissionState::FAILED;
      return;
    }

    auto req = std::make_shared<SpawnSrv::Request>();
    req->name = name;
    req->xml = box_sdf_xml_;
    req->initial_pose.position.x = x;
    req->initial_pose.position.y = y;
    req->initial_pose.position.z = z;

    auto result_callback = [this, name, next_state](rclcpp::Client<SpawnSrv>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", name.c_str());
          state_ = next_state;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to spawn %s: %s", name.c_str(), response->status_message.c_str());
          state_ = MissionState::FAILED;
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in spawn service: %s", e.what());
        state_ = MissionState::FAILED;
      }
    };

    RCLCPP_INFO(this->get_logger(), "Spawning %s at (%.2f, %.2f, %.2f)", name.c_str(), x, y, z);
    spawn_client_->async_send_request(req, result_callback);
  }

  void deleteEntity(const std::string &name, MissionState next_state)
  {
    if (!delete_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "Delete service not ready");
      return;
    }

    auto req = std::make_shared<DeleteSrv::Request>();
    req->name = name;

    auto result_callback = [this, name, next_state](rclcpp::Client<DeleteSrv>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Successfully deleted %s", name.c_str());
          state_ = next_state;
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to delete %s (continuing anyway): %s", 
                      name.c_str(), response->status_message.c_str());
          state_ = next_state;  
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in delete service: %s", e.what());
        state_ = next_state;  
      }
    };

    RCLCPP_INFO(this->get_logger(), "Deleting %s", name.c_str());
    delete_client_->async_send_request(req, result_callback);
  }

  void step()
  {
    switch (state_) {
      case MissionState::SPAWN_BOX:
        RCLCPP_INFO(this->get_logger(), "STATE: SPAWN_BOX");
        spawnEntity(box_model_name_, pick_x_ + 0.6, pick_y_ + 0.6, 0.05, MissionState::NAV_TO_PICK);
        break;

      case MissionState::NAV_TO_PICK:
        RCLCPP_INFO(this->get_logger(), "STATE: NAV_TO_PICK");
        startNavigation(pick_x_, pick_y_);
        state_ = MissionState::MONITOR_NAV_PICK;
        break;

      case MissionState::MONITOR_NAV_PICK:
        {
          double distance = distanceToGoal(pick_x_, pick_y_);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Monitoring pick navigation - Distance: %.3f m", distance);
          
          if (isGoalReached(pick_x_, pick_y_)) {
            RCLCPP_INFO(this->get_logger(), "Close enough to pick location! Proceeding.");
            cancelCurrentNavigation();
            state_ = MissionState::MOCK_PICK;
          } else if ((this->now() - nav_start_time_).seconds() > nav_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Pick navigation timeout after %.1f seconds", nav_timeout_);
            cancelCurrentNavigation();
            state_ = MissionState::FAILED;
          }
        }
        break;

      case MissionState::MOCK_PICK:
        RCLCPP_INFO(this->get_logger(), "STATE: MOCK_PICK (simulate attach)");
        deleteEntity(box_model_name_, MissionState::SPAWN_ATTACHED);
        break;

      case MissionState::SPAWN_ATTACHED:
        {
          nav_msgs::msg::Odometry::SharedPtr odom;
          {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom = latest_odom_;
          }
          if (!odom) {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry to spawn attached box");
            break;
          }
          double rx = odom->pose.pose.position.x;
          double ry = odom->pose.pose.position.y;
          double rz = odom->pose.pose.position.z + 0.6;
          spawnEntity(attached_model_name_, rx, ry, rz, MissionState::NAV_TO_DROP);
        }
        break;

      case MissionState::NAV_TO_DROP:
        RCLCPP_INFO(this->get_logger(), "STATE: NAV_TO_DROP");
        startNavigation(drop_x_, drop_y_);
        state_ = MissionState::MONITOR_NAV_DROP;
        break;

      case MissionState::MONITOR_NAV_DROP:
        {
          double distance = distanceToGoal(drop_x_, drop_y_);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Monitoring drop navigation - Distance: %.3f m", distance);
          
          if (isGoalReached(drop_x_, drop_y_)) {
            RCLCPP_INFO(this->get_logger(), "Close enough to drop location! Proceeding.");
            cancelCurrentNavigation();
            state_ = MissionState::MOCK_DROP;
          } else if ((this->now() - nav_start_time_).seconds() > nav_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Drop navigation timeout after %.1f seconds", nav_timeout_);
            cancelCurrentNavigation();
            state_ = MissionState::FAILED;
          }
        }
        break;

      case MissionState::MOCK_DROP:
        RCLCPP_INFO(this->get_logger(), "STATE: MOCK_DROP (simulate detach)");
        deleteEntity(attached_model_name_, MissionState::SPAWN_DROP);
        break;

      case MissionState::SPAWN_DROP:
        spawnEntity(box_model_name_, drop_x_ + 0.6, drop_y_ + 0.6, 0.05, MissionState::NAV_HOME);
        break;

      case MissionState::NAV_HOME:
        RCLCPP_INFO(this->get_logger(), "STATE: NAV_HOME");
        startNavigation(home_x_, home_y_);
        state_ = MissionState::MONITOR_NAV_HOME;
        break;

      case MissionState::MONITOR_NAV_HOME:
        {
          double distance = distanceToGoal(home_x_, home_y_);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Monitoring home navigation - Distance: %.3f m", distance);
          
          if (isGoalReached(home_x_, home_y_)) {
            RCLCPP_INFO(this->get_logger(), "Close enough to home location! Mission complete.");
            cancelCurrentNavigation();
            state_ = MissionState::DONE;
          } else if ((this->now() - nav_start_time_).seconds() > nav_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Home navigation timeout after %.1f seconds", nav_timeout_);
            cancelCurrentNavigation();
            state_ = MissionState::FAILED;
          }
        }
        break;

      case MissionState::DONE:
        RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE: switching to IDLE");
        state_ = MissionState::IDLE;
        break;

      case MissionState::FAILED:
        RCLCPP_ERROR(this->get_logger(), "MISSION FAILED: switching to IDLE");
        cancelCurrentNavigation();
        state_ = MissionState::IDLE;
        break;

      case MissionState::IDLE:
      default:
        break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}