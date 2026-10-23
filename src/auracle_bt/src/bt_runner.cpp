#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <behaviortree_cpp/xml_parsing.h>

using Poses = std::vector<geometry_msgs::msg::PoseStamped>;

struct DetectionState { std::string label; };

// Skill 1: wait for the detector, store the label
class GetDetectedClass : public BT::StatefulActionNode {
public:
  GetDetectedClass(const std::string& n, const BT::NodeConfig& c,
                   std::shared_ptr<DetectionState> s)
  : BT::StatefulActionNode(n, c), state_(s) {}
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<std::string>("detected_class")};
  }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    if (state_->label.empty()) return BT::NodeStatus::RUNNING;  // keep waiting
    std::string l = state_->label;
    std::transform(l.begin(), l.end(), l.begin(), ::toupper);
    setOutput("detected_class", l);
    return BT::NodeStatus::SUCCESS;
  }
  std::shared_ptr<DetectionState> state_;
};

// Skill 2: pick up the route card from waypoints.yaml
class LoadWaypoints : public BT::SyncActionNode {
public:
  LoadWaypoints(const std::string& n, const BT::NodeConfig& c)
  : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("path_name"),
            BT::OutputPort<Poses>("goals")};
  }
  BT::NodeStatus tick() override {
    std::string key;
    if (!getInput("path_name", key)) return BT::NodeStatus::FAILURE;
    auto file = ament_index_cpp::get_package_share_directory("auracle_bringup")
                + "/config/waypoints.yaml";
    YAML::Node root = YAML::LoadFile(file);
    if (!root[key]) return BT::NodeStatus::FAILURE;
    Poses goals;
    for (const auto& p : root[key]) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = p["x"].as<double>();
      ps.pose.position.y = p["y"].as<double>();
      double yaw = p["yaw"].as<double>();
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      goals.push_back(ps);
    }
    setOutput("goals", goals);
    return BT::NodeStatus::SUCCESS;
  }
};

// Skill 3: drive the route using Nav2
class FollowWaypoints : public BT::StatefulActionNode {
  using Nav = nav2_msgs::action::NavigateThroughPoses;
  using GH = rclcpp_action::ClientGoalHandle<Nav>;
public:
  FollowWaypoints(const std::string& n, const BT::NodeConfig& c,
                  rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(n, c), node_(node) {
    client_ = rclcpp_action::create_client<Nav>(node_, "navigate_through_poses");
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<Poses>("goals")};
  }
  BT::NodeStatus onStart() override {
    Poses goals;
    if (!getInput("goals", goals) || goals.empty()) return BT::NodeStatus::FAILURE;
    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
      return BT::NodeStatus::FAILURE;
    Nav::Goal g; g.poses = goals;
    done_ = false; ok_ = false; handle_.reset();
    rclcpp_action::Client<Nav>::SendGoalOptions o;
    o.goal_response_callback = [this](GH::SharedPtr h) {
      if (!h) { done_ = true; ok_ = false; } else handle_ = h; };
    o.result_callback = [this](const GH::WrappedResult& r) {
      done_ = true; ok_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED); };
    client_->async_send_goal(g, o);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (!done_) return BT::NodeStatus::RUNNING;
    return ok_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override { if (handle_) client_->async_cancel_goal(handle_); }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Nav>::SharedPtr client_;
  GH::SharedPtr handle_;
  bool done_{false}, ok_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_runner",
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("use_sim_time", true)}));   // you are in simulation

  auto state = std::make_shared<DetectionState>();
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/detected_class", 10,
      [state](std_msgs::msg::String::SharedPtr m) { state->label = m->data; });

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GetDetectedClass>("GetDetectedClass", state);
  factory.registerNodeType<LoadWaypoints>("LoadWaypoints");
  factory.registerNodeType<FollowWaypoints>("FollowWaypoints", node);

  // Palette file for Groot2
  std::ofstream("/tmp/auracle_nodes_model.xml") << BT::writeTreeNodesModelXML(factory);

  auto share = ament_index_cpp::get_package_share_directory("auracle_bt");
  auto tree = factory.createTreeFromFile(share + "/behavior_trees/sorting_tree.xml");
  BT::Groot2Publisher publisher(tree);   // lets Groot2 watch live

  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    rate.sleep();
  }
  RCLCPP_INFO(node->get_logger(), "Tree finished: %s", BT::toStr(status).c_str());
  rclcpp::shutdown();
}
