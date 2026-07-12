// uplink bridge (Plan B program 1/2, resident, stateless)
// responsibility = the existing joint_state_merger moved to a cross-domain version + stock joint name → prefixed name translation
//
//   ctx_armA (D20): subscribe to /joint_states ── joint_N → big_joint_N ──┐
//   ctx_armB (D30): subscribe to /joint_states ── joint_N → small_joint_N ┤→ merge 12 axes
//   ctx_host (D10): ────────────────────────────────────────────┘→ publish /joint_states
//
// consumers: move_group CurrentStateMonitor (planning start state), the host rsp→TF→RViz display
// without it the host is "blind": planning reports Failed to fetch current robot state, and the RViz model freezes
//
// usage: ros2 run dual_arm_domain_bridge joint_state_uplink_bridge [host_d armA_d armB_d]
//       default domains: host=10, armA=11, armB=12

#include "dual_arm_domain_bridge/multi_context.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

using dual_arm_domain_bridge::DomainNode;
using dual_arm_domain_bridge::make_domain_node;
using sensor_msgs::msg::JointState;

namespace
{

// ---- configuration (prefixes aligned with the host's dual-arm model) ----
const char * kPrefixA = "big_";     // joint_1 → big_joint_1
const char * kPrefixB = "small_";   // joint_1 → small_joint_1

struct SharedState
{
  std::mutex mtx;
  JointState arm_a;        // the latest state after renaming (prefix added)
  JointState arm_b;
  bool has_a{false};
  bool has_b{false};
};

// rename with a prefix (uplink name translation: stock joint_N → prefixed name)
JointState add_prefix(const JointState & in, const std::string & prefix)
{
  JointState out = in;
  for (auto & n : out.name) {
    n = prefix + n;
  }
  return out;
}

// two-arm cache → one merged 12-axis message (position always merged; velocity merged only when both arms are complete)
bool build_merged(SharedState & s, JointState & merged)
{
  std::lock_guard<std::mutex> lk(s.mtx);
  if (!s.has_a || !s.has_b) {return false;}

  merged.name = s.arm_a.name;
  merged.name.insert(merged.name.end(), s.arm_b.name.begin(), s.arm_b.name.end());

  merged.position = s.arm_a.position;
  merged.position.insert(merged.position.end(),
                         s.arm_b.position.begin(), s.arm_b.position.end());

  const bool vel_ok = s.arm_a.velocity.size() == s.arm_a.name.size() &&
                      s.arm_b.velocity.size() == s.arm_b.name.size();
  if (vel_ok) {
    merged.velocity = s.arm_a.velocity;
    merged.velocity.insert(merged.velocity.end(),
                           s.arm_b.velocity.begin(), s.arm_b.velocity.end());
  }
  return true;
}

size_t arg_or(int argc, char ** argv, int idx, size_t fallback)
{
  return (argc > idx) ? static_cast<size_t>(std::strtoul(argv[idx], nullptr, 10)) : fallback;
}

}  // namespace

int main(int argc, char ** argv)
{
  const size_t host_d = arg_or(argc, argv, 1, 10);
  const size_t arm_a_d = arg_or(argc, argv, 2, 11);
  const size_t arm_b_d = arg_or(argc, argv, 3, 12);

  rclcpp::install_signal_handlers();

  // three contexts (host built first, responsible for initializing logging)
  DomainNode host = make_domain_node("joint_state_uplink_bridge", host_d, true);
  DomainNode armA = make_domain_node("uplink_arm_a", arm_a_d, false);
  DomainNode armB = make_domain_node("uplink_arm_b", arm_b_d, false);

  auto state = std::make_shared<SharedState>();
  auto pub = host.node->create_publisher<JointState>("/joint_states", 10);
  auto host_clock = host.node->get_clock();

  // any arm update → merge and publish immediately (event-driven, minimum latency)
  auto publish_merged = [state, pub, host_clock]() {
      JointState merged;
      if (!build_merged(*state, merged)) {return;}
      merged.header.stamp = host_clock->now();
      pub->publish(merged);
    };

  auto sub_a = armA.node->create_subscription<JointState>(
    "/joint_states", 10,
    [state, publish_merged](JointState::ConstSharedPtr msg) {
      {
        std::lock_guard<std::mutex> lk(state->mtx);
        state->arm_a = add_prefix(*msg, kPrefixA);
        state->has_a = true;
      }
      publish_merged();
    });

  auto sub_b = armB.node->create_subscription<JointState>(
    "/joint_states", 10,
    [state, publish_merged](JointState::ConstSharedPtr msg) {
      {
        std::lock_guard<std::mutex> lk(state->mtx);
        state->arm_b = add_prefix(*msg, kPrefixB);
        state->has_b = true;
      }
      publish_merged();
    });

  RCLCPP_INFO(host.node->get_logger(),
    "uplink bridge started: D%zu (Arm A, prefix %s) + D%zu (Arm B, prefix %s) → D%zu /joint_states (12 axes)",
    arm_a_d, kPrefixA, arm_b_d, kPrefixB, host_d);
  RCLCPP_INFO(host.node->get_logger(),
    "waiting for both arms' /joint_states ... (not published until both arms are online)");

  host.start();
  armA.start();
  armB.start();
  host.join();
  armA.join();
  armB.join();

  rclcpp::uninstall_signal_handlers();
  return 0;
}
