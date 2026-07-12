// downlink: endpoint-layer transparent relay (see transparent_relay_architecture.md)
//
// core idea: the middleman "answers for no one" — it only transports messages + rewrites joint names.
//   D10 fake endpoints (3 services + 2 topics) ←forwarded verbatim→ D20/D30 real JTC endpoints
//   goal_id(UUID) is never swapped → no mapping table; no state machine; failure semantics = direct connection.
//
// design points:
// (1) deferred response: D10's service callback only "takes a ticket"
//     (storing the ticket rmw_request_id_t) and returns immediately, so the executor is not occupied;
//     when the arm's real answer arrives (on the D20/D30 thread) it uses the ticket to send_response with the answer.
//     ⚠ this file's only API bet: create_service's "deferred response with a service handle"
//       callback signature (service, request_header, request) — rclcpp_action
//       uses this mechanism internally to suspend get_result; if it does not compile, switch to the 2-argument variant
//       (request_header, request) + a member holding the service handle.
// (2) closure relay: each request's context is carried by its own lambda → this program has zero global mutable state.
// (3) content rewriting in only two places: strip the prefix from the send_goal request's Goal (4 fields), add the prefix to feedback.
// (4) the status topic must mirror transient_local QoS (on both the subscribe and publish sides).
// (5) fail-fast: if the remote service is not online → immediately return accepted=false / ERROR_REJECTED,
//     compensating for the effect that "the relay's presence masks an offline arm".
//
// known limitation: bare joint names in Result.error_string are not translated; not validated on real hardware.
// usage: ros2 run dual_arm_domain_bridge trajectory_downlink_endpoint_relay [10 11 12]

#include "dual_arm_domain_bridge/multi_context.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

using dual_arm_domain_bridge::DomainNode;
using dual_arm_domain_bridge::make_domain_node;
using FJT = control_msgs::action::FollowJointTrajectory;

// the implementation-layer types of the five endpoints (rclcpp_action relies on them; they exist stably)
using SendGoalSrv = FJT::Impl::SendGoalService;
using CancelSrv = FJT::Impl::CancelGoalService;
using GetResultSrv = FJT::Impl::GetResultService;
using FeedbackMsg = FJT::Impl::FeedbackMessage;
using StatusMsg = FJT::Impl::GoalStatusMessage;

namespace
{

constexpr char kArmAction[] = "/joint_trajectory_controller/follow_joint_trajectory";

std::string strip_prefix(const std::string & s, const std::string & prefix)
{
  return (s.rfind(prefix, 0) == 0) ? s.substr(prefix.size()) : s;
}

// downlink content rewriting: strip the prefix from all joint names in the Goal (4 places)
void strip_goal(FJT::Goal & g, const std::string & prefix)
{
  for (auto & n : g.trajectory.joint_names) {n = strip_prefix(n, prefix);}
  for (auto & t : g.path_tolerance) {t.name = strip_prefix(t.name, prefix);}
  for (auto & t : g.goal_tolerance) {t.name = strip_prefix(t.name, prefix);}
  for (auto & n : g.multi_dof_trajectory.joint_names) {n = strip_prefix(n, prefix);}
}

// the five-endpoint relay for one arm (zero global state: entirely closure-driven)
class ArmEndpointRelay
{
public:
  ArmEndpointRelay(
    const rclcpp::Node::SharedPtr & host_node, const rclcpp::Node::SharedPtr & arm_node,
    const std::string & host_action, std::string prefix)
  : prefix_(std::move(prefix)), logger_(host_node->get_logger())
  {
    const std::string arm_action(kArmAction);

    // ---- D20/D30 side: the 3 service clients to the real JTC ----
    cl_send_goal_ = arm_node->create_client<SendGoalSrv>(arm_action + "/_action/send_goal");
    cl_cancel_ = arm_node->create_client<CancelSrv>(arm_action + "/_action/cancel_goal");
    cl_get_result_ = arm_node->create_client<GetResultSrv>(arm_action + "/_action/get_result");

    // ---- D10 side: fake endpoint services ×3 (deferred-response mode: callback carries no Response) ----
    // ① send_goal — the only endpoint whose request content must be rewritten
    sv_send_goal_ = host_node->create_service<SendGoalSrv>(
      host_action + "/_action/send_goal",
      [this](const std::shared_ptr<rclcpp::Service<SendGoalSrv>> srv,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<SendGoalSrv::Request> req)
      {
        if (!cl_send_goal_->service_is_ready()) {       // (5) fail-fast
          SendGoalSrv::Response resp;                    // accepted defaults to false
          srv->send_response(*header, resp);
          RCLCPP_WARN(logger_, "[%s relay] arm side is offline → immediately return accepted=false",
                      prefix_.c_str());
          return;
        }
        auto fwd = std::make_shared<SendGoalSrv::Request>(*req);
        strip_goal(fwd->goal, prefix_);                  // (3) strip the prefix
        RCLCPP_INFO(logger_, "[%s relay] send_goal ticketed and forwarded (%zu points)",
                    prefix_.c_str(), fwd->goal.trajectory.points.size());
        cl_send_goal_->async_send_request(
          fwd,
          [srv, header](rclcpp::Client<SendGoalSrv>::SharedFuture fut) {
            srv->send_response(*header, *fut.get());     // forward the arm's own answer verbatim
          });
      });                                                // ← return immediately, do not occupy the executor

    // ② cancel_goal — zero rewriting
    sv_cancel_ = host_node->create_service<CancelSrv>(
      host_action + "/_action/cancel_goal",
      [this](const std::shared_ptr<rclcpp::Service<CancelSrv>> srv,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<CancelSrv::Request> req)
      {
        if (!cl_cancel_->service_is_ready()) {
          CancelSrv::Response resp;
          resp.return_code = CancelSrv::Response::ERROR_REJECTED;
          srv->send_response(*header, resp);
          return;
        }
        RCLCPP_WARN(logger_, "[%s relay] cancel ticketed and forwarded", prefix_.c_str());
        cl_cancel_->async_send_request(
          std::make_shared<CancelSrv::Request>(*req),
          [srv, header, this](rclcpp::Client<CancelSrv>::SharedFuture fut) {
            srv->send_response(*header, *fut.get());
            RCLCPP_WARN(logger_, "[%s relay] cancel reply forwarded (code=%d, %zu in-progress cancellations)",
                        prefix_.c_str(), static_cast<int>(fut.get()->return_code),
                        fut.get()->goals_canceling.size());
          });
      });

    // ③ get_result — zero rewriting; the ticket is held for the entire execution period (normal, does not occupy a thread)
    sv_get_result_ = host_node->create_service<GetResultSrv>(
      host_action + "/_action/get_result",
      [this](const std::shared_ptr<rclcpp::Service<GetResultSrv>> srv,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<GetResultSrv::Request> req)
      {
        if (!cl_get_result_->service_is_ready()) {
          GetResultSrv::Response resp;                   // status defaults to UNKNOWN
          srv->send_response(*header, resp);
          return;
        }
        RCLCPP_INFO(logger_, "[%s relay] get_result ticketed and forwarded", prefix_.c_str());
        cl_get_result_->async_send_request(
          std::make_shared<GetResultSrv::Request>(*req),
          [srv, header, this](rclcpp::Client<GetResultSrv>::SharedFuture fut) {
            srv->send_response(*header, *fut.get());     // reply after holding the ticket for the entire execution period
            RCLCPP_INFO(logger_, "[%s relay] result forwarded (status=%d)",
                        prefix_.c_str(), static_cast<int>(fut.get()->status));
          });
      });

    // ---- relaying the two topics ----
    // ④ feedback — the only return content that must be rewritten (add the prefix); goal_id passes through
    pub_feedback_ = host_node->create_publisher<FeedbackMsg>(
      host_action + "/_action/feedback", rclcpp::QoS(10));
    sub_feedback_ = arm_node->create_subscription<FeedbackMsg>(
      arm_action + "/_action/feedback", rclcpp::QoS(10),
      [this](FeedbackMsg::ConstSharedPtr msg) {
        FeedbackMsg out = *msg;
        for (auto & n : out.feedback.joint_names) {n = prefix_ + n;}
        pub_feedback_->publish(out);
      });

    // ⑤ status — zero rewriting; (4) transient_local QoS must be mirrored on both sides
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_status_ = host_node->create_publisher<StatusMsg>(
      host_action + "/_action/status", status_qos);
    sub_status_ = arm_node->create_subscription<StatusMsg>(
      arm_action + "/_action/status", status_qos,
      [this](StatusMsg::ConstSharedPtr msg) {pub_status_->publish(*msg);});

    RCLCPP_INFO(logger_, "[%s relay] five endpoints in place: %s ↔ %s",
                prefix_.c_str(), host_action.c_str(), arm_action.c_str());
  }

private:
  std::string prefix_;
  rclcpp::Logger logger_;

  rclcpp::Client<SendGoalSrv>::SharedPtr cl_send_goal_;
  rclcpp::Client<CancelSrv>::SharedPtr cl_cancel_;
  rclcpp::Client<GetResultSrv>::SharedPtr cl_get_result_;

  rclcpp::Service<SendGoalSrv>::SharedPtr sv_send_goal_;
  rclcpp::Service<CancelSrv>::SharedPtr sv_cancel_;
  rclcpp::Service<GetResultSrv>::SharedPtr sv_get_result_;

  rclcpp::Publisher<FeedbackMsg>::SharedPtr pub_feedback_;
  rclcpp::Publisher<StatusMsg>::SharedPtr pub_status_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr sub_feedback_;
  rclcpp::Subscription<StatusMsg>::SharedPtr sub_status_;
};

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

  DomainNode host = make_domain_node("trajectory_downlink_endpoint_relay", host_d, true);
  DomainNode armA = make_domain_node("endpoint_relay_arm_a", arm_a_d, false);
  DomainNode armB = make_domain_node("endpoint_relay_arm_b", arm_b_d, false);

  ArmEndpointRelay relay_a(
    host.node, armA.node,
    "/big_arm/joint_trajectory_controller/follow_joint_trajectory", "big_");
  ArmEndpointRelay relay_b(
    host.node, armB.node,
    "/small_arm/joint_trajectory_controller/follow_joint_trajectory", "small_");

  RCLCPP_INFO(host.node->get_logger(),
    "endpoint transparent relay started: D%zu fake endpoints ×2 ↔ D%zu/D%zu real JTC | answers for no one, UUID passthrough, "
    "failure semantics = direct connection", host_d, arm_a_d, arm_b_d);

  host.start();
  armA.start();
  armB.start();
  host.join();
  armA.join();
  armB.join();

  rclcpp::uninstall_signal_handlers();
  return 0;
}
