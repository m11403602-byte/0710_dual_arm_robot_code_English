// shared template for multi-domain contexts (shared by the uplink bridge and the downlink client)
// core technique: rclcpp opens multiple Contexts in one process, each attached to a different ROS_DOMAIN
//   (InitOptions::set_domain_id, supported on Humble)
// rule: a node cannot attach to the same executor across contexts → one context, one executor, one thread
#ifndef DUAL_ARM_DOMAIN_BRIDGE__MULTI_CONTEXT_HPP_
#define DUAL_ARM_DOMAIN_BRIDGE__MULTI_CONTEXT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <thread>

namespace dual_arm_domain_bridge
{

// the "context + node + executor + spin thread" quadruple for a single domain
struct DomainNode
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spin_thread;

  void start()
  {
    spin_thread = std::thread([this]() {executor->spin();});
  }

  void join()
  {
    if (spin_thread.joinable()) {spin_thread.join();}
  }
};

// create a node attached to the specified DOMAIN
// when first_context = true, it is responsible for initializing logging (which the whole process can do only once)
inline DomainNode make_domain_node(
  const std::string & node_name, size_t domain_id, bool first_context)
{
  DomainNode d;

  rclcpp::InitOptions init_opts;
  init_opts.auto_initialize_logging(first_context);
  init_opts.set_domain_id(domain_id);     // [key] the DDS domain of this context
  init_opts.shutdown_on_signal = true;    // Ctrl+C → close the context → the executor returns

  d.context = std::make_shared<rclcpp::Context>();
  d.context->init(0, nullptr, init_opts);

  rclcpp::NodeOptions node_opts;
  node_opts.context(d.context);
  d.node = std::make_shared<rclcpp::Node>(node_name, node_opts);

  rclcpp::ExecutorOptions exec_opts;
  exec_opts.context = d.context;
  d.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_opts);
  d.executor->add_node(d.node);
  return d;
}

}  // namespace dual_arm_domain_bridge

#endif  // DUAL_ARM_DOMAIN_BRIDGE__MULTI_CONTEXT_HPP_
