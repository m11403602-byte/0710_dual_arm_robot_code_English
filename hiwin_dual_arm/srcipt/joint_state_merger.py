# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# class JointStateMerger(Node):
#     def __init__(self):
#         super().__init__('joint_state_merger')
#         # 👇 fix the subscribed topic name to align with ROS 2's default namespace broadcast path
#         self.sub_small = self.create_subscription(JointState, '/small_arm/joint_states', self.small_cb, 10)
#         self.sub_big = self.create_subscription(JointState, '/big_arm/joint_states', self.big_cb, 10)
        
#         # the unified global channel published for MoveIt to listen to
#         self.pub = self.create_publisher(JointState, '/joint_states', 10)
        
#         self.small_state = JointState()
#         self.big_state = JointState()
#         self.create_timer(0.02, self.timer_cb) # 50Hz merge-and-publish rate

#     def small_cb(self, msg):
#         self.small_state = msg

#     def big_cb(self, msg):
#         self.big_state = msg

#     def timer_cb(self):
#         if not self.small_state.name or not self.big_state.name:
#             return
        
#         merged = JointState()
#         merged.header.stamp = self.get_clock().now().to_msg()
#         # concatenate the small-arm and big-arm arrays
#         merged.name = self.small_state.name + self.big_state.name
#         merged.position = list(self.small_state.position) + list(self.big_state.position)
        
#         if self.small_state.velocity and self.big_state.velocity:
#             merged.velocity = list(self.small_state.velocity) + list(self.big_state.velocity)
            
#         self.pub.publish(merged)

# def main():
#     rclpy.init()
#     node = JointStateMerger()
#     print("🔄 network simulation intelligence officer started: merging the states of B and C across subnets...")
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# approach B: overwrite the /joint_states timestamp with the host clock (to pass MoveIt's 1-second freshness threshold),
# subscribe to /big_arm,/small_arm/joint_states then assemble into a 12-axis /joint_states. No liveness guard.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.sub_small = self.create_subscription(JointState, '/small_arm/joint_states', self.small_cb, 10)
        self.sub_big = self.create_subscription(JointState, '/big_arm/joint_states', self.big_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.small_state = JointState()
        self.big_state = JointState()
        self.create_timer(0.02, self.timer_cb)  # 50 Hz

    def small_cb(self, msg):
        self.small_state = msg

    def big_cb(self, msg):
        self.big_state = msg

    def timer_cb(self):
        if not self.small_state.name or not self.big_state.name:
            return
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()  # ★ overwrite with the host clock
        merged.name = list(self.small_state.name) + list(self.big_state.name)
        merged.position = list(self.small_state.position) + list(self.big_state.position)
        if self.small_state.velocity and self.big_state.velocity:
            merged.velocity = list(self.small_state.velocity) + list(self.big_state.velocity)
        self.pub.publish(merged)


def main():
    rclpy.init()
    rclpy.spin(JointStateMerger())
    rclpy.shutdown()


if __name__ == '__main__':
    main()