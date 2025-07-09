import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from wormhole_nav_msg.action import WormGoal
from geometry_msgs.msg import PoseStamped


class WormGoalClient(Node):

    def __init__(self):
        super().__init__('worm_goal_client')

        self._action_client = ActionClient(self, WormGoal, '/wormhole/navigate')

        self.send_goal()

    def send_goal(self):
        goal_msg = WormGoal.Goal()

        # Populate goal_pose
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header.frame_id = 'map'
        goal_msg.goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal_pose.pose.position.x = -7.601
        goal_msg.goal_pose.pose.position.y = 8.570
        goal_msg.goal_pose.pose.orientation.z = -0.825
        goal_msg.goal_pose.pose.orientation.w = 0.564

        goal_msg.map_id = 3

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose
        self.get_logger().info(
            f"Current Pose x: {current_pose.pose.position.x}, y: {current_pose.pose.position.y}"
        )

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result.msg}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = WormGoalClient()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
