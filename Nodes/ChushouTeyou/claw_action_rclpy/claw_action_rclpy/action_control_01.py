import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from claw_control_interfaces.action import MoveClaw

class ActionControl01(Node):
    """Claw Action客户端"""
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Claw客户端节点已启动：{name}!")
        self.action_client_ = ActionClient(self, MoveClaw, 'move_claw')
        self.send_goal_timer_ = self.create_timer(1, self.send_grasp_goal)

    def send_grasp_goal(self):
        self.send_goal_timer_.cancel()
        goal_msg = MoveClaw.Goal()
        goal_msg.command = 0  # 0: 抓取
        self.action_client_.wait_for_server()
        self._send_goal_future = self.action_client_.send_goal_async(goal_msg,
                                                                     feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result status: {result.status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.status}')

def main(args=None):
    rclpy.init(args=args)
    node = ActionControl01("claw_action_control_01")
    rclpy.spin(node)
    rclpy.shutdown()
