import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from claw_control_interfaces.action import MoveClaw
from .claw import Claw
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ActionClaw01(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Claw节点已启动：{name}!")
        self.claw_ = Claw()
        self.action_server_ = ActionServer(
            self, MoveClaw, 'move_claw', self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        command = goal_handle.request.command
        feedback_msg = MoveClaw.Feedback()
        result = MoveClaw.Result()

        if command == Claw.CMD_GRASP:
            self.get_logger().info("收到抓取指令")
            self.claw_.grasp()
        elif command == Claw.CMD_RELEASE:
            self.get_logger().info("收到放下指令")
            self.claw_.release()
        elif command == Claw.CMD_QUERY:
            self.get_logger().info("收到仅查询状态指令")
            self.claw_.print_angles()
        else:
            self.get_logger().info("未知指令")
            result.status = Claw.STATUS_FAILED
            return result

        feedback_msg.status = self.claw_.get_status()
        goal_handle.publish_feedback(feedback_msg)
        result.status = self.claw_.get_status()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionClaw01("action_claw_01")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保正确关闭Claw对象，停止后台线程
        if hasattr(node, 'claw_'):
            node.claw_.close()
        node.destroy_node()
        rclpy.shutdown()
