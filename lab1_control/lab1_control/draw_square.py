import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from turtlesim.msg import Pose
from std_srvs.srv import Trigger
from lab1_interfaces.action import GoToPose, DrawSquare
from lab1_interfaces.msg import TargetPose


class DrawSquareServer(Node):
    def __init__(self):
        super().__init__('draw_square_server')
        self.declare_parameter('side_length', 2.0)
        self.pose = None
        self.cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10,
            callback_group=self.cb_group)
        self.client = ActionClient(
            self, GoToPose, 'go_to_pose', callback_group=self.cb_group)
        self.reset_client = self.create_client(
            Trigger, 'reset_turtle', callback_group=self.cb_group)
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self.execute_callback,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self.cb_group)
        self.get_logger().info('DrawSquare server ready: /draw_square')

    def pose_callback(self, msg):
        self.pose = msg

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def fail(self, goal_handle, result, text):
        self.get_logger().error(text)
        goal_handle.abort()
        result.success = False
        result.message = text
        return result

    def execute_callback(self, goal_handle):
        result = DrawSquare.Result()
        feedback = DrawSquare.Feedback()
        side = float(self.get_parameter('side_length').value)

        # Reset first: clean screen + known start pose
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            return self.fail(goal_handle, result, 'reset service not available')
        reset_future = self.reset_client.call_async(Trigger.Request())
        while not reset_future.done():
            time.sleep(0.02)
        if not reset_future.result().success:
            return self.fail(goal_handle, result, 'Reset failed')
        self.pose = None  # discard the pre-reset pose, wait for a fresh one
        deadline = time.time() + 3.0
        while self.pose is None and time.time() < deadline:
            time.sleep(0.05)
        time.sleep(0.2)
        if self.pose is None:
            return self.fail(goal_handle, result, 'No pose received after reset')
        if not self.client.wait_for_server(timeout_sec=5.0):
            return self.fail(goal_handle, result, 'go_to_pose server not available')

        # Compute the 4 corners relative to the current pose
        x, y, th0 = self.pose.x, self.pose.y, self.pose.theta
        corners = []
        for i in range(4):
            heading = th0 + i * math.pi / 2
            x += side * math.cos(heading)
            y += side * math.sin(heading)
            corners.append((x, y, self.normalize_angle(heading + math.pi / 2)))

        for cx, cy, _ in corners:
            if not (0.5 <= cx <= 10.5 and 0.5 <= cy <= 10.5):
                return self.fail(goal_handle, result,
                                 'Square would leave the turtlesim window')

        for i, (cx, cy, cyaw) in enumerate(corners):
            side_number = i + 1
            feedback.current_side = side_number
            self.get_logger().info(
                f'Side {side_number}/4 -> x={cx:.2f} y={cy:.2f}')

            sub_goal = GoToPose.Goal()
            sub_goal.target = TargetPose(x=cx, y=cy, yaw=cyaw)

            def fb_cb(msg, n=side_number):
                feedback.current_side = n
                feedback.distance_remaining = msg.feedback.distance_remaining
                goal_handle.publish_feedback(feedback)

            send_future = self.client.send_goal_async(
                sub_goal, feedback_callback=fb_cb)
            while not send_future.done():
                time.sleep(0.02)
            sub_handle = send_future.result()
            if not sub_handle.accepted:
                return self.fail(goal_handle, result, 'GoToPose goal rejected')

            res_future = sub_handle.get_result_async()
            while not res_future.done():
                if goal_handle.is_cancel_requested:
                    sub_handle.cancel_goal_async()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Square canceled'
                    return result
                time.sleep(0.05)

            if not res_future.result().result.success:
                return self.fail(goal_handle, result,
                                 f'Side {side_number} failed')

        goal_handle.succeed()
        result.success = True
        result.message = 'Square completed'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
