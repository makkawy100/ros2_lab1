import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)

        self.pose = None
        self.cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10,
            callback_group=self.cb_group)

        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        self.create_service(
            SetSpeedLimits, 'set_speed_limits', self.set_limits_callback,
            callback_group=self.cb_group)
        self.get_logger().info('GoToPose server ready')

    def pose_callback(self, msg):
        self.pose = msg

    def set_limits_callback(self, request, response):
        lin, ang = request.max_linear_speed, request.max_angular_speed
        if not (math.isfinite(lin) and math.isfinite(ang)) or lin <= 0.0 or ang <= 0.0:
            response.success = False
            response.message = 'Speeds must be finite and greater than zero'
            return response
        results = self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, float(lin)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, float(ang)),
        ])
        response.success = all(r.successful for r in results)
        response.message = ('Speed limits updated' if response.success
                            else 'Failed to set parameters')
        return response

    def goal_callback(self, goal_request):
        t = goal_request.target
        if not all(math.isfinite(v) for v in (t.x, t.y, t.yaw)):
            self.get_logger().warn('Rejected: non-finite target')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def clamp(v, limit):
        return max(-limit, min(limit, v))

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target
        feedback = GoToPose.Feedback()
        result = GoToPose.Result()
        self.get_logger().info(
            f'Executing goal: x={target.x} y={target.y} yaw={target.yaw}')

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled'
                    return result

                if self.pose is None:
                    time.sleep(0.05)
                    continue

                # Read speed limits every cycle so changes apply live
                max_lin = self.get_parameter('max_linear_speed').value
                max_ang = self.get_parameter('max_angular_speed').value

                dx = target.x - self.pose.x
                dy = target.y - self.pose.y
                distance = math.hypot(dx, dy)
                feedback.distance_remaining = float(distance)
                goal_handle.publish_feedback(feedback)

                cmd = Twist()
                if distance > 0.05:
                    angle_error = self.normalize_angle(
                        math.atan2(dy, dx) - self.pose.theta)
                    if abs(angle_error) > 0.08:
                        cmd.angular.z = self.clamp(4.0 * angle_error, max_ang)
                    else:
                        cmd.linear.x = min(max_lin, max(0.3, 1.5 * distance))
                        cmd.angular.z = self.clamp(2.0 * angle_error, max_ang)
                else:
                    yaw_error = self.normalize_angle(
                        target.yaw - self.pose.theta)
                    if abs(yaw_error) > 0.05:
                        cmd.angular.z = self.clamp(4.0 * yaw_error, max_ang)
                    else:
                        self.cmd_pub.publish(Twist())
                        goal_handle.succeed()
                        result.success = True
                        result.message = 'Target pose reached'
                        return result

                self.cmd_pub.publish(cmd)
                time.sleep(0.05)

            goal_handle.abort()
            result.success = False
            result.message = 'Shutdown'
            return result
        except Exception as exc:
            self.get_logger().error(str(exc))
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result
        finally:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
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
