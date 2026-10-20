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


# Closed-loop tuning.  The turtle is driven from odometry, so the tolerances
# below decide how exact the drawn shape is: 2 mm / ~1 degree.
CONTROL_PERIOD = 0.02      # seconds between control updates
POSITION_TOL = 0.002       # metres - accept a target only this close
YAW_TOL = 0.02             # radians - final heading tolerance (~1.1 deg)
ALIGN_TOL = 0.05           # radians - rotate on the spot above this error
POSITION_GAIN = 2.5        # 1/s - speed per metre of remaining distance
TURN_GAIN = 4.0            # 1/s - angular speed per radian of heading error
DRIVE_TURN_GAIN = 2.0      # 1/s - heading correction while driving
MIN_SPEED = 0.01           # m/s - keeps the last millimetres from stalling


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
                if distance > POSITION_TOL:
                    angle_error = self.normalize_angle(
                        math.atan2(dy, dx) - self.pose.theta)

                    if abs(angle_error) > ALIGN_TOL:
                        # Point at the target first: no forward motion, so the
                        # corner is not cut short.
                        cmd.angular.z = self.clamp(TURN_GAIN * angle_error, max_ang)
                    else:
                        # Brake proportionally to the remaining distance so the
                        # turtle settles on the target instead of overshooting.
                        speed = min(max_lin, max(MIN_SPEED,
                                                 POSITION_GAIN * distance))
                        cmd.linear.x = speed
                        cmd.angular.z = self.clamp(
                            DRIVE_TURN_GAIN * angle_error, max_ang)
                else:
                    yaw_error = self.normalize_angle(
                        target.yaw - self.pose.theta)
                    if abs(yaw_error) > YAW_TOL:
                        cmd.angular.z = self.clamp(TURN_GAIN * yaw_error, max_ang)
                    else:
                        self.cmd_pub.publish(Twist())
                        goal_handle.succeed()
                        result.success = True
                        result.message = 'Target pose reached'
                        return result

                self.cmd_pub.publish(cmd)
                time.sleep(CONTROL_PERIOD)

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
