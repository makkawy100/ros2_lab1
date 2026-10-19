import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.declare_parameter('initial_x', 5.544445)
        self.declare_parameter('initial_y', 5.544445)
        self.declare_parameter('initial_theta', 0.0)

        self.cb_group = ReentrantCallbackGroup()
        self.clear_client = self.create_client(
            Empty, '/clear', callback_group=self.cb_group)
        self.teleport_client = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute',
            callback_group=self.cb_group)
        self.pen_client = self.create_client(
            SetPen, '/turtle1/set_pen', callback_group=self.cb_group)
        self.create_service(
            Trigger, 'reset_turtle', self.reset_callback,
            callback_group=self.cb_group)
        self.get_logger().info('Reset service ready: /reset_turtle')

    def pen_request(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = 179, 184, 255, 3  # turtlesim default
        req.off = 1 if off else 0
        return req

    def reset_callback(self, request, response):
        clients = (self.clear_client, self.teleport_client, self.pen_client)
        if not all(c.wait_for_service(timeout_sec=2.0) for c in clients):
            response.success = False
            response.message = 'turtlesim services not available'
            return response

        tp = TeleportAbsolute.Request()
        tp.x = float(self.get_parameter('initial_x').value)
        tp.y = float(self.get_parameter('initial_y').value)
        tp.theta = float(self.get_parameter('initial_theta').value)

        try:
            self.pen_client.call(self.pen_request(off=True))   # pen up
            self.teleport_client.call(tp)
            self.clear_client.call(Empty.Request())
        finally:
            self.pen_client.call(self.pen_request(off=False))  # pen down

        response.success = True
        response.message = 'Turtle reset and screen cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
    executor = MultiThreadedExecutor(num_threads=2)
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
