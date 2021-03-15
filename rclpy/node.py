# mock rclpy.node.Node

class Node:

    def __init__(self, *args, **kwargs):
        pass

    def create_client(self, *args, **kwargs):
        pass

    def create_service(self, *args, **kwargs):
        pass

    def create_subscription(self, *args, **kwargs):
        pass

    def create_publisher(self, *args, **kwargs):
        pass

    def get_logger(self, *args, **kwargs):
        return Logger()

    def destroy_node(self, *args, **kwargs):
        pass


class Logger:

    def __init__(self, *args, **kwargs):
        pass

    def debug(self, msg, *args, **kwargs):
        print(msg)

    def info(self, msg, *args, **kwargs):
        print(msg)

    def warn(self, msg, *args, **kwargs):
        print(msg)

    def error(self, msg, *args, **kwargs):
        print(msg)

    def fatal(self, msg, *args, **kwargs):
        print(msg)
