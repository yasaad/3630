# mock geometry_msgs
from std_msgs.msg import Header

class PoseStamped:

    def __init__(self, header=None, pose=None):
        self.header = header if header else Header()
        self.pose = pose if pose else Pose()


class Pose:

    def __init__(self, position=None, orientation=None):
        self.position = position if position else Point()
        self.orientation = orientation if orientation else Quaternion()


class Point:

    def __init__(self, x=None, y=None, z=None):
        self.x = x if x else 0.0
        self.y = y if y else 0.0
        self.z = z if z else 0.0


class Quaternion:

    def __init__(self, x=None, y=None, z=None, w=None):
        self.x = x if x else 0.0
        self.y = y if y else 0.0
        self.z = z if z else 0.0
        self.w = w if w else 1.0
