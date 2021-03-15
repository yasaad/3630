# mock nav_msgs
from std_msgs.msg import Header

class Path:

    def __init__(self, header=None, poses=None):
        self.header = header if header else Header()
        self.poses = poses if poses else []
