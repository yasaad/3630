# mock nav_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class GetPlan:

    def __init__(self):
        pass

    def Request():
        return GetPlan_Request()

    def Response():
        return GetPlan_Response()


class GetPlan_Request:

    def __init__(self, start=None, goal=None, tolerance=None):
        self.start = start if start else PoseStamped()
        self.goal = goal if goal else PoseStamped()
        self.tolerance = tolerance if tolerance else 0.0


class GetPlan_Response:

    def __init__(self, plan=None):
        self.plan = plan if plan else Path()
