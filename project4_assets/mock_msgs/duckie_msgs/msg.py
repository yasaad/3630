# mock duckie_msgs

class Wheels:

    def __init__(self, left_wheel=None, right_wheel=None):
        self.left_wheel = left_wheel if left_wheel else 0.0
        self.right_wheel = right_wheel if right_wheel else 0.0


class Obstacle:

    def __init__(self, x=None, y=None, width=None, height=None):
        self.x = x if x else 0.0
        self.y = y if y else 0.0
        self.width = width if width else 0.0
        self.height = height if height else 0.0


class ObstacleList:

    def __init__(self, map_width=None, map_height=None, obs=None):
        self.map_width = map_width if map_width else 0.0
        self.map_height = map_height if map_height else 0.0
        self.obs = obs if obs else []


class RangeBearingLandmark:

    def __init__(self, id=None, bearing=None, range=None):
        self.id = id if id else ''
        self.bearing = bearing if bearing else 0.0
        self.range = range if range else 0.0


class RangeBearingLandmarkList:

    def __init__(self, landmarks=None):
        self.landmarks = landmarks if landmarks else []
