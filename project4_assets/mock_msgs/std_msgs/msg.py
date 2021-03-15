# mock std_msgs
import time

class Header:

    def __init__(self, stamp=None, frame_id=None):
        self.stamp = stamp if stamp else time.time()
        self.frame_id = frame_id if frame_id else ''
