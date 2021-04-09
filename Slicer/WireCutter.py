
class WireCutter(object):
    def __init__(self, wire_length, max_height, max_speed, min_speed, release_height=None, start_height=None):
        self.wire_length = wire_length
        self.max_height = max_height
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.release_height = release_height
        self.start_height = start_height
