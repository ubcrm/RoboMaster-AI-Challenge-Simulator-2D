
class State(object):
    def __init__(self, time, robots, zones):
        self.time = time
        self.robots = robots
        self.zones = zones


class Record(object):
    def __init__(self, time, robots, bullets):
        self.time = time
        self.robots = robots
        self.bullets = bullets
