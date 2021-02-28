
class State(object):
    def __init__(self, time, zones, robots):
        self.time = time
        self.zone_types = zones.types
        self.robots_status = [r.status_list() for r in robots]


class Transition(object):
    def __init__(self, old_state: State, new_state: State, action, reward):
        self.old_state = vars(old_state)
        self.new_state = vars(new_state)
        self.action = action
        self.reward = reward


class Record(object):
    def __init__(self, time, robots, bullets):
        self.time = time
        self.robots = robots
        self.bullets = bullets
