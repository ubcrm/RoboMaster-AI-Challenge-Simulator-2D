import numpy as np
import pickle
import pathlib

# Indices of important robot properties in state.agents[car_num]
from _modules.robot import Robot
from _modules.waypoints.navigator import Navigator

OWNER = 0
POS_X = 1
POS_Y = 2
ANGLE = 3
YAW = 4
BULLET_COUNT = 10


def det(v0, v1):
    return v0[0]*v1[1] - v1[0]*v0[1]


class Actor:
    def __init__(self, car_num):
        self.car_num = car_num
        self.team = 1 if car_num % 2 == 0 else 0 # Red = 0, Blue = 1
        self.prev_action = None
        self.next_waypoint = None
        self.destination = None
        self.nav = Navigator(pathlib.Path('_modules', 'waypoints', 'data.json'))
        #self.current_robot: Robot = Robot()
    
    def action_from_state(self, state):
        '''Given the current state of the arena, determine what this robot should do next
        using a simple rule-based algorithm. This action is represented as an array of numbers,
        which are interpreted by game.step()'''
        # action = [x, y, rotate, barrel_yaw, shoot, supply, shoot_mode, autoaim]
        x = y = rotate = yaw = shoot = supply = shoot_mode = 0
        autoaim = 1

        '''
        if self.get_property(state, BULLET_COUNT) == 0:
            supply_zone = g_map.areas[self.team][1]
            supply_zone_x = np.mean(supply_zone[:2])
            supply_zone_y = np.mean(supply_zone[2:])
            self.set_destination((supply_zone_x, supply_zone_y))
        elif np.sum(state.vision[self.car_num]) > 0:
            #TODO: Change this condition so it ignores friendly robots
            enemy_coords = g_map.areas[self.team][1]
            enemy_x = np.mean(enemy_coords[:2])
            enemy_y = np.mean(enemy_coords[2:])
            self.set_destination((enemy_x, enemy_y))
            
            shoot = 1
        '''
        x,y,rotate = self.navigate(state, 6, 15, [17, 4])

        action = [x, y, rotate, yaw, shoot, supply, shoot_mode, autoaim]
        
        self.prev_action = action
        return action

    '''
    @arg pos should be np.array([x, y])
    '''
    def nearest_waypoint(self, pos):
        return np.argmin(np.linalg.norm(self.nav.nodes - pos))

    def get_path(self, from_waypoint, to_waypoint, avoid_nodes=None):
        path = self.nav.navigate(from_waypoint, to_waypoint, avoid_nodes)
        return self.nav.interpolate(path, 5) if path is not None else None

    '''
        Sets the destination to the nearest waypoint, stored as the waypoint number
        @arg dest should be np.array([x, y])
    '''
    def set_destination(self, dest):
        '''Update the robots (x,y) destination co-ordinates'''
        self.destination = self.nearest_waypoint(dest)

    def navigate(self, state, from_waypoint, to_waypoint, avoid_nodes=None):
        '''Pathfind to the destination. Returns the x,y,rotation values'''
        path = self.get_path(from_waypoint, to_waypoint, avoid_nodes)
        if path is None:
            return 0, 0, 0

        target = np.array([path[0][-1], path[1][-1]])
        pos_angle = state.robots_status[0][-1]
        pos_vec = np.array([np.cos(pos_angle * np.pi / 180), np.sin(pos_angle * np.pi / 180)])
        pos = np.array(state.robots_status[0][0:2])

        for t_x, t_y in zip(path[0], path[1]):
            if np.linalg.norm(target - np.array([t_x, t_y])) < np.linalg.norm(target - pos):
                target = np.array([t_x, t_y])
                break

        print('-------')
        print(pos)
        print(target)
        target_angle = np.arctan2(target[1] - pos[1], target[0] - pos[0])
        target_vec = np.array([np.cos(target_angle), np.sin(target_angle)])
        print(pos_vec)
        print(target_vec)

        '''
        :current navigation:
        - if target is in a 90 degree cone that extends out from the front of the robot, go forward
        - else, dont move, just turn
        - angles for kernel, positive down, negative up
        '''
        turn = np.arcsin(det(pos_vec, target_vec))
        print(turn)
        if abs(turn) < 0.12:
            turn = 0

        forward = 0
        if abs(turn / np.pi * 180) < 8:
            forward = 1

        print('-------')
        return forward, 0, np.sign(turn)

    def get_property(self, state, prop):
        # TODO: Update this method to use the new standard for accessing robot properties
        return state.agents[self.car_num][prop]
