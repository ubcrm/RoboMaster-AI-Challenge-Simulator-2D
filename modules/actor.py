import numpy as np
import pickle

# Indices of important robot properties in state.agents[car_num]
from modules.robot import Robot
from modules.waypoints.navigator import NavigationGraph

OWNER = 0
POS_X = 1
POS_Y = 2
ANGLE = 3
YAW = 4
BULLET_COUNT = 10

class Actor:
    def __init__(self, car_num, robot):
        self.car_num = car_num
        self.is_blue = robot.is_blue
        self.prev_commands = None
        self.next_waypoint = None
        self.destination = None
        self.nav = NavigationGraph()
        self.robot = robot
    
    def commands_from_state(self, state):
        """Given the current state of the arena, determine what this robot should do next
        using a simple rule-based algorithm. These commands are represented as an array of numbers,
        which are interpreted by game.step()"""
        # return [forward, side, rotate, rotate gun barrel, shoot]
        x = y = rotate = yaw = shoot = 1

        # if self.robot.ammo == 0:
            # supply_zone = g_map.areas[self.team][1]
            # supply_zone_x = np.mean(supply_zone[:2])
            # supply_zone_y = np.mean(supply_zone[2:])
            # self.set_destination((supply_zone_x, supply_zone_y))
        # elif np.sum(state.vision[self.car_num]) > 0:
            # #TODO: Change this condition so it ignores friendly robots
            # enemy_coords = g_map.areas[self.team][1]
            # enemy_x = np.mean(enemy_coords[:2])
            # enemy_y = np.mean(enemy_coords[2:])
            # self.set_destination((enemy_x, enemy_y))
            #
            # shoot = 1
        
        # x,y,rotate = self.navigate()

        commands = [x, y, rotate, yaw, shoot]
        
        self.prev_commands = commands
        return commands

    @property
    def current_waypoint(self):
        return self.nav.get_nearest_waypoint(self.current_robot.center)

    def get_path(self, target_pos):
        return self.nav.calculate_path(self.current_waypoint, self.nav.get_nearest_waypoint(target_pos))

    def set_destination(self, dest):
        """Update the robots (x,y) destination co-ordinates"""
        nav_path = self.get_path(dest)
    
    def navigate(self):
        """Pathfind to the destination. Returns the x,y,rotation values"""
        # TODO: Implement this using the waypoint system to hop in the
        #       direction of the destination
        return 1, 0, 0
