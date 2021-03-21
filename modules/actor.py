import numpy as np
import pathlib

# Indices of important robot properties in state.agents[car_num]
from modules.robot import Robot
from modules.waypoints.navigator import NavigationGraph
from modules.waypoints.navigator import Navigator

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
        self.nav = Navigator(pathlib.Path('modules', 'waypoints', 'data.json'))
        self.robot = robot
        self.zones = None

        # TODO replace dummy values with proper ones representing starting state
        self.next_state_commands = {}
        self.has_ammo = False
        self.not_shooting = True
        self.has_buff = False
        self.is_at_centre = False
        self.centre_waypoint = None
        self.is_at_spawn_zone = False
        self.spawn_waypoint = None
        self.buff_waypoint = None
        self.ammo_waypoint = None
    
    def commands_from_state(self, state):
        """Given the current state of the arena, determine what this robot should do next
        using a simple rule-based algorithm. These commands are represented as an array of numbers,
        which are interpreted by game.step()"""
        x = y = rotate = yaw = shoot = 1

        destination = np.array([756, 217])
        pos = self.nearest_waypoint(self.robot.center)
        dest = self.nearest_waypoint(destination)
        x, y, rotate = self.navigate(state, pos, dest, [17, 20])

        commands = [x, y, rotate, yaw, shoot]
        
        self.prev_commands = commands
        return commands

    '''
       @arg pos should be np.array([x, y])
    '''
    def nearest_waypoint(self, pos):
        return np.argmin([np.linalg.norm(node - pos) for node in self.nav.nodes])

    def get_path(self, from_waypoint, to_waypoint, avoid_nodes=None):
        path = self.nav.navigate(from_waypoint, to_waypoint, avoid_nodes)
        return self.nav.interpolate(path, 20) if path is not None else None

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

        target = [path[0][1], path[1][1]]
        pos_x = state.robots[0].center[0]
        pos_y = state.robots[0].center[1]
        pos_angle = state.robots[0].angle

        '''
        :current navigation:
        - if target is in a 90 degree cone that extends out from the front of the robot, go forward
        - else, dont move, just turn
        '''
        angle_diff = np.abs(
            np.array(-np.arctan((target[1] - pos_y) / (target[0] - pos_x)) - pos_angle * np.pi / 180)) * 180 / np.pi
        print('-------')
        print(-np.arctan2(-(target[1] - pos_y), (target[0] - pos_x)))
        print(pos_angle * np.pi / 180)
        print(angle_diff)
        print('-------')

        if np.abs(angle_diff) > 45:
            return 0, 0, np.sign(angle_diff)
        else:
            return np.sign(target[0] - pos_x), np.sign(target[1] - pos_y), np.sign(angle_diff)

    def take_action(self, state):
        """
        Called on every frame by the Actor, it first updates the board state as stored in Actor memory
        Then it checks the current robot state to determine what the next line of action should be.
        It then accordingly modifies the next state command that is returned to kernel on every frame
        :return: The decisions to be made in the next time frame
        """
        self.update_board_zones(state.zones)
        if self.has_ammo:
            if self.is_hp_zone_active():
                if self.has_buff:
                    if self.is_at_centre:
                        self.wait()
                    else:
                        self.move_to(self.centre_waypoint)
                else:
                    self.move_to(self.buff_waypoint)
            else:
                if self.is_at_centre:
                    self.wait()
                else:
                    self.move_to(self.centre_waypoint)
        else:
            if self.is_ammo_zone_active():
                self.rush_to(self.ammo_waypoint)
            else:
                self.rush_to(self.spawn_waypoint)

        return self.next_state_commands

    def update_board_zones(self, zones):
        """
        Updates the Actor's brain with known values of the buff/debuff zones
        :return:
        """
        """
        TODO Dummy function
        Can either be called on every 60, 120 and 180 second time mark if competition time info is passed
        to the robot, or manually checking if there is a mismatch between Actor brain and board zone's as passed in
        by outpost/competition info, and updating Actor brain accordingly
        """
        self.zones = zones

    def scan_for_enemies(self):
        """
        TODO scans the nearby vicinity of the robot using LiDAR + Camera implementation and returns a list
        of enemies that can be aimed at
        :return:
        """
        return {}

    def aim_then_shoot(self, enemies_found):
        """
        TODO Given the list of enemies that are nearby, modify the next_state_command such that the robot
        :param enemies_found:
        :return:
        """
        pass

    def wait(self):
        """
        TODO Scan's the robot's nearby environment for enemies to shoot, does not move
        :return:
        """
        scanned_enemies = self.scan_for_enemies()
        if scanned_enemies is not None:
            self.aim_then_shoot(scanned_enemies)
        else:
            # Does not make changes to next_state_command
            pass

    def move_to(self, waypoint):
        """
        TODO Scans the robot's nearby environment for enemies to shoot and sets the robots next_state_commands
        such that it moves towards its set waypoint
        :param waypoint:
        :return:
        """
        scanned_enemies = self.scan_for_enemies()
        if scanned_enemies is not None:
            self.aim_then_shoot(scanned_enemies)
        else:
            # TODO Insert navigation implementation
            # self.navigate(state, self.current_waypoint, self.destination)
            pass

    def rush_to(self, waypoint):
        """
        TODO Sets robot to navigate to the specified waypoint without checking for enemies
        :param waypoint:
        :return:
        """
        pass

    def is_ammo_zone_active(self):
        """
        TODO Checks if the supply zone is has not been activated yet from the current board zone info
        :return:
        """
        if self.is_blue:
            return self.zones.is_zone_active('ammo_blue')
        else:
            return self.zones.is_zone_active('ammo_red')

    def is_hp_zone_active(self):
        """
        TODO Checks if the ammo zone has not been activated yet from the current board zone info
        :return:
        """
        if self.is_blue:
            return self.zones.is_zone_active('hp_blue')
        else:
            return self.zones.is_zone_active('hp_red')


