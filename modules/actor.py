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

    def take_action(self):
        """
        Called on every frame by the Actor, it first updates the board state as stored in Actor memory
        Then it checks the current robot state to determine what the next line of action should be.
        It then accordingly modifies the next state command that is returned to kernel on every frame
        :return: The decisions to be made in the next time frame
        """
        self.update_board_zones()
        if self.has_ammo:
            if self.is_buff_zone_active():
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
            if self.is_supply_zone_active():
                self.rush_to(self.ammo_waypoint)
            else:
                self.rush_to(self.spawn_waypoint)

        return self.next_state_commands

    def update_board_zones(self):
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
        pass

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

    def set_waypoint(self, waypoint):
        """
        TODO Marks the specified waypoint as a point to navigate to in the robot's internal navigation system
        :param waypoint:
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
            pass

    def rush_to(self, waypoint):
        """
        TODO Sets robot to navigate to the specified waypoint without checking for enemies
        :param waypoint:
        :return:
        """
        pass

    def is_supply_zone_active(self):
        """
        TODO Checks if the supply zone is has not been activated yet from the current board zone info
        :return:
        """
        pass

    def is_buff_zone_active(self):
        """
        TODO Checks if the ammo zone has not been activated yet from the current board zone info
        :return:
        """
        pass


