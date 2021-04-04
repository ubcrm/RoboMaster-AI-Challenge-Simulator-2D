import numpy as np
import pathlib

# Indices of important robot properties in state.agents[car_num]
from modules.geometry import Rectangle, Line
from modules.robot import Robot
from modules.waypoints.navigator import NavigationGraph
from modules.waypoints.navigator import Navigator

OWNER = 0
POS_X = 1
POS_Y = 2
ANGLE = 3
YAW = 4
BULLET_COUNT = 10

B5 = Rectangle(35.4, 35.4, 0, 0, image='images/area/lcm.png')
B2 = Rectangle(80, 20, -214, 0, image='images/area/lhm.png')
B1 = Rectangle(100, 20, -354, 114, image='images/area/hhu.png')
B3 = Rectangle(20, 100, -244, -174, image='images/area/hvu.png')
B4 = Rectangle(100, 20, 0, 120.5, image='images/area/hhm.png')

low_barriers = [B2, B2.mirror(), B5]  # areas B2, B5, B8
high_barriers = [B1, B3, B4, B4.mirror(), B3.mirror(), B1.mirror()]  # areas B1, B3, B4, B6, B7, B9

barrier_vertices = []


'''
Returns the determinant of the matrix made from two 2d column vectors, det((v0 v1))
'''
def det(v0, v1):
    return v0[0]*v1[1] - v1[0]*v0[1]


class Actor:
    def __init__(self, car_num, robot):
        self.car_num = car_num
        self.is_blue = robot.is_blue
        self.prev_commands = None
        self.next_waypoint = None
        self.destination = None
        self.nav = Navigator(pathlib.Path('modules', 'waypoints', 'data.json'))
        self.robot = robot
        self.state = None
        self.barriers = low_barriers + high_barriers
        self.next_state_commands = [0, 0, 0, 0, 0, 0, 0]

        # TODO replace dummy values with proper ones representing starting state

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
        self.state = state
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
        """
        Update the robots (x,y) destination co-ordinates
        """
        self.destination = self.nearest_waypoint(dest)

    def navigate(self, state, from_waypoint, to_waypoint, avoid_nodes=None):
        """
        Pathfind to the destination. Returns the x,y,rotation values
        """
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
        if self.state.time == 0 or self.state.time == 60 or self.state.time == 120:
            self.ammo_waypoint = self.state.zones.get_index_by_type()
            self.buff_waypoint = 0
            # self.
        pass

    def scan_for_enemies(self):
        """
        Scans the nearby vicinity of the robot using LiDAR + Camera implementation and returns the id of the first
        robot that is visible. Returns None if no robots are visible
        """
        camera_enemies = self.get_camera_vision()
        if camera_enemies == -1:
            lidar_enemies = self.get_lidar_vision()
            if lidar_enemies == -1:
                return None
            else:
                return lidar_enemies
        else:
            return camera_enemies

    def aim_then_shoot(self, scanned_enemy):
        """
        Makes the robot chassis aim towards the robot specified. If the robot is already locked on to the enemy
        in a straight line, it modifies the next state to shoot
        :param scanned_enemy:
        :return:
        """
        theta = np.rad2deg(np.arctan(45 / 60))
        delta_x, delta_y = self.state.robots[scanned_enemy].center - self.robot.center
        relative_angle = np.angle(delta_x + delta_y * 1j, deg=True) - self.state.robots[scanned_enemy].rotation
        # Normalize angle
        if relative_angle >= 180: relative_angle -= 360
        if relative_angle <= -180: relative_angle += 360

        if -theta <= relative_angle < theta:
            armor = self.get_relative_robot_vertices(self.state.robots[scanned_enemy], 2)
        elif theta <= relative_angle < 180 - theta:
            armor = self.get_relative_robot_vertices(self.state.robots[scanned_enemy], 3)
        elif -180 + theta <= relative_angle < -theta:
            armor = self.get_relative_robot_vertices(self.state.robots[scanned_enemy], 1)
        else:
            armor = self.get_relative_robot_vertices(self.state.robots[scanned_enemy], 0)
        delta_x, delta_y = armor - self.robot.center
        adjustment_angle = np.angle(delta_x + delta_y * 1j, deg=True) - self.robot.yaw - self.robot.rotation
        if adjustment_angle >= 180: adjustment_angle -= 360
        if adjustment_angle <= -180: adjustment_angle += 360



        pass

    def get_relative_robot_vertices(self, robot, direction):
        """
        Helper function for aiming at another robot
        Returns transformed vertices of the other robot
        """
        rotate_matrix = np.array([[np.cos(-np.deg2rad(robot.rotation + 90)),
                                   -np.sin(-np.deg2rad(robot.rotation + 90))],
                                  [np.sin(-np.deg2rad(robot.rotation + 90)),
                                   np.cos(-np.deg2rad(robot.rotation + 90))]])
        xs = np.array([[0, -30], [18.5, 0], [0, 30], [-18.5, 0]])
        return np.matmul(xs[direction], rotate_matrix) + robot.center

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
        if self.is_blue:
            pass
            #
        else:
            # TODO Check for red team ammo
            pass

        pass

    def get_lidar_vision(self):
        """
        Check's if the current acting robot can see other robots using camera vision, and returns id of the first
        visible robot
        Returns -1 if no robot can be seen
        """
        robot_id = self.robot.id_
        robot_coordinates = self.robot.center
        for offset_index in range(len(self.state.robots - 1)):
            other_robot_coordinates = self.state.robots[robot_id - offset_index - 1].center
            delta_x, delta_y = other_robot_coordinates - robot_coordinates
            angle = np.angle(delta_x + delta_y * 1j, deg=True)
            if angle >= 180: angle -= 360
            if angle <= -180: angle += 360
            angle = angle - self.robot.rotation
            if angle >= 180: angle -= 360
            if angle <= -180: angle += 360
            if abs(angle) < 60:
                if self.line_intersects_barriers(robot_coordinates, other_robot_coordinates) \
                        or self.line_intersects_robots(robot_coordinates, other_robot_coordinates):
                    pass
                else:
                    return self.state.robots[robot_id - offset_index - 1].id_
            else:
                pass

        return -1

    def get_camera_vision(self):
        """
        Check's if the current acting robot can see other robots using camera vision, and returns id of the first
        visible robot
        Returns -1 if no robot can be seen
        """
        robot_id = self.robot.id_
        robot_coordinates = self.robot.center
        for offset_index in range(len(self.state.robots - 1)):
            other_robot_coordinates = self.state.robots[robot_id - offset_index - 1].center
            delta_x, delta_y = other_robot_coordinates - robot_coordinates
            angle = np.angle(delta_x + delta_y * 1j, deg=True)
            if angle >= 180: angle -= 360
            if angle <= -180: angle += 360
            # Get relative angle
            angle = angle - self.robot.yaw - self.robot.rotation
            if angle >= 180: angle -= 360
            if angle <= -180: angle += 360
            if abs(angle) < 37.5:
                if self.line_intersects_barriers(robot_coordinates, other_robot_coordinates) \
                        or self.line_intersects_robots(robot_coordinates, other_robot_coordinates):
                    pass
                else:
                    return self.state.robots[robot_id - offset_index - 1].id_
            else:
                pass

        return -1

    """Low-level enemy scanning functions below"""

    def cross_product(self, p1, p2, p3):
        """
        Given 3 points p1, p2, p3, it calculates the cross product of p1->p2 and p1->p3 vectors.
        Hence it returns 0 if all 3 line on the same line, a non-zero value otherwise
        """
        x1 = p2[0] - p1[0]
        y1 = p2[1] - p1[1]
        x2 = p3[0] - p1[0]
        y2 = p3[1] - p1[1]
        return x1 * y2 - x2 * y1

    def get_robot_outline(self, robot):
        """
        Given a robot object, it returns the coordinates of its vertices based on the robot's rotation values
        """
        rotate_matrix = np.array([[np.cos(-np.deg2rad(robot.rotation + 90)),
                                   -np.sin(-np.deg2rad(robot.rotation + 90))],
                                  [np.sin(-np.deg2rad(robot.rotation + 90)),
                                   np.cos(-np.deg2rad(robot.rotation + 90))]])
        xs = np.array([[-22.5, -30], [22.5, 30], [-22.5, 30], [22.5, -30]])
        return [np.matmul(xs[i], rotate_matrix) + robot.center for i in range(xs.shape[0])]

    def segment(self, p1, p2, p3, p4):
        if (max(p1[0], p2[0]) >= min(p3[0], p4[0])
                and max(p3[0], p4[0]) >= min(p1[0], p2[0])
                and max(p1[1], p2[1]) >= min(p3[1], p4[1])
                and max(p3[1], p4[1]) >= min(p1[1], p2[1])):
            if (self.cross_product(p1, p2, p3) * self.cross_product(p1, p2, p4) <= 0
                    and self.cross_product(p3, p4, p1) * self.cross_product(p3, p4, p2) <= 0):
                return True
            else:
                return False
        else:
            return False

    def line_rect_check(self, l1, l2, sq):
        # this part code came from: https://www.jianshu.com/p/a5e73dbc742a
        # check if line cross rect, sq = [x_leftdown, y_leftdown, x_rightup, y_rightup]
        p1 = [sq[0], sq[1]]
        p2 = [sq[2], sq[3]]
        p3 = [sq[2], sq[1]]
        p4 = [sq[0], sq[3]]
        if self.segment(l1, l2, p1, p2) or self.segment(l1, l2, p3, p4):
            return True
        else:
            return False

    def line_intersects_barriers(self, point1, point2):
        """
        Given two points, it checks if the line created by those points intersect any map barrier
        """
        line = Line(point1, point2)
        for barrier in self.barriers:
            if barrier.intersects(line):
                return True
        return False

    def line_intersects_robots(self, robot_center1, robot_center2):
        """
        Given the center coordinates of two robots, it checks if any other robots are between the straight line
        connecting them
        """
        for robot in self.state.robots:
            if (robot.center == robot_center1).all() or (robot.center == robot_center2).all():
                # Performing comparison with the current robots, so pass onto next loop
                continue
            vertex1, vertex2, vertex3, vertex4 = self.get_robot_outline(robot)
            if self.segment(robot_center1, robot_center2, vertex1, vertex2) \
                    or self.segment(robot_center1, robot_center2, vertex3, vertex4):
                return True
        return False
