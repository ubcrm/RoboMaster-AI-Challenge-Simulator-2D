import pygame
import math
import typing
from game.game import Game
from game.config import CYCLES
from graphic_game.config import TEXT, COLOR, RENDER
from shared import ZoneType


class GraphicGame(Game):
    def __init__(self):
        super().__init__()
        pygame.init()
        self._screen = pygame.display.set_mode(RENDER.screen_dims)
        pygame.display.set_caption(TEXT.title)
        pygame.display.set_icon(RENDER.logo)
        pygame.font.init()
        self._font = pygame.font.SysFont(*TEXT.font)

    def render(self):
        self._blit()
        pygame.display.flip()

    def _blit(self):
        self._screen.blit(*RENDER.background)
        for zone in self._zones:
            self._blit_zone(zone)
        for robot in self._blue_robots:
            self._blit_robot(robot)
        for robot in self._red_robots:
            self._blit_robot(robot)
        for bullet in self._bullets:
            self._blit_bullet(bullet)
        self._blit_text(self._state.time_remaining, TEXT.info_coords[0])
        self._blit_text(self._state.blue_state.damage_output, TEXT.info_coords[1])
        self._blit_text(self._state.red_state.damage_output, TEXT.info_coords[2])
        self._blit_text(self._state.winner.name, TEXT.info_coords[3])

    def _blit_zone(self, zone: 'Zone'):
        if zone.is_activated:
            return
        image = RENDER.zones[zone.type_]
        rect = image.get_rect()
        rect.center = zone.box.center.to_top_left(offset=RENDER.offset)
        self._screen.blit(image, rect)

    def _blit_robot(self, robot: 'Robot'):
        if not robot.hp:
            chassis_image = RENDER.dead_robot
        elif robot.is_blue:
            chassis_image = RENDER.blue_robot
        else:
            chassis_image = RENDER.red_robot
        chassis_image = pygame.transform.rotate(chassis_image, math.degrees(robot.rotation))
        gimbal_image = pygame.transform.rotate(RENDER.gimbal, math.degrees(robot.gimbal_yaw + robot.rotation))
        chassis_rect = chassis_image.get_rect()
        gimbal_rect = gimbal_image.get_rect()
        chassis_rect.center = gimbal_rect.center = robot.center.to_top_left(offset=RENDER.offset)
        self._screen.blit(chassis_image, chassis_rect)
        self._screen.blit(gimbal_image, gimbal_rect)
        self._blit_text(f'{robot.id_} | {robot.hp}', robot.center.to_top_left(offset=TEXT.robot_label_offset),
                        COLOR.blue if robot.is_blue else COLOR.red)
        self._blit_robot_status(robot)

    def _blit_robot_status(self, robot: 'Robot'):
        row_coords = TEXT.state_coords[robot.id_]
        self._blit_text(f'{robot.center.x:.0f}', row_coords[0])
        self._blit_text(f'{robot.center.y:.0f}', row_coords[1])
        self._blit_text(f'{math.degrees(robot.rotation):.0f}', row_coords[2])
        self._blit_text(f'{math.degrees(robot.gimbal_yaw):.0f}', row_coords[3])
        self._blit_text(f'{robot.speed.x:.2f}', row_coords[4])
        self._blit_text(f'{robot.speed.y:.2f}', row_coords[5])
        self._blit_text(f'{math.degrees(robot.rotation_speed):.2f}', row_coords[6])
        self._blit_text(f'{math.degrees(robot.gimbal_yaw_speed):.2f}', row_coords[7])
        self._blit_text(f'{robot.ammo}', row_coords[8])
        self._blit_text(f'{robot.heat}', row_coords[9])
        self._blit_text(f'{robot.hp}', row_coords[10])
        self._blit_text(f'{robot.is_shooting}', row_coords[11])
        self._blit_text(f'{robot.shot_cooldown_cycles / CYCLES.second:.3f}', row_coords[12])
        self._blit_text(f'{robot.barrier_hits}', row_coords[13])
        self._blit_text(f'{robot.robot_hits}', row_coords[14])
        self._blit_text(f'{robot.can_move}', row_coords[15])
        self._blit_text(f'{robot.can_shoot}', row_coords[16])
        self._blit_text(f'{robot.debuff_timeout_cycles / CYCLES.second: .0f}', row_coords[17])

    def _blit_bullet(self, bullet: 'Bullet'):
        bullet_rect = RENDER.bullet.get_rect()
        bullet_rect.center = bullet.center.to_top_left(offset=RENDER.offset)
        self._screen.blit(RENDER.bullet, bullet_rect)

    def _blit_text(self, text, position: typing.Tuple[int, ...], color=COLOR.black):
        label = self._font.render(str(text), True, color)
        self._screen.blit(label, position)


from game.bullet import Bullet
from game.robot import Robot
from game.zone import Zone
