import pygame
from game.geometry import Vector
from shared import IMAGE_DIR, ZoneType


class TEXT:
    title = 'UBC RoboMaster AI Challenge Simulator'
    font = 'arial', 11
    robot_label_offset = (-10, 45)
    state_coords = [[(72 + 41 * i, 516 + 17 * j) for i in range(18)] for j in range(4)]
    info_coords = [(195 + 4 * 41 * i, 482) for i in range(4)]


class RENDER:
    offset = (10, -10)
    screen_dims = (828, 638)
    background = pygame.image.load(IMAGE_DIR / 'background.png'), (0, 0)
    logo = pygame.image.load(IMAGE_DIR / 'logo.png')
    bullet = pygame.image.load(IMAGE_DIR / 'robot/bullet.png')
    blue_robot = pygame.image.load(IMAGE_DIR / 'robot/blue.png')
    red_robot = pygame.image.load(IMAGE_DIR / 'robot/red.png')
    dead_robot = pygame.image.load(IMAGE_DIR / 'robot/dead.png')
    gimbal = pygame.image.load(IMAGE_DIR / 'robot/gimbal.png')
    zones = {
        ZoneType.blue_hp_buff: pygame.image.load(IMAGE_DIR / 'zone/blue_hp_buff.png'),
        ZoneType.red_hp_buff: pygame.image.load(IMAGE_DIR / 'zone/red_hp_buff.png'),
        ZoneType.blue_ammo_buff: pygame.image.load(IMAGE_DIR / 'zone/blue_ammo_buff.png'),
        ZoneType.red_ammo_buff: pygame.image.load(IMAGE_DIR / 'zone/red_ammo_buff.png'),
        ZoneType.move_debuff: pygame.image.load(IMAGE_DIR / 'zone/move_debuff.png'),
        ZoneType.shoot_debuff: pygame.image.load(IMAGE_DIR / 'zone/shoot_debuff.png')
    }


class COLOR:
    blue = (0, 0, 210)
    red = (210, 0, 0)
    silver = (182, 186, 187)
    black = (0, 0, 0)
