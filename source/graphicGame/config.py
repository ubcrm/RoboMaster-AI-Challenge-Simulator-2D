import pygame
from shared import IMAGE_DIR


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


class COLOR:
    blue = (0, 0, 210)
    red = (210, 0, 0)
    silver = (182, 186, 187)
    black = (0, 0, 0)
