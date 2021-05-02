from game.geometry import Vector, LineSegment, Box, x_mirrors, y_mirrors, xy_mirrors


class FIELD:
    id_is_blue = {0: True, 1: True, 2: False, 3: False}  # blue, blue, red, red
    id_is_zero = {0: True, 1: False, 2: True, 3: False}  # 0, 1, 0, 1
    box = Box(Vector(808, 448))
    low_barriers = [  # low barriers B2, B8, B5
        *xy_mirrors(Box(Vector(80, 20), Vector(-214, 0))),
        Box(Vector(35.4, 35.4))]
    high_barriers = [  # high barriers B1, B6, B3, B7, B4, B9
        *xy_mirrors(Box(Vector(100, 20), Vector(-354, 114))),
        *xy_mirrors(Box(Vector(20, 100), Vector(-244, -174))),
        *xy_mirrors(Box(Vector(100, 20), Vector(0, 120.5)))]


class ZONE:
    dims = Vector(36, 32)
    boxes = [  # F4, F1, F5, F2, F3, F6
        *xy_mirrors(Box(dims, Vector(354, -55))),
        *xy_mirrors(Box(dims, Vector(214, 59))),
        *xy_mirrors(Box(dims, Vector(0, 179.5)))]
    hp_buff = 200
    ammo_buff = 100


class ROBOT:
    box = Box(Vector(60, 50))
    center = Vector(354, 174)
    hp = 2000
    ammo = 50

    _armor_length = 14
    armors = [  # front, left, right, back
        LineSegment(*y_mirrors(Vector(box.dims.x / 2, _armor_length / 2))),
        LineSegment(*x_mirrors(Vector(_armor_length / 2, box.dims.y / 2))),
        LineSegment(*x_mirrors(Vector(_armor_length / 2, -box.dims.y / 2))),
        LineSegment(*y_mirrors(Vector(-box.dims.x / 2, _armor_length / 2)))]
    armors_damage = [20, 40, 40, 60]

    @staticmethod
    def settle_heat(heat: int, hp: int):  # barrel heat (rules 4.1.2)
        if heat >= 360:
            hp -= (heat - 360) * 40
            heat = 360
        elif heat > 240:
            hp -= (heat - 240) * 4
        heat -= 12 if hp >= 400 else 24
        return max(heat, 0), max(hp, 0)


class MOTION:
    x_accel, x_speed = 0.25, 1.5
    y_accel, y_speed = 0.2, 1.2
    rotation_accel, rotation_speed = 0.005, 0.02
    gimbal_yaw_accel, gimbal_yaw_speed = 0.02, 0.05
    rebound_coefficient = 0.3
    gimbal_yaw_range = 4  # full range span


class BULLET:
    speed = 20
    speed_sigma = 1


class CYCLES:
    step = 10
    second = 100  # changing this will have indirect consequences
    shot_cooldown = 8
    heat_settlement = 10
    debuff_timeout = 10 * second
    zone_reset = 60 * second
    game_duration = 180 * second
