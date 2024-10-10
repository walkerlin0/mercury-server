import math

try:
    import config
except:
    from .. import config


class FragmentInput(object):
    #
    def __init__(self, start_frame, end_frame, start_position, end_position, pre_velocity, bump=True):
        """
        :param start_frame: 起始帧，此帧不需要计算动画
        :param end_frame: 结束帧，此帧需要计算动画
        :param start_position: 球在起始帧的位置
        :param end_position: 期望到达的位置
        :param pre_velocity: 球在起始帧前的速度
        """
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.pre_velocity = pre_velocity
        self.start_position = start_position
        self.end_position = end_position
        self.bump = bump  # if start with bump

    def height(self):
        # 落差，下落为正值，上升为负值
        return self.start_position[2] - self.end_position[2]

    def width(self):
        return abs(self.start_position[0] - self.end_position[0])

    def direction(self):
        return 1 if self.start_position[0] < self.end_position[0] else -1

    def direction_x(self):
        return 1 if self.start_position[0] < self.end_position[0] else -1

    def direction_y(self):
        return 1 if self.start_position[1] < self.end_position[1] else -1

    def duration(self):
        return (self.end_frame - self.start_frame) / config.FRAME_RATE

    def max_vx(self):
        return math.sqrt(
            self.pre_velocity[0] ** 2 + self.pre_velocity[1] ** 2 + self.pre_velocity[2] ** 2)


class Animation(object):
    def __init__(self, obj_name, frame2location=None, frame2rotation=None):
        if frame2rotation is None:
            frame2rotation = {}
        if frame2location is None:
            frame2location = {}
        self.obj_name = obj_name
        self.frame2location = frame2location
        self.frame2rotation = frame2rotation

    def update(self, ani):
        self.frame2location.update(ani.frame2location)
        self.frame2rotation.update(ani.frame2rotation)


class Item(object):
    def __init__(self):
        pass


class Tube(Item):
    def __init__(self, positions, depth):
        super().__init__()
        self.positions = positions
        self.depth = depth


class CopyItem(Item):
    def __init__(self, obj_name, position, rotation):
        super().__init__()
        self.obj_name = obj_name
        self.position = position
        self.rotation = rotation


class FragmentOutput(object):
    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity
        self.animations = []
        self.items = []
        self.copy_items = []


class Stage(object):
    def __init__(self, ball, support, pad, assist_collection_name, track_material_name, track_depth=0.1,
                 track_name='track'):
        self.ball = ball
        self.support = support
        self.pad = pad
        self.assist_collection_name = assist_collection_name
        self.track_material_name = track_material_name
        self.track_depth = track_depth
        self.track_name = track_name

    def get_ball_radius(self):
        return config.ball_radius
