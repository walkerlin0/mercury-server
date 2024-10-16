import math

from helper import Math

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


class FragmentOutput(object):
    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity
        self.animations = []
        self.items = []
        self.copy_items = []


class BlenderObject(dict):
    def __init__(self, typ=None, copy_from=None, location=None, rotation_euler=None):
        super().__init__()
        self.typ = typ
        if copy_from:
            self.typ = copy_from.typ
        if not self.typ:
            raise "must set a type to blender object"
        self.location = location
        self.rotation_euler = rotation_euler
        self.copy_from = copy_from
        self.data_path2frame2value = {}

    def keyframe_insert(self, data_path, value, frame):
        # value = self.get(data_path, None)
        # if not value:
        #     raise 'data_path not exist'
        self.data_path2frame2value[data_path][frame] = value

    def get_animation(self):
        return self.data_path2frame2value


class Ball(BlenderObject):
    def __init__(self, copy_from=None, location=None, rotation_euler=None):
        super().__init__('ball', copy_from, location, rotation_euler)


class Tube(BlenderObject):
    def __init__(self, copy_from=None, location=None, rotation_euler=None, points=None, depth=None):
        super().__init__('tube', copy_from, location, rotation_euler)
        self.points = points
        self.depth = depth


class TrackSupport(BlenderObject):
    def __init__(self, copy_from=None, location=None, rotation_euler=None, depth=None):
        super().__init__('track_support', copy_from, location, rotation_euler)
        self.depth = depth


class Stage(object):
    def __init__(self, ball: Ball, support, pad, ball_radius, luminous, track_depth=0.1):
        self.ball = ball
        self.support = support
        self.pad = pad
        self.track_depth = track_depth
        self.ball_radius = ball_radius
        self.luminous = luminous

    def get_ball_radius(self):
        return self.ball_radius

    def copy_obj(self, obj, location, rotation_euler):
        return BlenderObject(copy_from=obj, location=location, rotation_euler=rotation_euler)

    def new_straight_thin_track(self, start_pos, end_pos, depth, need_support):
        min_len = 2
        if abs(start_pos[0] - end_pos[0]) < min_len:
            start_pos = (end_pos[0] + min_len * (1 if start_pos[0] > end_pos[0] else -1), start_pos[1], start_pos[2])
        dist_vert = (self.ball_radius + depth) * math.sin(math.radians(10))
        dist_horz = (self.ball_radius + depth) * math.cos(math.radians(10))
        bottom_points = [(start_pos[0], start_pos[1], start_pos[2] - self.ball_radius - depth),
                         (end_pos[0], end_pos[1], end_pos[2] - self.ball_radius - depth)]
        inner_points = [(start_pos[0], start_pos[1] + dist_horz, start_pos[2] - dist_vert),
                        (end_pos[0], end_pos[1] + dist_horz, end_pos[2] - dist_vert)]
        outer_points = [(start_pos[0], start_pos[1] - dist_horz, start_pos[2] - dist_vert),
                        (end_pos[0], end_pos[1] - dist_horz, end_pos[2] - dist_vert)]
        # Bpy.new_hollow_tube(bottom_points, depth, f'{name}_bottom', collection_name, material_name)
        self.new_hollow_tube(inner_points, depth)
        self.new_hollow_tube(outer_points, depth)
        positions = Math.calculate_support_position(start_pos[0], end_pos[0])
        for pos in positions:
            inner_pos = (pos, start_pos[1] + dist_horz, start_pos[2] - dist_vert)
            self.new_hollow_tube([
                (pos, start_pos[1] - dist_horz, start_pos[2] - dist_vert),
                (pos, start_pos[1], start_pos[2] - self.ball_radius - depth),
                inner_pos],
                depth)
            if need_support:
                self.add_track_support(inner_pos, depth)

    def new_hollow_tube(self, points, depth):
        return Tube(points, depth)

    def add_track_support(self, location, depth):
        return TrackSupport(location=location, depth=depth)


class Environment(object):
    def __init__(self, G, frame_rate):
        self.G = G
        self.frame_rate = frame_rate
