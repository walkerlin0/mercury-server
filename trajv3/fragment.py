import math
from abc import abstractmethod

import numpy as np

from helper import Math
from trajv3.model import FragmentInput, FragmentOutput, Stage, Environment


def get_fragment(inp: FragmentInput, env: Environment):
    # dt = (inp.end_frame - inp.start_frame) / stage.frame_rate
    jgr = JumpGoRoundFragment(inp, env)
    if jgr.is_suitable():
        return jgr
    jgs = JumpGoStraightFragment(inp, env)
    if jgs.is_suitable():
        return jgs
    return ThrowFragment(inp, env)
    #
    # p = dt / inp.frequent_dt
    # if p < config.traj_free_fall_ratio or inp.start_frame <= 1:


class Fragment(object):
    def __init__(self, inp: FragmentInput, env: Environment):
        self.inp = inp
        self.env = env

    @abstractmethod
    def is_suitable(self):
        pass

    @abstractmethod
    def apply(self, stage: Stage) -> FragmentOutput:
        pass


class ThrowFragment(Fragment):
    def __init__(self, inp: FragmentInput, env: Environment):
        super().__init__(inp, env)

    def is_suitable(self):
        return True

    def apply(self, stage: Stage) -> FragmentOutput:
        dt = (self.inp.end_frame - self.inp.start_frame) / self.env.frame_rate

        ball = stage.ball
        start_frame, end_frame = self.inp.start_frame, self.inp.end_frame

        pre_velocity = self.inp.pre_velocity
        start_position = self.inp.start_position
        start_velocity = Math.find_suitable_velocity(start_position, self.inp.end_position,
                                                     end_frame - start_frame, self.env.frame_rate, self.env.G)
        # print('ThrowFragment', start_velocity)

        # else:
        #     x_end = start_position[0] + start_velocity[0] * dt
        # last_velocity = self.inp.first_velocity
        # if last_velocity == (0, 0, 0):
        #     d = 0.1
        # else:
        #     d = Math.vectors_xz_angle(last_velocity, start_velocity)
        # if config.calculate_rotation:
        #     rdt = dt if not rotation_dt else rotation_dt
        #     r_end_frame = end_frame if not rotation_end_frame else rotation_end_frame
        #     d_angle_y = ((180 - d) + 180) * rdt * direction
        #     config.ball_rotation = (
        #         config.ball_rotation[0], config.ball_rotation[1] + math.radians(d_angle_y), config.ball_rotation[2])
        #     ball.rotation_euler = config.ball_rotation
        #     ball.keyframe_insert(data_path="rotation_euler", frame=r_end_frame)
        ball_location = start_position
        cur_velocity = start_velocity
        for frame in range(start_frame + 1, end_frame + 1):
            x = ball_location[0] + cur_velocity[0] / self.env.frame_rate
            y = ball_location[1] + cur_velocity[1] / self.env.frame_rate
            z = ball_location[2] + cur_velocity[
                2] / self.env.frame_rate + 0.5 * self.env.G / self.env.frame_rate ** 2
            cur_velocity = cur_velocity[0], cur_velocity[1], cur_velocity[
                                                                 2] + self.env.G / self.env.frame_rate
            # ball.location = (x, y, z)
            ball_location = (x, y, z)
            ball.keyframe_insert(data_path="location", value=ball_location, frame=frame)

        d = Math.angle_between_vectors_xz(pre_velocity, start_velocity)
        if pre_velocity[0] < 0 and start_velocity[0] < 0:
            d = d + 180
            if d > 180:
                d -= 360
        elif pre_velocity[0] > 0 > start_velocity[0]:
            if d < 0:
                d += 180
        elif pre_velocity[0] < 0 < start_velocity[0]:
            if d > 0:
                d -= 180
        if self.inp.bump:
            pos = self.inp.start_position
            radius = stage.get_ball_radius()
            loc = Math.find_perpendicular_point(pos, radius, d)
            loc_jump = Math.find_perpendicular_point(pos, radius + 0.28, d)
            loc_support = Math.find_perpendicular_point(pos, radius + 0.5, d)
            dist = stage.support.location[1] - stage.pad.location[1]
            loc_support = loc_support[0], loc_support[1] + dist, loc_support[2]
            obj_pad = stage.copy_obj(stage.pad, location=loc,
                                     rotation_euler=(0, math.radians(-d), 0))
            stage.copy_obj(stage.support, location=loc_support,
                           rotation_euler=(math.radians(-90 - d), 0, math.radians(90)))
            # set pad jump animation
            obj_pad.keyframe_insert(data_path="location", frame=self.inp.start_frame - 1)
            obj_pad.location = loc_jump
            obj_pad.keyframe_insert(data_path="location", frame=self.inp.start_frame + 4)
            obj_pad.location = loc
            obj_pad.keyframe_insert(data_path="location", frame=self.inp.start_frame + 9)

            prop_name = 'luminous'
            obj_pad[prop_name] = 0
            obj_pad.keyframe_insert(data_path=f'["{prop_name}"]', frame=self.inp.start_frame - 1)
            obj_pad[prop_name] = stage.luminous
            obj_pad.keyframe_insert(data_path=f'["{prop_name}"]', frame=self.inp.start_frame + 4)
        return FragmentOutput(ball_location, cur_velocity)


class JumpGoStraightFragment(Fragment):
    def __init__(self, inp: FragmentInput, env: Environment):
        super().__init__(inp, env)
        height = self.inp.height()
        self.throw_dt_1 = math.ceil(math.sqrt(height / 0.5 / -self.env.G) * self.env.frame_rate) / self.env.frame_rate
        self.throw_dt_2 = math.ceil(math.sqrt(height / -self.env.G) * self.env.frame_rate) / self.env.frame_rate
        self.mid_dt = self.inp.duration() - self.throw_dt_1 - self.throw_dt_2

    def is_suitable(self):
        height = self.inp.height()
        if height < 0:
            print('unable to place LongSlideFragment because height too short')
            return False
        min_time = self.throw_dt_1 + self.throw_dt_2
        # 确保比水平抛物时间多一帧以上，否则不如直接采用抛物运动
        time_request = self.inp.duration() > min_time + 0.04
        return time_request

    def apply(self, stage):
        dt = self.inp.duration()
        height = self.inp.height()
        mid_start_frame = self.inp.start_frame + round(self.throw_dt_1 * self.env.frame_rate)
        mid_end_frame = self.inp.end_frame - round(self.throw_dt_2 * self.env.frame_rate)
        normal_v_x = self.inp.width() / dt
        w_throw_2 = normal_v_x * self.throw_dt_2
        mid_start_pos = Math.position_offset(self.inp.start_position,
                                             (normal_v_x * self.throw_dt_1 * self.inp.direction(), 0, -height / 2))
        mid_end_pos = Math.position_offset(self.inp.end_position,
                                           (-w_throw_2 * self.inp.direction(), 0, height / 2))

        first_frag = ThrowFragment(FragmentInput(self.inp.start_frame, mid_start_frame,
                                                 self.inp.start_position, mid_start_pos,
                                                 self.inp.pre_velocity), self.env)
        first_result = first_frag.apply(stage)
        sec_frag = StraightMovingFragment(FragmentInput(mid_start_frame, mid_end_frame,
                                                        first_result.position, mid_end_pos,
                                                        first_result.velocity), self.env)
        sec_result = sec_frag.apply(stage)
        third_frag = ThrowFragment(FragmentInput(mid_end_frame, self.inp.end_frame,
                                                 sec_result.position, self.inp.end_position,
                                                 sec_result.velocity, bump=False), self.env)
        return third_frag.apply(stage)


class JumpGoRoundFragment(Fragment):
    def __init__(self, inp: FragmentInput, env: Environment):
        super().__init__(inp, env)
        height = self.inp.height()
        self.throw_dt_1 = math.ceil(math.sqrt(height / 0.5 / -self.env.G) * self.env.frame_rate) / self.env.frame_rate
        self.throw_dt_2 = math.ceil(math.sqrt(height / -self.env.G) * self.env.frame_rate) / self.env.frame_rate
        self.mid_dt = self.inp.duration() - self.throw_dt_1 - self.throw_dt_2

    def is_suitable(self):
        height = self.inp.height()
        if height < 0:
            print('unable to place LongSlideFragment because height too short')
            return False
        min_time = self.throw_dt_1 + self.throw_dt_2
        # 确保比水平抛物时间多一帧以上，否则不如直接采用抛物运动
        time_request = self.inp.duration() > min_time + 0.04

        max_vx = self.inp.max_vx()
        max_mid_len = abs(max_vx * self.mid_dt)
        max_width_throw = max_vx * self.throw_dt_2
        circle_r = 2.5
        long_less_len = 2 * circle_r * math.pi + max_width_throw
        distance_request = max_mid_len > long_less_len
        return time_request and distance_request

    def apply(self, stage: Stage) -> FragmentOutput:
        dt = self.inp.duration()
        height = self.inp.height()
        mid_start_frame = self.inp.start_frame + round(self.throw_dt_1 * self.env.frame_rate)
        mid_end_frame = self.inp.end_frame - round(self.throw_dt_2 * self.env.frame_rate)
        # 如果速度上去也不够用，就让速度慢下来
        max_vx = self.inp.max_vx()
        max_mid_len = abs(max_vx * self.mid_dt)
        max_width_throw = max_vx * self.throw_dt_2
        circle_r = 2.5
        normal_v_x = self.inp.width() / dt
        w_throw_2 = normal_v_x * self.throw_dt_2
        long_less_len = 2 * circle_r * math.pi + max_width_throw
        vx = abs(self.inp.pre_velocity[0])
        w_throw_2 = vx * self.throw_dt_2
        w_throw_1 = vx * self.throw_dt_1
        mid_l = vx * self.mid_dt
        mid_top_l = (mid_l - 2 * math.pi * circle_r + w_throw_2 + self.inp.width() - w_throw_2 - circle_r * 2) / 2
        mid_bottom_l = mid_top_l - w_throw_2 - (self.inp.width() - w_throw_2 - circle_r * 2)
        mid_start_pos = Math.position_offset(self.inp.start_position,
                                             (w_throw_1 * self.inp.direction(), 0, -height / 2))
        mid_end_pos = Math.position_offset(self.inp.end_position,
                                           (w_throw_2 * self.inp.direction(), 0, -height / 2))

        first_frag = ThrowFragment(FragmentInput(self.inp.start_frame, mid_start_frame,
                                                 self.inp.start_position, mid_start_pos,
                                                 self.inp.pre_velocity), self.env)
        first_result = first_frag.apply(stage)

        cur_start_frame = mid_start_frame
        frame = math.ceil((0.5 * math.pi * circle_r) / mid_l * self.mid_dt * self.env.frame_rate)
        # print(cur_start_frame, frame)
        # print('input v', self.inp.pre_velocity)
        center = Math.position_offset(mid_start_pos, (0, circle_r, 0))
        end_pos = Math.position_offset(center, (self.inp.direction() * circle_r, 0, 0))
        tmp_result = CircleMovingFragment(
            FragmentInput(cur_start_frame, cur_start_frame + frame, mid_start_pos, end_pos, first_result.velocity),
            self.env,
            center, circle_r, -self.inp.direction(), 1).apply(stage)

        center = Math.position_offset(center, (self.inp.direction() * 2 * circle_r, 0, 0))
        start_pos = tmp_result.position
        end_pos = Math.position_offset(start_pos, (self.inp.direction() * circle_r, circle_r, 0))
        cur_start_frame = cur_start_frame + frame
        frame = math.ceil((0.5 * math.pi * circle_r) / mid_l * self.mid_dt * self.env.frame_rate)
        tmp_result = CircleMovingFragment(
            FragmentInput(cur_start_frame, cur_start_frame + frame, start_pos, end_pos, tmp_result.velocity), self.env,
            center, circle_r, self.inp.direction(), 1).apply(stage)

        start_pos = tmp_result.position
        end_pos = Math.position_offset(start_pos, (self.inp.direction() * mid_top_l, 0, 0))
        cur_start_frame = cur_start_frame + frame
        frame = int(mid_top_l / mid_l * self.mid_dt * self.env.frame_rate)
        # print(cur_start_frame, frame)
        tmp_result = StraightMovingFragment(
            FragmentInput(cur_start_frame, cur_start_frame + frame, start_pos, end_pos, tmp_result.velocity)).apply(
            stage)

        start_pos = tmp_result.position
        cur_start_frame += frame
        frame = math.ceil((math.pi * circle_r) / mid_l * self.mid_dt * self.env.frame_rate)
        end_pos = Math.position_offset(start_pos, (0, -2 * circle_r, 0))
        center = Math.position_offset(start_pos, (0, -circle_r, 0))
        # print(cur_start_frame, mid_end_frame)
        # cur_start_frame += frame
        # frame = (math.pi * circle_r) / long_less_len * mid_dt * self.env.frame_rate
        tmp_result = CircleMovingFragment(
            FragmentInput(cur_start_frame, cur_start_frame + frame, start_pos, end_pos, tmp_result.velocity),
            self.env, center, circle_r, self.inp.direction(), 2).apply(stage)
        if mid_bottom_l > 0:
            start_pos = tmp_result.position
            end_pos = Math.position_offset(start_pos, (-self.inp.direction() * mid_bottom_l, 0, 0))
            cur_start_frame = cur_start_frame + frame
            sec_result = StraightMovingFragment(
                FragmentInput(cur_start_frame, mid_end_frame, start_pos, end_pos,
                              tmp_result.velocity), self.env, support=False).apply(
                stage)
        else:
            sec_result = tmp_result
        third_frag = ThrowFragment(FragmentInput(mid_end_frame, self.inp.end_frame,
                                                 sec_result.position, self.inp.end_position,
                                                 sec_result.velocity, bump=False), self.env)
        return third_frag.apply(stage)


class StraightMovingFragment(Fragment):

    def __init__(self, inp: FragmentInput, env: Environment, support=True):
        self.support = support
        super().__init__(inp, env)

    def is_suitable(self):
        return True

    def apply(self, stage) -> FragmentOutput:
        dt = (self.inp.end_frame - self.inp.start_frame) / self.env.frame_rate
        start_position = self.inp.start_position
        start_frame = self.inp.start_frame
        end_frame = self.inp.end_frame
        d_width = self.inp.pre_velocity[0] * dt
        # second_phase_end_position = (
        #     start_position[0] + d_width, start_position[1], start_position[2])
        vx = (self.inp.end_position[0] - start_position[0]) / dt
        vy = (self.inp.end_position[2] - start_position[2]) / dt
        # z = end_position[2] - start_position[2]
        for frame in range(start_frame + 1, end_frame + 1):
            # stage.ball.location = (
            #     start_position[0] + vx / self.env.frame_rate * (frame - start_frame),
            #     start_position[1],
            #     start_position[2] + vy / self.env.frame_rate * (frame - start_frame))
            ball_location = (
                start_position[0] + vx / self.env.frame_rate * (frame - start_frame),
                start_position[1],
                start_position[2] + vy / self.env.frame_rate * (frame - start_frame))
            stage.ball.keyframe_insert(data_path="location", value=ball_location, frame=frame)
        stage.new_straight_thin_track(start_pos=start_position, end_pos=self.inp.end_position, depth=stage.track_depth,
                                      need_support=self.support)
        return FragmentOutput(self.inp.end_position, (vx, 0, vy))


class CircleMovingFragment(Fragment):
    def __init__(self, inp: FragmentInput, env: Environment, center, radius, circle_direction, typ):
        self.center = center
        self.radius = radius
        self.circle_direction = circle_direction  # 顺时针1， 逆时针-1，
        self.typ = typ  # 1/2/3/4 画圆的部分，1为四分之一
        super().__init__(inp, env)

    def apply(self, stage: Stage) -> FragmentOutput:
        start_angle, end_angle = None, None
        vx = 0
        vy = 0
        if self.inp.pre_velocity[0] > 0 and self.circle_direction == 1:
            start_angle, end_angle = 90, 90 - 90 * self.typ
            vx, vy = 0, -self.inp.pre_velocity[0]
        elif self.inp.pre_velocity[0] > 0 and self.circle_direction == -1:
            start_angle, end_angle = -90, -90 + 90 * self.typ
            vx, vy = 0, self.inp.pre_velocity[0]
        elif self.inp.pre_velocity[0] < 0 and self.circle_direction == 1:
            start_angle, end_angle = -90, -90 - 90 * self.typ
            vx, vy = 0, -self.inp.pre_velocity[0]
        elif self.inp.pre_velocity[0] < 0 and self.circle_direction == -1:
            start_angle, end_angle = 90, 90 + 90 * self.typ
            vx, vy = 0, self.inp.pre_velocity[0]
        elif self.inp.pre_velocity[1] < 0 and self.circle_direction == 1:
            start_angle, end_angle = 0, 0 - 90 * self.typ
            vx, vy = -self.inp.pre_velocity[1], 0
        elif self.inp.pre_velocity[1] < 0 and self.circle_direction == -1:
            start_angle, end_angle = -180, -180 + 90 * self.typ
            vx, vy = self.inp.pre_velocity[1], 0
        elif self.inp.pre_velocity[1] > 0 and self.circle_direction == 1:
            start_angle, end_angle = 180, 180 - 90 * self.typ
            vx, vy = -self.inp.pre_velocity[1], 0
        elif self.inp.pre_velocity[1] > 0 and self.circle_direction == -1:
            start_angle, end_angle = 0, 0 + 90 * self.typ
            vx, vy = self.inp.pre_velocity[1], 0
        dist_vert = (stage.get_ball_radius() + stage.track_depth) * math.sin(math.radians(10))
        dist_horz = (stage.get_ball_radius() + stage.track_depth) * math.cos(math.radians(10))
        track_inner_points = []
        track_outer_points = []
        frames = self.inp.end_frame - self.inp.start_frame
        angle_v = abs(end_angle - start_angle) / frames
        for angle in np.linspace(start_angle, end_angle, abs(end_angle - start_angle) + 1):
            rad = math.radians(angle)
            x = self.center[0] + self.radius * math.cos(rad)
            y = self.center[1] + self.radius * math.sin(rad)
            z = self.center[2]
            # stage.ball.location = (x, y, z)
            ball_location = (x, y, z)
            frame = int(abs(angle - start_angle) / angle_v) + self.inp.start_frame + 1
            stage.ball.keyframe_insert(data_path="location",value=ball_location, frame=frame)
            inner_point = (self.center[0] + (self.radius - dist_horz) * math.cos(rad),
                           self.center[1] + (self.radius - dist_horz) * math.sin(rad), z - dist_vert)
            outer_point = (self.center[0] + (self.radius + dist_horz) * math.cos(rad),
                           self.center[1] + (self.radius + dist_horz) * math.sin(rad), z - dist_vert)
            bottom_point = (x, y, z - stage.get_ball_radius() - stage.track_depth)
            track_inner_points.append(inner_point)
            track_outer_points.append(outer_point)
            if angle % 30 == 0:
                stage.new_hollow_tube([inner_point, bottom_point, outer_point], stage.track_depth,
                                      f'{stage.track_name}_bar')
        stage.new_hollow_tube(track_inner_points, stage.track_depth, f'{stage.track_name}_inner')
        stage.new_hollow_tube(track_outer_points, stage.track_depth, f'{stage.track_name}_outer')

        return FragmentOutput(self.inp.end_position, (vx, vy, 0))
