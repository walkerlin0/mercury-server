import math

import numpy as np


class Math:
    @classmethod
    def initial_vertical_velocity(cls, h, t, g):
        return (h / t) - (0.5 * g * t)

    @classmethod
    def find_suitable_velocity(cls, cur_pos, next_loc, dis_frames, frame_rate, g):
        dx = next_loc[0] - cur_pos[0]
        dz = next_loc[2] - cur_pos[2]
        dt = dis_frames / frame_rate
        vx = dx / dt
        vy = 0
        vz = Math.initial_vertical_velocity(dz, dt, g)
        return (vx, vy, vz)

    # 计算合并向量的角度
    @classmethod
    def angle_between_vectors_xz(cls, vec1, vec2):
        n1 = np.linalg.norm(vec1)
        n2 = np.linalg.norm(vec2)
        v = tuple([vec1[i] / n1 + vec2[i] / n2 for i in range(len(vec1))])
        return math.degrees(math.atan2(v[2], v[0]))

    @classmethod
    def vectors_xz_angle(cls, vec1, vec2):
        a1 = math.atan2(vec1[2], vec2[0])
        a2 = math.atan2(vec2[2], vec2[0])
        if a1 > 0 and a2 < 0:
            da = a1 - a2
        elif a2 > 0 and a1 < 0:
            da = a2 - a1
        elif a2 > 0 and a1 > 0:
            da = abs(a2 - a1)
        else:
            da = abs(abs(a2) - abs(a1))
        if da > math.pi:
            da = 2 * math.pi - da
        return da * 180 / math.pi

    @classmethod
    def find_perpendicular_point(cls, circle_center, circle_radius, line_degree):
        rad = math.radians(line_degree)
        xc, yc = circle_center[0], circle_center[2]

        xp = xc + circle_radius * math.sin(rad)
        yp = yc - circle_radius * math.cos(rad)
        return (xp, 0, yp)

    @classmethod
    def almost_equal(cls, a, b, acc=0.005):
        return abs(abs(a) - abs(b)) < acc

    @classmethod
    def calculate_support_position(cls, start_x, end_x):
        minx = min(start_x, end_x)
        maxx = max(start_x, end_x)
        count = (maxx - minx) / 2
        if count < 2:
            count = 2
        result = np.linspace(minx, maxx, int(count)).tolist()
        return result

    @classmethod
    def calculate_velocity_in_slope(cls, velocity, slope):
        radians = math.atan(slope)
        vx = velocity[0] * math.cos(radians)
        vz = velocity[2] * math.sin(radians)
        v = vx + vz
        return v * math.cos(radians), velocity[1], v * math.sin(radians)

    @classmethod
    def add_time_limited_track(cls, frame, position, velocity, dt, m=1, g=-9.8, frame_rate=30, x_factor=10,
                               y_factor=10):
        pi = math.pi * x_factor

        dt_frame = 1 / frame_rate
        direction = 1 if velocity[0] > 0 else -1
        factor = dt
        if factor > 1:
            factor = 1
        start_radians = pi - (pi / 4 * direction) * factor
        end_radians = pi + (pi / 4 * direction) * factor
        d_radians = 0.01

        slope = (Math.track_y(start_radians + direction * d_radians, x_factor, y_factor) - Math.track_y(start_radians,
                                                                                                        x_factor,
                                                                                                        y_factor)) / d_radians
        velocity = Math.calculate_velocity_in_slope(velocity, slope)
        energy = 0.5 * m * (velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)
        v = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)

        start_y = Math.track_y(start_radians, x_factor, y_factor)
        last_y = 0
        temp_t = 0
        total_frame = 0
        positions = []
        for radians in np.arange(start_radians, end_radians, d_radians * direction):
            y = Math.track_y(radians, x_factor)
            if last_y == 0:
                last_y = y
                continue
            dt = math.sqrt(d_radians ** 2 + (y - last_y) ** 2) / v
            energy += m * g * (y - last_y)
            v = math.sqrt(2 * energy / m)
            last_y = y
            temp_t += dt
            if temp_t > dt_frame:
                rx = radians - (start_radians if direction > 0 else end_radians)
                real_x = position[0] + rx
                if direction < 0:
                    real_x = real_x - abs(start_radians - end_radians)
                real_y = position[2] + y - start_y
                positions.append([real_x, 0, real_y])
                # print(temp_t, real_x, real_y)
                temp_t = 0
                total_frame += 1
                if total_frame >= frame:
                    break
        vend = (np.array(positions[-1]) - np.array(positions[-2])) / dt_frame
        # print(velocity, vend)
        return positions

    @classmethod
    def track_y(cls, x, x_factor, y_factor=15):
        return math.cos(x / x_factor) * y_factor

    @classmethod
    def tangent_slope_difference(cls, circle_r, distance):
        def tangent_slope(angle):
            if angle % 360 == 0:
                return None
            return -1 * math.cos(angle / 360 * 2 * math.pi) / math.sin(angle / 360 * 2 * math.pi)

        length = 2 * circle_r * math.pi
        angle = distance / length * 360

        return tangent_slope(angle + 90) - tangent_slope(90)

    @classmethod
    def find_second_point(cls, p1, slope, distance, direction):
        x1, y1 = p1[0], p1[2]
        delta_x = distance / math.sqrt(1 + slope ** 2)
        delta_y = slope * delta_x

        x2_plus = x1 + delta_x
        y2_plus = y1 + delta_y

        x2_minus = x1 - delta_x
        y2_minus = y1 - delta_y

        if direction > 0:
            return (x2_plus, 0, y2_plus)
        else:
            return (x2_minus, 0, y2_minus)

    @classmethod
    def slope(cls, p1, p2):
        return (p2[2] - p1[2]) / (p2[0] - p1[0])

    @classmethod
    def smooth_extend_previous_track(cls, positions, min_radius):
        start = positions[0]
        sec = positions[1]
        distance = math.sqrt((start[0] - sec[0]) ** 2 + (start[2] - sec[2]) ** 2)
        d_slope = abs(Math.tangent_slope_difference(min_radius, distance))
        direction = 1 if start[0] < sec[0] else -1
        ext = []
        for i in range(10):
            slope = Math.slope(start, sec)
            target_slope = slope + direction * d_slope
            p2 = Math.find_second_point(start, target_slope, distance, - direction)
            # print(slope, target_slope, p2)
            ext.append(p2)
            sec = start
            start = p2

        ext.reverse()
        ext.extend(positions)
        return ext

    @classmethod
    def position_offset(cls, position, offset):
        return position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]

    @classmethod
    def distance(cls, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)
