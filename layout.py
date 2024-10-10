def transfer(key_frames):
    intervals = []
    for i in range(0, len(key_frames)):
        intervals.append(key_frames[i] - (0 if i < 1 else key_frames[i - 1]))
    # itvl2count = {}
    # for itvl in intervals:
    #     itvl2count[itvl] = itvl2count.get(itvl, 0) + 1
    # a = sorted(itvl2count, key=lambda x: itvl2count[x], reverse=True)
    # usual_itvl = a[0]
    return intervals


def cal_animation_height(dt, usual_itvl, should_y_speed_fast, vy):
    height_free_fall = vy * dt * (1 if should_y_speed_fast else 0.2)
    height_ceil = vy * 1 * usual_itvl
    height = min(height_free_fall, height_ceil)
    return height


def layout(rhythm_times, bpm, frame_rate, vx, vy):
    key_frames = [int(p * frame_rate) for p in rhythm_times]
    intervals = transfer(key_frames)
    beat = 60 / bpm
    last_pos = (0, 0, 0)
    broad_direction = True  # True vertical False horizontal
    direction = 1
    result = []
    for i, itvl in enumerate(intervals):
        dt = itvl / frame_rate

        if dt < beat:
            if broad_direction:
                direction *= -1
            y_speed_fast = True
            broad_direction = not broad_direction
        else:
            if broad_direction:
                direction *= -1
                y_speed_fast = True
            else:
                y_speed_fast = False

        move_x = min(vx * dt, vx * 2 * beat) * direction
        move_y = cal_animation_height(dt, beat, y_speed_fast, vy)
        last_pos = last_pos[0] + move_x, last_pos[1], last_pos[2] - move_y
        result.append(last_pos)

    return result
