from trajv3 import fragment
from trajv3.model import FragmentInput, Animation


def get(key_frames, positions):
    next_result = None
    obj2animation = {}
    items = []
    copy_items = []
    for i, _ in enumerate(key_frames):
        start_frame = 0 if i < 1 else key_frames[i - 1]
        end_frame = key_frames[i]
        velocity = (0, 0, 0)
        if next_result is None:
            start_position = (0, 0, 0) if i < 1 else positions[i - 1]
        else:
            start_position = next_result.position
            velocity = next_result.velocity
        end_position = positions[i]
        frag = fragment.get_fragment(
            FragmentInput(start_frame, end_frame, start_position, end_position, velocity))
        next_result = frag.apply()
        if next_result is not None:
            if next_result.items is not None:
                items.extend(next_result.items)
            if next_result.copy_items is not None:
                copy_items.extend(next_result.copy_items)
            if next_result.animations is not None:
                for ani in next_result.animations:
                    rani = obj2animation.get(ani.obj_name, Animation(ani.obj_name))
                    rani.update(ani)
                    obj2animation[ani.obj_name] = rani
    return {
        'items': items,
        'copy_items': copy_items,
        'animations': list(obj2animation.values())
    }
