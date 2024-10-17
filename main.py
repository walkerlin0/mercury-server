from flask import Flask, request

from layout import layout
from trajv3 import trajectory

app = Flask(__name__)


def resp(data):
    return {
        'code': 0,
        'data': data
    }


@app.route('/layout', methods=['POST'])
def layout_req():
    data = request.json
    bpm = data.get('bpm')
    vx = data.get('vx')
    vy = data.get('vy')
    frame_rate = data.get('frame_rate')
    rhythm_times = data.get('rhythm_times')
    return resp(layout(rhythm_times, bpm, frame_rate, vx, vy))


@app.route('/trajectory', methods=['POST'])
def trajectory_req():
    data = request.json
    g = data.get('g')
    frame_rate = data.get('frame_rate')
    rhythm_times = data.get('rhythm_times')
    positions = data.get('positions')
    ball_radius = data.get('ball_radius', 1)
    luminous = data.get('luminous', 5)
    pad_loc = data.get('pad_loc', (0, 1, 2))
    support_loc = data.get('support_loc', (3, 4, 5))
    result = trajectory.get(rhythm_times, positions, frame_rate, g, ball_radius, luminous, pad_loc, support_loc)
    return resp([r.to_dict() for r in result])


if __name__ == '__main__':
    app.run()
