from flask import Flask, request

from layout import layout

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


@app.route('/trajectory', methods=['post'])
def trajectory():
    print(request.json)
    return {'positions': [(1, 2, 3)]}


if __name__ == '__main__':
    app.run()
