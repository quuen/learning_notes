import sys
import json
import matplotlib.pyplot as plt

import numpy
from scipy.signal import butter, lfilter, freqz

def get_s_xy_kappa(fn):
    f = open(fn, 'r')
    xs = []
    ys = []
    ks = []
    theta = []
    s = []
    cnt = 0
    for line in f:
        cnt += 1
        if cnt < 3:
            continue
        line = line.replace("\n", '')
        data = json.loads(line)
        ks.append(data['kappa'])
        s.append(data['s'])
        xs.append(data['x'])
        ys.append(data['y'])
        theta.append(data['theta'])
    f.close()
    return s, xs, ys, theta, ks

s, xs, ys, theta, ks = get_s_xy_kappa('path1.txt.smoothed')
plt.plot(s)
plt.show()
