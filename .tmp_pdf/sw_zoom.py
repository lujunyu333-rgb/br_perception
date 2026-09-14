# -*- coding: utf-8 -*-
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D
data = json.load(open(r'C:\Users\希望\br_perception\.tmp_pdf\sketch_segments.json', encoding='utf-8'))
cols = plt.cm.tab20.colors
def draw(ax, xr, yr, title):
    ci = 0
    for s in data:
        if not s.get('segs'): continue
        c = cols[ci % 20]; ci += 1
        for g in s['segs']:
            if not g.get('p1') or not g.get('p2'): continue
            (x0,y0,_),(x1,y1,_) = g['p1'], g['p2']
            ax.add_line(Line2D([x0,x1],[y0,y1], color=c, lw=2.0))
    ax.set_xlim(*xr); ax.set_ylim(*yr); ax.set_title(title)
    ax.set_aspect('equal'); ax.grid(True, lw=0.3, alpha=0.5)
fig, axs = plt.subplots(1, 2, figsize=(16, 7))
draw(axs[0], (4.5, 6.5), (0.4, 2.1), 'SOUTH end 0-2.1 (grid 1m?)')
draw(axs[1], (4.5, 6.5), (8.6, 10.6), 'NORTH end 8.6-10.6 (center divider opening?)')
plt.tight_layout()
plt.savefig(r'C:\Users\希望\br_perception\.tmp_pdf\zoom_ends.png', dpi=170)
print('ok')
