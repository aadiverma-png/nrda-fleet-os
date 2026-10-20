#!/usr/bin/env python3
"""Regenerate models/nrds_lidar_proxy from models/nrds_warehouse/model.sdf.

The AMR lidars (visibility_mask 2) see only this proxy: every floor-level collision box of the
warehouse as a plain box visual (visibility_flags 2), clipped to 0.02-0.60 m and inset by 1 cm so it
stays hidden inside the detailed mesh in the Gazebo GUI. The 24 MB mesh carries visibility_flags 1,
so it is drawn for cameras but never rendered by the 40 lidars. Run after editing the warehouse model.
"""
import os
import re
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if len(sys.argv) < 2 else sys.argv[1]
src = open(os.path.join(root, "models", "nrds_warehouse", "model.sdf")).read()
pat = re.compile(r'<collision name="([^"]+)"><pose>([^<]+)</pose><geometry><box><size>([^<]+)</size></box></geometry></collision>')
vis = []
for n, p, sz in pat.findall(src):
    x, y, z, r, pi, yw = map(float, p.split())
    sx, sy, sz_ = map(float, sz.split())
    lo, hi = z - sz_ / 2, z + sz_ / 2
    if hi <= 0.02 or lo >= 0.6:
        continue
    lo, hi = max(lo, 0.02), min(hi, 0.6)
    ix, iy = max(sx - 0.02, 0.01), max(sy - 0.02, 0.01)
    vis.append(f'      <visual name="{n}"><pose>{x} {y} {(lo + hi) / 2:.3f} {r} {pi} {yw}</pose><geometry><box><size>'
               f'{ix:.3f} {iy:.3f} {hi - lo:.3f}</size></box></geometry><visibility_flags>2</visibility_flags>'
               '<material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>')
out = os.path.join(root, "models", "nrds_lidar_proxy", "model.sdf")
with open(out, "w") as f:
    f.write('<?xml version="1.0"?>\n<!-- Generated from models/nrds_warehouse/model.sdf by scripts/make_lidar_proxy.py. '
            'Lidar-only geometry (visibility flag 2). -->\n<sdf version="1.9">\n  <model name="nrds_lidar_proxy">\n'
            '    <static>true</static>\n    <link name="proxy">\n' + "\n".join(vis) + '\n    </link>\n  </model>\n</sdf>\n')
print(f"{len(vis)} proxy visuals -> {out}")
