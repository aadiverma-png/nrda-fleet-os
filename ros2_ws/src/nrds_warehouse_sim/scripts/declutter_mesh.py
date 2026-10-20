#!/usr/bin/env python3
"""Build the demo warehouse mesh from the original: open roof, low racking, no overhead clutter.

    python3 scripts/declutter_mesh.py [--height 2.0]

Reads ``models/nrds_warehouse/meshes/nrds_warehouse.glb`` (the untouched original, with roof) and writes
``nrds_warehouse_no_roof.glb`` (rendered by Gazebo and RViz), so the result is reproducible:

* removed: roof (``roof_structure``, ``roof_panels_solar``), the 11 m building columns (with their guards; the
  collision boxes ``column_*`` go from ``model.sdf`` too), the wall shelving along the north and south
  walls (the space became lanes and chargers), the fire-sprinkler pipes and the high-bay lamps (they cut
  across every camera view), and the upper pallet levels (pallets and loads all sit above 1.97 m);
* racking and tote shelving are cut at ``--height``: triangles entirely above it are dropped and the
  remaining vertices are clamped to it, so uprights end cleanly (loads are not squashed);
* the Wi-Fi routers that sat on the rack tops are lowered onto the new tops (the one on the removed south
  wall shelving goes).

* the pack desks, the put wall, the outbound conveyors and their workers are moved beside the drive-through
  lanes (``MOVE``).

Visual only: robots, lidar and planning use the collision boxes in ``model.sdf`` (robot band 0.02-0.45 m).
The scene stays lit by the ``<light>`` elements of ``worlds/nrds_warehouse.sdf``.
"""

from __future__ import annotations

import argparse
import json
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(HERE, "models", "nrds_warehouse", "meshes")

REMOVE = ("roof_structure", "roof_panels_solar", "fire_sprinklers", "lighting",
          "wall_racking_north_upright", "wall_racking_north_beam", "wall_racking_north_deckMesh",
          "wall_shelving_south_shelf", "wall_shelving_south_totes_toteGrey", "wall_shelving_south_totes_tote",
          "pallets_pallet", "pallet_loads_carton", "pallet_loads_carton2", "pallet_loads_wrap",
          "columns_steel", "column_guards_yellow")
CUT = ("pallet_racking_upright", "pallet_racking_beam", "pallet_racking_deckMesh",
       "tote_shelving_shelf", "tote_shelving_totes_tote", "tote_shelving_totes_toteGrey")
# Drive-through service lanes (2026-09-30): the pack desks stand beside the packing lane VP, the outbound conveyors
# beside corridor VE2, the put wall clear of VE2's southern stretch. Node-name prefix -> (dx, dy) in world metres
# (the same shifts as the collision boxes in model.sdf; glTF x = world x, glTF z = -world y).
MOVE = {"pack_station_1_": (0.4, 4.6), "pack_station_2_": (0.4, 4.6), "pack_station_3_": (0.4, 4.6),
        "packer_1": (0.4, 4.6), "packer_2": (0.4, 4.6), "packer_3": (0.4, 4.6),
        "consolidation_put_wall_": (1.6, 0.0),
        "drop_conveyor_1_": (-1.7, -2.0), "dock_worker_1a": (-1.7, -2.0), "floor_text_drop_1": (0.0, -2.0),
        "drop_conveyor_2_": (-1.7, -1.4), "dock_worker_2a": (-1.7, -1.4), "floor_text_drop_2": (0.0, -1.4),
        "drop_conveyor_3_": (-1.7, 0.0), "dock_worker_3a": (-1.7, 0.0)}
DTYPE = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def read_glb(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, _ver, _total = struct.unpack("<4sII", data[:12])
    assert magic == b"glTF"
    jlen, jtype = struct.unpack("<I4s", data[12:20])
    assert jtype == b"JSON"
    gltf = json.loads(data[20:20 + jlen])
    off = 20 + jlen
    blen, btype = struct.unpack("<I4s", data[off:off + 8])
    assert btype == b"BIN\x00"
    return gltf, bytearray(data[off + 8:off + 8 + blen])


def write_glb(path, gltf, binary):
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    binary = bytes(binary) + b"\x00" * ((4 - len(binary) % 4) % 4)
    gltf["buffers"][0]["byteLength"] = len(binary)
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    out = struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(js) + 8 + len(binary))
    out += struct.pack("<I4s", len(js), b"JSON") + js + struct.pack("<I4s", len(binary), b"BIN\x00") + binary
    with open(path, "wb") as f:
        f.write(out)


def accessor_array(gltf, binary, idx):
    acc = gltf["accessors"][idx]
    bv = gltf["bufferViews"][acc["bufferView"]]
    dt = np.dtype(DTYPE[acc["componentType"]])
    n = NCOMP[acc["type"]]
    start = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = bv.get("byteStride") or dt.itemsize * n
    raw = np.frombuffer(bytes(binary[start:start + stride * acc["count"]]), dtype=np.uint8)
    rows = raw.reshape(acc["count"], stride)[:, :dt.itemsize * n].copy()
    return rows.view(dt).reshape(acc["count"], n) if n > 1 else rows.view(dt).reshape(acc["count"])


def append_accessor(gltf, binary, arr, comp_type, acc_type, target, with_bounds=False):
    while len(binary) % 4:
        binary.append(0)
    off = len(binary)
    raw = np.ascontiguousarray(arr).tobytes()
    binary.extend(raw)
    gltf["bufferViews"].append({"buffer": 0, "byteOffset": off, "byteLength": len(raw), "target": target})
    acc = {"bufferView": len(gltf["bufferViews"]) - 1, "componentType": comp_type,
           "count": int(arr.shape[0]), "type": acc_type}
    if with_bounds:
        acc["min"] = [float(v) for v in arr.min(axis=0)]
        acc["max"] = [float(v) for v in arr.max(axis=0)]
    gltf["accessors"].append(acc)
    return len(gltf["accessors"]) - 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--height", type=float, default=2.0, help="racking height in metres")
    ap.add_argument("--src", default=os.path.join(MESHES, "nrds_warehouse.glb"))
    ap.add_argument("--dst", default=os.path.join(MESHES, "nrds_warehouse_no_roof.glb"))
    a = ap.parse_args()
    H = a.height
    gltf, binary = read_glb(a.src)
    nodes = gltf["nodes"]
    byname = {}
    for i, n in enumerate(nodes):
        byname.setdefault(n.get("name", ""), i)

    # 1. unlink removed nodes (and the rack-top router over the removed south wall shelving)
    drop = {byname[k] for k in REMOVE if k in byname}
    for i, n in enumerate(nodes):
        if n.get("name", "").startswith("mesh_router_racktop") and "matrix" in n and abs(n["matrix"][14]) > 23.8:
            drop.add(i)
    for n in nodes:
        if "children" in n:
            n["children"] = [c for c in n["children"] if c not in drop]
    for sc in gltf["scenes"]:
        sc["nodes"] = [c for c in sc["nodes"] if c not in drop]
    print(f"removed {len(drop)} node groups: " + ", ".join(sorted(nodes[i].get("name", "?") for i in drop)))

    # 2. cut the racking at H
    for name in CUT:
        node = nodes[byname[name]]
        node.pop("scale", None)
        for prim in gltf["meshes"][node["mesh"]]["primitives"]:
            pos = accessor_array(gltf, binary, prim["attributes"]["POSITION"]).astype(np.float32)
            tri = accessor_array(gltf, binary, prim["indices"]).reshape(-1, 3)
            keep = ~(pos[tri][:, :, 1] > H + 1e-6).all(axis=1)
            new_tri = tri[keep].astype(np.uint32).reshape(-1)
            pos[:, 1] = np.minimum(pos[:, 1], H)
            prim["attributes"]["POSITION"] = append_accessor(gltf, binary, pos, 5126, "VEC3", 34962, True)
            prim["indices"] = append_accessor(gltf, binary, new_tri, 5125, "SCALAR", 34963)
            print(f"cut {name:30s} triangles {len(tri):7d} -> {int(keep.sum()):7d}   top {pos[:, 1].max():.2f} m")

    # 3. routers that sat on the rack tops rest on the new tops
    for n in nodes:
        if n.get("name", "").startswith("mesh_router_racktop") and "matrix" in n and n["matrix"][13] > H + 0.2:
            n["matrix"][13] = H + 0.15
    # 4. layout moves (whole named groups; children follow their parent)
    moved = 0
    for n in nodes:
        name = n.get("name", "")
        hit = next((d for pre, d in MOVE.items() if name == pre or (pre.endswith("_") and name.startswith(pre))), None)
        if hit is None:
            continue
        dx, dy = hit
        if "matrix" in n:
            n["matrix"][12] += dx
            n["matrix"][14] -= dy
        else:
            t = n.get("translation", [0.0, 0.0, 0.0])
            n["translation"] = [t[0] + dx, t[1], t[2] - dy]
        moved += 1
    print(f"moved {moved} nodes for the drive-through lanes")
    write_glb(a.dst, gltf, binary)
    print(f"wrote {a.dst} ({os.path.getsize(a.dst) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
