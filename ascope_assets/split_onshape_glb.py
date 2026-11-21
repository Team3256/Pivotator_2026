"""
Split one full-robot Onshape .glb export into the files AdvantageScope needs.

Why: AdvantageScope wants one .glb for the fixed robot plus one per moving part. Doing that in
Onshape means five exports with lots of suppressing. Instead, export the whole assembly ONCE
(everything at its zero position) and run:

    python3 ascope_assets/split_onshape_glb.py "~/Downloads/00. Main Unconstrained.glb"

It writes model.glb, model_0.glb ... model_3.glb into ascope_assets/Robot_Pivotvator2026/.
Every part keeps its exact CAD position, so config.json can use the same zero offsets for all.

Which part goes where is decided by the rules in classify() below, using the part names in the
"00. Main Unconstrained" assembly. If the CAD gets new parts that land in the wrong file, add a
name rule there. Small hardware (bolts, nuts, spacers...) is dropped to keep the files small.
Only uses the Python standard library.
"""

import json
import os
import re
import struct
import sys

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Robot_Pivotvator2026")

# Output file for each bucket. Order of model_N must match "components" in config.json and the
# Pose3d[] order in RobotVisualizer.
FILES = {
    "base": "model.glb",  # never moves: drivebase, swerve, pivot gearbox on the frame
    "fixed_stage": "model_0.glb",  # tilted by the pivot, doesn't slide
    "stage1": "model_1.glb",  # slides h/2
    "carriage": "model_2.glb",  # slides h
    "wrist": "model_3.glb",  # rotates on the carriage
}

# Top-level sub-assemblies in "00. Main Unconstrained".
ELEVATOR_ASM = "Assembly 1 <1>"  # 02. Elevator (with the pivot interface plates)
WRIST_ASM = "03. Wrist Assembly <1>"

# Dropped everywhere: fasteners/spacers (tiny but there are hundreds), cable chain (it bends, so it
# looks wrong in any single file), and Onshape reference cubes.
DROP = re.compile(
    r"Bolt|Screw|SHCS|BHCS|FHCS|Locknut|Nut\b|Washer|Spacer|Rivet|Standoff|Zip Tie|"
    r"Igus|Origin Cube|Reference",
    re.I,
)


def elevator_bucket(name, center_x):
    """Sort one part of the elevator sub-assembly into fixed_stage / stage1 / carriage."""
    if re.search(r"Belt HTD5", name, re.I):
        return "fixed_stage"  # the belt loop stays with the fixed stage (name says "carriage")
    if re.search(r"Carriage", name, re.I):
        return "carriage"
    if re.search(r"Stage 1", name, re.I):
        return "stage1"
    if re.search(r"Hardstop|Belt-in-Tube Block|Corner Bearing Block", name, re.I):
        # These ride on the rails. The three tubes are nested side by side in X when stowed:
        # fixed rails outermost (|x| ~0.07-0.09 m), 1st stage (~0.03-0.06), carriage in the middle.
        ax = abs(center_x)
        if ax > 0.065:
            return "fixed_stage"
        if ax > 0.03:
            return "stage1"
        return "carriage"
    # Everything else (fixed rails, crossbeam, gearbox, motors, belt, pivot plates) tilts with the
    # pivot but doesn't slide.
    return "fixed_stage"


def classify(top_name, part_name, center_x):
    if top_name == WRIST_ASM:
        return "wrist"
    if top_name == ELEVATOR_ASM:
        return elevator_bucket(part_name, center_x)
    return "base"


# ---------------- glTF plumbing ----------------


def read_glb(path):
    data = open(path, "rb").read()
    magic, _, _ = struct.unpack("<4sII", data[:12])
    assert magic == b"glTF", "not a .glb file"
    jlen, _ = struct.unpack("<II", data[12:20])
    gltf = json.loads(data[20 : 20 + jlen])
    off = 20 + jlen
    blen, _ = struct.unpack("<II", data[off : off + 8])
    return gltf, data[off + 8 : off + 8 + blen]


def node_matrix(n):
    if "matrix" in n:
        m = n["matrix"]
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    t = n.get("translation", [0, 0, 0])
    x, y, z, w = n.get("rotation", [0, 0, 0, 1])
    s = n.get("scale", [1, 1, 1])
    r = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return [[r[i][k] * s[k] for k in range(3)] + [t[i]] for i in range(3)] + [[0, 0, 0, 1]]


def matmul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


IDENTITY = [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)]


def subtree_bbox(g, i, parent, box):
    n = g["nodes"][i]
    m = matmul(parent, node_matrix(n))
    if "mesh" in n:
        for p in g["meshes"][n["mesh"]]["primitives"]:
            a = g["accessors"][p["attributes"]["POSITION"]]
            for cx in (a["min"][0], a["max"][0]):
                for cy in (a["min"][1], a["max"][1]):
                    for cz in (a["min"][2], a["max"][2]):
                        v = [sum(m[r][k] * (cx, cy, cz, 1)[k] for k in range(4)) for r in range(3)]
                        for k in range(3):
                            box[0][k] = min(box[0][k], v[k])
                            box[1][k] = max(box[1][k], v[k])
    for c in n.get("children", []):
        subtree_bbox(g, c, m, box)
    return box


def write_subset(g, binary, keep_children, out_path):
    """Write a .glb with the same node tree, where each node only keeps children in keep_children
    (a dict node -> list of kept children). Unused meshes/accessors/bytes are dropped."""
    nodes_out, remap = [], {}

    def copy(i):
        n = dict(g["nodes"][i])
        n.pop("extensions", None)
        new_i = len(nodes_out)
        remap[i] = new_i
        nodes_out.append(n)
        kids = keep_children.get(i, n.get("children", []))
        n["children"] = [copy(c) for c in kids]
        if not n["children"]:
            del n["children"]
        return new_i

    roots = [copy(r) for r in g["scenes"][g.get("scene", 0)]["nodes"]]

    mesh_map, meshes = {}, []
    acc_map, accessors = {}, []
    bv_map, views = {}, []
    out_bin = bytearray()

    def use_accessor(ai):
        if ai in acc_map:
            return acc_map[ai]
        a = dict(g["accessors"][ai])
        bvi = a["bufferView"]
        if bvi not in bv_map:
            bv = dict(g["bufferViews"][bvi])
            start = bv.get("byteOffset", 0)
            chunk = binary[start : start + bv["byteLength"]]
            while len(out_bin) % 4:
                out_bin.append(0)
            bv["byteOffset"] = len(out_bin)
            out_bin.extend(chunk)
            bv_map[bvi] = len(views)
            views.append(bv)
        a["bufferView"] = bv_map[bvi]
        acc_map[ai] = len(accessors)
        accessors.append(a)
        return acc_map[ai]

    for n in nodes_out:
        if "mesh" not in n:
            continue
        mi = n["mesh"]
        if mi not in mesh_map:
            mesh = dict(g["meshes"][mi])
            prims = []
            for p in mesh["primitives"]:
                p = dict(p)
                p.pop("extensions", None)
                p["attributes"] = {k: use_accessor(v) for k, v in p["attributes"].items()}
                if "indices" in p:
                    p["indices"] = use_accessor(p["indices"])
                prims.append(p)
            mesh["primitives"] = prims
            mesh.pop("extensions", None)
            mesh_map[mi] = len(meshes)
            meshes.append(mesh)
        n["mesh"] = mesh_map[mi]

    while len(out_bin) % 4:
        out_bin.append(0)
    out = {
        "asset": {"version": "2.0", "generator": "3256 split_onshape_glb.py"},
        "scene": 0,
        "scenes": [{"nodes": roots}],
        "nodes": nodes_out,
        "meshes": meshes,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(out_bin)}],
        "materials": [
            {k: v for k, v in m.items() if k != "extensions"} for m in g.get("materials", [])
        ],
    }
    js = json.dumps(out, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(out_bin)
    with open(out_path, "wb") as fh:
        fh.write(struct.pack("<4sII", b"glTF", 2, total))
        fh.write(struct.pack("<II", len(js), 0x4E4F534A))
        fh.write(js)
        fh.write(struct.pack("<II", len(out_bin), 0x004E4942))
        fh.write(out_bin)


def main():
    src = os.path.expanduser(sys.argv[1])
    g, binary = read_glb(src)
    root = g["scenes"][g.get("scene", 0)]["nodes"][0]
    root_m = node_matrix(g["nodes"][root])

    buckets = {b: {} for b in FILES}  # bucket -> keep_children
    report = {b: [] for b in FILES}

    def keep(bucket, parent, child):
        buckets[bucket].setdefault(parent, []).append(child)

    for top in g["nodes"][root].get("children", []):
        top_name = g["nodes"][top].get("name", "")
        if DROP.search(top_name):
            continue
        if top_name in (ELEVATOR_ASM, WRIST_ASM):
            top_m = matmul(root_m, node_matrix(g["nodes"][top]))
            for part in g["nodes"][top].get("children", []):
                pname = g["nodes"][part].get("name", "")
                if DROP.search(pname):
                    continue
                box = subtree_bbox(g, part, top_m, [[1e9] * 3, [-1e9] * 3])
                if box[0][0] > 1e8:
                    continue  # no geometry
                b = classify(top_name, pname, (box[0][0] + box[1][0]) / 2)
                keep(b, top, part)
                report[b].append(pname)
            for b in FILES:
                if top in buckets[b]:
                    keep(b, root, top)
        else:
            keep("base", root, top)
            report["base"].append(top_name)

    # Also drop hardware nested deeper inside kept parts.
    def prune(i, keep_children):
        n = g["nodes"][i]
        kids = keep_children.get(i, n.get("children", []))
        kids = [c for c in kids if not DROP.search(g["nodes"][c].get("name", ""))]
        keep_children[i] = kids
        for c in kids:
            prune(c, keep_children)

    os.makedirs(OUT_DIR, exist_ok=True)
    for b, fname in FILES.items():
        kc = buckets[b]
        kc.setdefault(root, [])
        prune(root, kc)
        path = os.path.join(OUT_DIR, fname)
        write_subset(g, binary, kc, path)
        sub, _ = read_glb(path)
        box = [[1e9] * 3, [-1e9] * 3]
        for r in sub["scenes"][0]["nodes"]:
            subtree_bbox(sub, r, IDENTITY, box)
        names = sorted(set(re.sub(r"^occurrence of ", "", x) for x in report[b]))
        print(f"{fname:12s} {b:12s} {os.path.getsize(path) / 1e6:6.1f} MB  "
              f"min {[round(v, 2) for v in box[0]]} max {[round(v, 2) for v in box[1]]}")
        if b != "base":
            print("    " + "; ".join(names))


if __name__ == "__main__":
    main()
