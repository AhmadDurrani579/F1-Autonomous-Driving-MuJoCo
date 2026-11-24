
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path.home() / "projects/F1-MuJoCo"
SOURCE = ROOT / "models/formula1.xml"
OUTPUT = ROOT / "models/formula1_track.xml"
WAYPOINTS = ROOT / "tracks/oval_waypoints.csv"

TRACK_WIDTH = 12.0
RADIUS = 22.0
STRAIGHT_HALF = 35.0
STEP = 2.0

# Centreline: front straight, right turn,
# return straight, left turn.
points = []

def add_point(x, y):
    p = (float(x), float(y))
    if not points or math.dist(points[-1], p) > 1e-6:
        points.append(p)

# Front straight: left to right
for i in range(36):
    add_point(-STRAIGHT_HALF + i * 2, -RADIUS)

# Right-hand semicircle
n_arc = math.ceil(math.pi * RADIUS / STEP)
for i in range(1, n_arc + 1):
    angle = -math.pi / 2 + math.pi * i / n_arc
    add_point(
        STRAIGHT_HALF + RADIUS * math.cos(angle),
        RADIUS * math.sin(angle)
    )

# Return straight: right to left
for i in range(1, 36):
    add_point(STRAIGHT_HALF - i * 2, RADIUS)

# Left-hand semicircle
for i in range(1, n_arc):
    angle = math.pi / 2 + math.pi * i / n_arc
    add_point(
        -STRAIGHT_HALF + RADIUS * math.cos(angle),
        RADIUS * math.sin(angle)
    )

if not SOURCE.exists():
    raise FileNotFoundError(SOURCE)

parser = ET.XMLParser(
    target=ET.TreeBuilder(insert_comments=True)
)
tree = ET.parse(SOURCE, parser=parser)
root = tree.getroot()
world = root.find("worldbody")

if world is None:
    raise RuntimeError("Missing worldbody in Formula 1 XML")

# Keep original vehicle, geoms, joints and actuators.
# Add only static visual track geometry.

for i in range(len(points)):
    x1, y1 = points[i]
    x2, y2 = points[(i + 1) % len(points)]

    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2

    # Asphalt strip: visual only.
    ET.SubElement(world, "geom", {
        "name": f"road_{i:03d}",
        "type": "box",
        "pos": f"{mx:.6f} {my:.6f} 0.0035",
        "size": f"{length/2 + 0.15:.6f} 6 0.003",
        "euler": f"0 0 {yaw:.8f}",
        "rgba": "0.16 0.17 0.19 1",
        "contype": "0",
        "conaffinity": "0",
        "group": "1"
    })

    # Track edges with alternating red/white curbs.
    nx, ny = -dy / length, dx / length
    color = (
        "0.85 0.08 0.07 1" if i % 2 == 0
        else "0.95 0.95 0.95 1"
    )

    for side, label in [(1, "outer"), (-1, "inner")]:
        offset = side * (TRACK_WIDTH / 2 + 0.25)

        ET.SubElement(world, "geom", {
            "name": f"curb_{label}_{i:03d}",
            "type": "box",
            "pos": (
                f"{mx + nx * offset:.6f} "
                f"{my + ny * offset:.6f} 0.005"
            ),
            "size": f"{length/2 + 0.15:.6f} 0.25 0.004",
            "euler": f"0 0 {yaw:.8f}",
            "rgba": color,
            "contype": "0",
            "conaffinity": "0",
            "group": "1"
        })

# Starting line across the lower straight
ET.SubElement(world, "geom", {
    "name": "start_finish_line",
    "type": "box",
    "pos": "-30 -22 0.008",
    "size": "0.12 5.8 0.001",
    "rgba": "1 1 1 1",
    "contype": "0",
    "conaffinity": "0"
})

# Position the original Formula 1 chassis on the track.
chassis = world.find("./body[@name='chassis']")
if chassis is None:
    raise RuntimeError("Chassis body not found")

# Front of the vehicle already points along +X.
chassis.set("pos", "-34 -22 0.01")

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
WAYPOINTS.parent.mkdir(parents=True, exist_ok=True)

ET.indent(tree, space="  ")
tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)

# Save centreline waypoints for later C++ algorithms.
with open(WAYPOINTS, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "heading"])

    for i, (x, y) in enumerate(points):
        nx, ny = points[(i + 1) % len(points)]
        heading = math.atan2(ny - y, nx - x)
        writer.writerow([x, y, heading])

print("TRACK GENERATION COMPLETE")
print("Track:", OUTPUT)
print("Waypoints:", WAYPOINTS)
print("Segments:", len(points))
print("Track width:", TRACK_WIDTH, "m")
print("Car start:", "(-34, -22, 0.01)")
