#!/usr/bin/env python3
"""Generate a custom full-size MuJoCo circuit, preserving existing Formula 1 MJCF.
Run with /usr/bin/python3 tools/build_custom_f1_circuit.py.
Requires only the Python standard library.
"""
import copy
import csv
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'models' / 'formula1.xml'
OUTPUT = ROOT / 'models' / 'formula1_custom.xml'
OUT_DIR = ROOT / 'assets' / 'f1_car' / 'custom_track'
WAYPOINTS = ROOT / 'tracks' / 'custom_waypoints.csv'
TRACK_WIDTH = 13.0
SAMPLES_PER_SECTION = 24
RNG = random.Random(101)

# Clockwise-ish sequence, with a hairpin on the right and a flowing S-section.

# Custom full-size Formula 1 circuit
# Revised to remove two excessively tight control points.

CONTROL = [
    (-95, -35),
    (-53, -62),
    (-7, -59),
    (25, -41),
    (56, -53),
    (91, -70),
    (116, -47),
    (114, -18),
    (86, -5),
    (77, 29),
    (66, 58),
    (34, 71),
    (-24, 69),
    (-64, 63),
    (-102, 40),
    (-117, 7)
]

def catmull(p0, p1, p2, p3, t):
    t2, t3 = t*t, t*t*t
    return (0.5*((2*p1[0])+(-p0[0]+p2[0])*t+(2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2+(-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3),
            0.5*((2*p1[1])+(-p0[1]+p2[1])*t+(2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2+(-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3))

points = []
for i in range(len(CONTROL)):
    for j in range(SAMPLES_PER_SECTION):
        n = len(CONTROL)
        p = catmull(CONTROL[(i-1)%n], CONTROL[i], CONTROL[(i+1)%n], CONTROL[(i+2)%n], j/SAMPLES_PER_SECTION)
        if not points or math.dist(p, points[-1]) > 0.04:
            points.append(p)

N = len(points)
pathlength = sum(math.dist(points[i], points[(i+1)%N]) for i in range(N))
print('Path samples:', N, 'lap length approx:', round(pathlength, 1), 'm')

# Tangent / local normal at each sample.
tangents = []
normals = []
for i, p in enumerate(points):
    prior, nxt = points[i-1], points[(i+1)%N]
    dx, dy = nxt[0]-prior[0], nxt[1]-prior[1]
    length = math.hypot(dx, dy)
    tx, ty = dx/length, dy/length
    tangents.append((tx, ty))
    normals.append((-ty, tx))

OUT_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
WAYPOINTS.parent.mkdir(parents=True, exist_ok=True)

# Closed manifold strip with top, bottom and side faces (better behaved than open planar mesh).
def ribbon_mesh(filename, inner, outer, z_top, z_bottom, uv=False):
    """Build ribbon between two offset curves; each offset may be variable."""
    verts = []
    for i, (x, y) in enumerate(points):
        nx, ny = normals[i]
        for offset, z in ((inner, z_top), (outer, z_top), (inner, z_bottom), (outer, z_bottom)):
            verts.append((x+nx*offset, y+ny*offset, z))
    faces = []
    for i in range(N):
        a, b = 4*i, 4*((i+1)%N)
        # top, bottom, and side walls
        quads = [(a,a+1,b+1,b), (a+2,b+2,b+3,a+3),
                 (a,b,b+2,a+2), (a+1,a+3,b+3,b+1)]
        for p,q,r,s in quads:
            faces.extend([(p+1,q+1,r+1), (p+1,r+1,s+1)])
    path = OUT_DIR / filename
    with path.open('w') as f:
        f.write('# Generated track mesh; units metres, Z up, no texture coordinates\n')
        for x,y,z in verts:
            f.write(f'v {x:.6f} {y:.6f} {z:.6f}\n')
        for a,b,c in faces:
            f.write(f'f {a} {b} {c}\n')
    return path

# One road mesh and two visually distinct kerb strips outside each side.
ribbon_mesh('road.obj', -TRACK_WIDTH/2, TRACK_WIDTH/2, 0.015, 0.003)
ribbon_mesh('kerb_left.obj', TRACK_WIDTH/2, TRACK_WIDTH/2+0.65, 0.025, 0.005)
ribbon_mesh('kerb_right.obj', -TRACK_WIDTH/2-0.65, -TRACK_WIDTH/2, 0.025, 0.005)

# Source is the complete coloured vehicle (not the old already-generated oval).
if not SOURCE.exists():
    raise FileNotFoundError(f'Missing vehicle XML: {SOURCE}')
parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
tree = ET.parse(SOURCE, parser=parser)
root = tree.getroot()
root.set('model', 'F1_Custom_Circuit')
world = root.find('worldbody')
assets = root.find('asset')
if world is None or assets is None:
    raise RuntimeError('Missing asset or worldbody section')

# Grass existing ground: remains the sole collision plane.
for geom in world.findall('geom'):
    if geom.get('name') == 'ground':
        geom.set('rgba', '0.20 0.42 0.19 1')
        geom.attrib.pop('material', None)
        geom.set('size', '250 250 0.1')
        break
else:
    raise RuntimeError('Source MJCF lacks ground geom')

# Material names explicit and unique.
for name, color in [
    ('custom_asphalt','0.19 0.21 0.23 1'),
    ('custom_kerb_red','0.80 0.09 0.08 1'),
    ('custom_kerb_white','0.96 0.96 0.93 1'),
    ('custom_gravel','0.58 0.48 0.33 1'),
    ('custom_barrier','0.77 0.81 0.84 1'),
    ('custom_concrete','0.63 0.67 0.70 1'),
    ('custom_roof','0.13 0.19 0.25 1'),
    ('custom_window','0.27 0.48 0.62 1'),
    ('custom_tree','0.09 0.29 0.12 1'),
    ('custom_trunk','0.31 0.21 0.14 1'),
    ('custom_yellow','0.95 0.75 0.15 1')]:
    ET.SubElement(assets, 'material', name=name, rgba=color)
for name in ('road', 'kerb_left', 'kerb_right'):
    ET.SubElement(assets, 'mesh', name='custom_'+name, file='custom_track/'+name+'.obj')

# Use completely non-colliding visual road; existing flat ground handles wheel collision.
for name, material in [('road','custom_asphalt'),('kerb_left','custom_kerb_red'),('kerb_right','custom_kerb_white')]:
    ET.SubElement(world, 'geom', name='custom_'+name, type='mesh', mesh='custom_'+name,
                  material=material, contype='0', conaffinity='0', group='1')

# Add alternating kerb block markings. Thin placed boxes with no collision.
for side, s_name in ((1,'L'),(-1,'R')):
    for i in range(0,N,10):
        x,y=points[i]
        nx,ny=normals[i]
        tx,ty=tangents[i]
        yaw=math.atan2(ty,tx)
        o=side*(TRACK_WIDTH/2+0.325)
        ET.SubElement(world,'geom',name=f'custom_kerb_tile_{s_name}_{i}',type='box',
            pos=f'{x+o*nx:.5f} {y+o*ny:.5f} 0.029', size='1.9 0.32 0.002',
            euler=f'0 0 {yaw:.7f}', material='custom_kerb_white' if (i//10)%2 else 'custom_kerb_red',
            contype='0',conaffinity='0',group='1')

# Low barriers near road edge: rendered + detectable by LiDAR/raycast; no immediate vehicle hazard.

# ==============================================
# TRACKSIDE GUARDRAIL CONFIGURATION
# ==============================================


# ==========================================
# KERB TILE CONFIGURATION
# ==========================================

# Disable individual rectangular kerb tiles
# until curved kerb geometry is implemented.
ENABLE_KERB_TILES = False

if ENABLE_KERB_TILES:
    for side, s_name in ((1, 'L'), (-1, 'R')):
        for i in range(0, N, 10):
            x, y = points[i]
            nx, ny = normals[i]
            tx, ty = tangents[i]
            yaw = math.atan2(ty, tx)

            offset = side * (TRACK_WIDTH / 2 + 0.325)

            ET.SubElement(
                world,
                'geom',
                name=f'custom_kerb_tile_{s_name}_{i}',
                type='box',
                pos=(
                    f'{x + offset * nx:.5f} '
                    f'{y + offset * ny:.5f} 0.029'
                ),
                size='1.0 0.32 0.002',
                euler=f'0 0 {yaw:.7f}',
                material=(
                    'custom_kerb_white'
                    if (i // 10) % 2
                    else 'custom_kerb_red'
                ),
                contype='0',
                conaffinity='0',
                group='1'
            )

# Place the car exactly on the centreline, facing the same direction as waypoints.
chassis=world.find("./body[@name='chassis']")
if chassis is None:
    raise RuntimeError('chassis body missing')
start_i=24  # Starting zone on opening section
sx,sy=points[start_i]
tx,ty=tangents[start_i]
yaw=math.atan2(ty,tx)
chassis.set('pos', f'{sx:.6f} {sy:.6f} 0.01')
chassis.set('quat',f'{math.cos(yaw/2):.8f} 0 0 {math.sin(yaw/2):.8f}')

# Start line across track at start pose (visual only).
nx,ny=normals[start_i]
ET.SubElement(world,'geom',name='custom_start_line',type='box',
    pos=f'{sx:.6f} {sy:.6f} 0.035',size=f'0.16 {TRACK_WIDTH/2-0.5:.4f} 0.003',
    euler=f'0 0 {yaw:.6f}',rgba='0.98 0.98 0.98 1',contype='0',conaffinity='0')

# Pit complex outside starting lane; all display-only for first tests.
pit_x,pit_y=sx-13*nx,sy-13*ny
for name,offset_x,offset_y,height,half_dims,mat in [
    ('pit_garage',-13, -8, 3.0,(19,6,3),'custom_concrete'),
    ('pit_upper',-13, -8, 6.6,(19,6,0.55),'custom_roof'),
    ('pit_windows',-13, -1.9, 4.6,(18.4,0.12,1.15),'custom_window'),
    ('control_tower',12,-9,5,(3.5,3.5,5),'custom_concrete'),
    ('tower_top',12,-9,10.6,(4.4,4.4,0.7),'custom_roof'),
    ('grandstand',38,-13,2.3,(18,7,2.3),'custom_concrete'),
    ('grandstand_roof',38,-13,7.3,(20,8,0.4),'custom_roof')]:
    # Local u=road tangent, v=road normal; keep objects beyond road margin.
    cx=sx+offset_x*tx+offset_y*nx
    cy=sy+offset_x*ty+offset_y*ny
    ET.SubElement(world,'geom',name='custom_'+name,type='box',
        pos=f'{cx:.5f} {cy:.5f} {height:.4f}',
        size=' '.join(str(x) for x in half_dims),
        euler=f'0 0 {yaw:.6f}',material=mat,contype='0',conaffinity='0',group='3')

# Start/finish gantry runs across road, beyond track edges; vertical clearance 6 m.
for side in (-1,1):
    cx=sx+side*nx*(TRACK_WIDTH/2+1.6)
    cy=sy+side*ny*(TRACK_WIDTH/2+1.6)
    ET.SubElement(world,'geom',name=f'custom_start_post_{side}',type='box',
        pos=f'{cx:.5f} {cy:.5f} 3',size='0.20 0.20 3',
        euler=f'0 0 {yaw:.6f}',material='custom_roof',
        contype='0',conaffinity='0')
ET.SubElement(world,'geom',name='custom_start_gantry',type='box',
    pos=f'{sx:.5f} {sy:.5f} 6.0',size=f'0.25 {TRACK_WIDTH/2+1.9:.4f} 0.33',
    euler=f'0 0 {yaw:.6f}',material='custom_roof',contype='0',conaffinity='0')

# Trees at safe distance from any road sample and outside pit paddock.
def min_distance_to_centerline(x,y):
    return min(math.hypot(x-px,y-py) for px,py in points[::3])

num_trees=0
for k in range(250):
    x,y=RNG.uniform(-155,155),RNG.uniform(-108,108)
    if min_distance_to_centerline(x,y)<21 or math.hypot(x-pit_x,y-pit_y)<40:
        continue
    h=RNG.uniform(3.3,5.8)
    ET.SubElement(world,'geom',name=f'custom_tree_trunk_{k}',type='cylinder',
        pos=f'{x:.4f} {y:.4f} {h/2:.4f}',size=f'0.22 {h/2:.4f}',
        material='custom_trunk',contype='0',conaffinity='0',group='3')
    ET.SubElement(world,'geom',name=f'custom_tree_crown_{k}',type='sphere',
        pos=f'{x:.4f} {y:.4f} {h+1.55:.4f}',size='1.9',
        material='custom_tree',contype='0',conaffinity='0',group='3')
    num_trees+=1
    if num_trees>=75:break

ET.SubElement(world,'camera',name='custom_aerial',pos='0 -170 125',
              xyaxes='1 0 0 0 0.55 0.835')
# Fixed onboard cam parented to car.
ET.SubElement(chassis,'camera',name='custom_onboard',pos='0.55 0 1.35',
              xyaxes='0 -1 0 0 0 1',fovy='85')

with WAYPOINTS.open('w',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(('x','y','heading','s'))
    arc_length=0.0
    for i,(x,y) in enumerate(points):
        if i:arc_length+=math.dist(points[i-1],points[i])
        tx,ty=tangents[i]
        writer.writerow((f'{x:.5f}',f'{y:.5f}',f'{math.atan2(ty,tx):.7f}',f'{arc_length:.4f}'))

ET.indent(tree,space='  ')
tree.write(OUTPUT,encoding='utf-8',xml_declaration=True)
print('SUCCESS')
print('Generated model:',OUTPUT)
print('Waypoints:',WAYPOINTS)
print('Track width:',TRACK_WIDTH,'m')
print('Approx lap length:',round(pathlength,1),'m')
print('Trees:',num_trees)
print('Car start:',round(sx,2),round(sy,2),'yaw:',round(yaw,3))
print('Open formula1_custom.xml in MuJoCo Studio, not formula1_track.xml.')
