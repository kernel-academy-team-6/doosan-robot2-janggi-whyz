"""Photographic top surfaces, purely rendered assets (never recognition labels)."""
from pathlib import Path
import math
from xml.sax.saxutils import escape

from collision_masks import PIECE_CATEGORY, PIECE_COLLIDE, contact_xml

ASSETS = Path(__file__).resolve().parent/'assets/pieces'
# Gazebo opposite-flat width and thickness, not the octagon's circumdiameter.
WIDTHS_MM = {'king':34., 'sa':22., 'soldier':22.}
HEIGHTS_MM = {'king':8., 'sa':6., 'soldier':6.}
DEFAULT_WIDTH_MM, DEFAULT_HEIGHT_MM = 28., 7.
DIAMETERS_MM = {kind:width/math.cos(math.pi/8) for kind,width in WIDTHS_MM.items()}
DEFAULT_DIAMETER_MM = DEFAULT_WIDTH_MM/math.cos(math.pi/8)
# Top/bottom inset of the three non-supporting octagon boxes (see piece_physics).
SUPPORT_CLEARANCE_M = .0001


def piece_height_m(kind):
    return HEIGHTS_MM.get(kind,DEFAULT_HEIGHT_MM)/1000.


def body_geometry(kind,height):
    diameter=DIAMETERS_MM.get(kind,DEFAULT_DIAMETER_MM)/1000.
    uri=escape((ASSETS/'body.dae').as_uri())
    return (f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{diameter} {diameter} {height}</scale></mesh></geometry>')


def piece_visuals(side,kind,height=None,texture_key=None):
    height=piece_height_m(kind) if height is None else height
    diameter=DIAMETERS_MM.get(kind,DEFAULT_DIAMETER_MM)/1000.
    design=(ASSETS/'instances'/f'{texture_key}.png' if texture_key else ASSETS/f'{side}_{kind}.png')
    if not design.is_file():raise FileNotFoundError(f'Gazebo piece texture missing: {design}')
    mesh=escape((ASSETS/'top.dae').as_uri());texture=escape(design.as_uri())
    return (f'<visual name="body">{body_geometry(kind,height)}'
            '<material><ambient>0.82 0.78 0.67 1</ambient><diffuse>0.82 0.78 0.67 1</diffuse></material></visual>'
            f'<visual name="photo_top"><pose>0 0 {height/2+.0001} 0 0 0</pose>'
            f'<geometry><mesh><uri>{mesh}</uri><scale>{diameter} {diameter} 1</scale></mesh></geometry>'
            f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><emissive>0.5 0.5 0.5 1</emissive><pbr><metal>'
            f'<albedo_map>{texture}</albedo_map><emissive_map>{texture}</emissive_map><metalness>0</metalness><roughness>1</roughness>'
            f'</metal></pbr></material></visual>')


def piece_physics(kind, height=None, density=1200., friction=.8):
    """Assumed plastic density (kg/m³), regular octagonal prism inertia/contact."""
    height=piece_height_m(kind) if height is None else height
    if not all(math.isfinite(v) for v in (height,density,friction)) or height <= 0 or density <= 0 or friction < 0:
        raise ValueError('Invalid piece physical properties')
    radius = DIAMETERS_MM.get(kind, DEFAULT_DIAMETER_MM)/2000.
    mass = density*2*math.sqrt(2)*radius**2*height
    iz = mass*radius**2*(2+math.cos(math.pi/4))/6
    ixy = iz/2+mass*height**2/12
    # Exact regular octagonal prism = union of four rectangular prisms at
    # 0/45/90/135 degrees. Native boxes avoid voxelized thin-mesh contact and
    # unknown convex-hull margins. Mass/inertia describe the union, not its sum.
    # Only the 0-degree box spans the full thickness. The other three are
    # SUPPORT_CLEARANCE_M shorter at top and bottom: their octagon side faces
    # are unchanged for the pads, but four coplanar bottoms no longer touch the
    # board at once. With four supports Bullet never deactivated resting pieces;
    # 32 such pieces held the world near RTF 0.12 (sim-time camera ~4 FPS).
    w=WIDTHS_MM.get(kind,DEFAULT_WIDTH_MM)/1000.
    short=w*math.tan(math.pi/8)
    surface=(f'<surface><friction><torsional><coefficient>0</coefficient></torsional>'
             f'<ode><mu>{friction}</mu><mu2>{friction}</mu2></ode>'
             f'<bullet><friction>{friction}</friction><friction2>{friction}</friction2></bullet></friction>'
             '<bounce><restitution_coefficient>0</restitution_coefficient></bounce>'
             f'{contact_xml(PIECE_CATEGORY, PIECE_COLLIDE)}</surface>')
    contacts=''.join(f'<collision name="piece_contact{("_"+str(i)) if i else ""}">'
                     f'<pose>0 0 0 0 0 {i*math.pi/4}</pose>'
                     f'<geometry><box><size>{w} {short} {height-(2*SUPPORT_CLEARANCE_M if i else 0.)}</size></box></geometry>'
                     f'{surface}</collision>' for i in range(4))
    return (f'<inertial><mass>{mass}</mass><inertia><ixx>{ixy}</ixx><iyy>{ixy}</iyy>'
            f'<izz>{iz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<gravity>true</gravity>{contacts}')
