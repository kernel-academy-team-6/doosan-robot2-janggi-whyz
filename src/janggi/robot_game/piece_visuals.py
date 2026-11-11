"""Photographic top surfaces, purely rendered assets (never recognition labels)."""
from pathlib import Path
from xml.sax.saxutils import escape

ASSETS = Path(__file__).resolve().parent/'assets/pieces'
DIAMETERS_MM = {'king':35.44792306470889, 'sa':22.728967273502516, 'soldier':22.728967273502516}
DEFAULT_DIAMETER_MM = 28.767328979182714


def piece_visuals(side,kind,height=.005,texture_key=None):
    diameter=DIAMETERS_MM.get(kind,DEFAULT_DIAMETER_MM)/1000.
    design=(ASSETS/'instances'/f'{texture_key}.png' if texture_key else ASSETS/f'{side}_{kind}.png')
    if not design.is_file():raise FileNotFoundError(f'Gazebo piece texture missing: {design}')
    mesh=escape((ASSETS/'top.dae').as_uri());texture=escape(design.as_uri())
    return (f'<visual name="body"><geometry><cylinder><radius>{diameter/2}</radius><length>{height}</length>'
            f'</cylinder></geometry><material><ambient>0.82 0.78 0.67 1</ambient><diffuse>0.82 0.78 0.67 1</diffuse></material></visual>'
            f'<visual name="photo_top"><pose>0 0 {height/2+.0001} 0 0 0</pose>'
            f'<geometry><mesh><uri>{mesh}</uri><scale>{diameter} {diameter} 1</scale></mesh></geometry>'
            f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><emissive>0.5 0.5 0.5 1</emissive><pbr><metal>'
            f'<albedo_map>{texture}</albedo_map><emissive_map>{texture}</emissive_map><metalness>0</metalness><roughness>1</roughness>'
            f'</metal></pbr></material></visual>')


def piece_physics(kind, height=.005, density=1200., friction=.8):
    """Assumed plastic density (kg/m³), cylinder inertia and contact geometry."""
    import math
    if height <= 0 or density <= 0 or friction < 0:
        raise ValueError('Invalid piece physical properties')
    radius = DIAMETERS_MM.get(kind, DEFAULT_DIAMETER_MM)/2000.
    mass = density*math.pi*radius**2*height
    ixy = mass*(3*radius**2+height**2)/12
    iz = mass*radius**2/2
    return (f'<inertial><mass>{mass}</mass><inertia><ixx>{ixy}</ixx><iyy>{ixy}</iyy>'
            f'<izz>{iz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<gravity>true</gravity><collision name="piece_contact"><geometry><cylinder>'
            f'<radius>{radius}</radius><length>{height}</length></cylinder></geometry>'
            f'<surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction>'
            f'<bounce><restitution_coefficient>0</restitution_coefficient></bounce></surface></collision>')
