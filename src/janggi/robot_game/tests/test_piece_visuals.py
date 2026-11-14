from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from piece_visuals import ASSETS,piece_visuals,piece_physics


def test_all_piece_materials_use_supplied_photo_with_recorded_sources():
    root=Path(__file__).resolve().parents[4]
    provenance=json.loads((ASSETS/'provenance.json').read_text())
    assert len(provenance)==14
    for label,entry in provenance.items():
        source=root/entry['source']
        assert source.suffix=='.jpg' and source.is_file()
        assert hashlib.sha256(source.read_bytes()).hexdigest()==entry['sha256']
        assert (ASSETS/(label+'.png')).is_file()
        side,kind=label.split('_',1)
        tree=ET.fromstring('<root>'+piece_visuals(side,kind)+'</root>')
        visual=tree.find("visual[@name='photo_top']")
        assert visual is not None
        assert visual.findtext('material/pbr/metal/albedo_map')==(ASSETS/(label+'.png')).as_uri()
        body=tree.find("visual[@name='body']/geometry/mesh")
        # Collision hull approximation must not alter the visible top and hide
        # the photograph on medium/large pieces (Ogre2 rendering regression).
        assert body.get('optimization') is None
        assert body.find('convex_decomposition') is None
        contact=ET.fromstring('<root>'+piece_physics(kind)+'</root>')
        assert len(contact.findall('collision'))==4
        assert all(c.find('geometry/box') is not None for c in contact.findall('collision'))


def test_each_rendered_body_uses_its_own_photographed_piece():
    import sim_app
    from gazebo_scene import RenderedWorld
    from simulation_calibration import ensure_simulation_calibration
    world=RenderedWorld(ensure_simulation_calibration())
    keys=[piece['texture_key'] for piece in world.pieces]
    assert len(keys)==len(set(keys))==32
    manifest=json.loads((ASSETS/'photo_instances.json').read_text())
    assert set(keys)==set(manifest)
    for piece in world.pieces:
        key=piece['texture_key']
        assert manifest[key]['label']==f"{piece['side']}_{piece['kind']}"
        tree=ET.fromstring('<root>'+piece_visuals(piece['side'],piece['kind'],texture_key=key)+'</root>')
        visual=tree.find("visual[@name='photo_top']")
        assert visual.findtext('material/pbr/metal/albedo_map')==(ASSETS/'instances'/(key+'.png')).as_uri()


def test_four_boxes_form_exact_octagon_without_collision_margins():
    import numpy as np
    import math
    width=.028
    root=ET.fromstring('<root>'+piece_physics('cha')+'</root>')
    # Compare union membership against all eight analytical octagon half-planes.
    xy=np.array([(x,y) for x in np.linspace(-.016,.016,81) for y in np.linspace(-.016,.016,81)])
    in_union=np.zeros(len(xy),bool)
    for c in root.findall('collision'):
        angle=float(c.findtext('pose').split()[-1]);size=np.array(list(map(float,c.findtext('geometry/box/size').split())))
        rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        local=xy@rotation
        in_union|=np.all(np.abs(local)<=size[:2]/2+1e-12,axis=1)
    normals=np.array([[math.cos(i*math.pi/4),math.sin(i*math.pi/4)] for i in range(8)])
    in_octagon=np.all(xy@normals.T<=width/2+1e-12,axis=1)
    assert np.array_equal(in_union,in_octagon)


def test_only_one_box_supports_the_piece_so_resting_pieces_can_sleep():
    from piece_visuals import SUPPORT_CLEARANCE_M
    for kind,thickness in (('soldier',.006),('cha',.007),('king',.008)):
        root=ET.fromstring('<root>'+piece_physics(kind)+'</root>')
        heights=[float(c.findtext('geometry/box/size').split()[2]) for c in root.findall('collision')]
        # Exactly one full-thickness box touches the board; side faces stay exact.
        assert heights[0]==pytest.approx(thickness)
        assert heights[1:]==pytest.approx([thickness-2*SUPPORT_CLEARANCE_M]*3)
        assert 0<SUPPORT_CLEARANCE_M<=.0002
