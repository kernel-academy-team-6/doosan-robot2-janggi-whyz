from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from piece_visuals import ASSETS,piece_visuals


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
