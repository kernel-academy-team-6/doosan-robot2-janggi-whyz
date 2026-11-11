#!/usr/bin/env python3
"""Render the user's original 14 SVG designs for Gazebo. Run with system Python (Rsvg/cairo)."""
import hashlib
import json
import re
from io import BytesIO
from PIL import Image
from pathlib import Path

import cairo
import gi
gi.require_version('Rsvg','2.0')
from gi.repository import Rsvg

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'tools/piece_dataset/assets/pieces'
OUTPUT=ROOT/'src/janggi/robot_game/assets/pieces'


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True)
    provenance={}
    for svg in sorted(SOURCE.glob('*.svg')):
        text=svg.read_text()
        if svg.stem.startswith('blue_'):
            # Preserve the source artwork and its shading; swap blue/green only
            # in blue-dominant fills for the Gazebo display variant.
            def green_fill(match):
                color=match.group(0)
                red,green,blue=(int(color[i:i+2],16) for i in (1,3,5))
                return f'#{red:02x}{blue:02x}{green:02x}' if blue>max(red,green) else color
            text=re.sub(r'#[0-9a-fA-F]{6}\b',green_fill,text)
        handle=Rsvg.Handle.new_from_data(text.encode())
        surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,512,512)
        viewport=Rsvg.Rectangle();viewport.x=viewport.y=0.;viewport.width=viewport.height=512.
        if not handle.render_document(cairo.Context(surface),viewport):
            raise RuntimeError(f'SVG rendering failed: {svg}')
        buffer=BytesIO();surface.write_to_png(buffer);buffer.seek(0)
        image=Image.open(buffer).convert('RGBA')
        bounds=image.getchannel('A').getbbox()
        if bounds is None:raise RuntimeError(f'Empty SVG design: {svg}')
        image.crop(bounds).resize((512,512),Image.Resampling.LANCZOS).save(OUTPUT/(svg.stem+'.png'))
        provenance[svg.stem]=dict(source=str(svg.relative_to(ROOT)),sha256=hashlib.sha256(svg.read_bytes()).hexdigest(),
                                  renderer='librsvg/cairo; transparent margins trimmed; 512x512 RGBA',
                                  display_color='green' if svg.stem.startswith('blue_') else 'red',purpose='Gazebo rendered surface only')
    if len(provenance)!=14:raise RuntimeError('Expected all 14 user SVG designs')
    (OUTPUT/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+'\n')
    print(f'Rendered {len(provenance)} original SVG designs to {OUTPUT}')


if __name__=='__main__':main()
