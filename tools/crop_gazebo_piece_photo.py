#!/usr/bin/env python3
"""Extract all 32 physical piece faces from the user's board photograph."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'src/janggi/robot_game/assets/pieces'
# Centres measured in the 1368x1824 chat preview; extraction uses native pixels.
CENTRES={
 'blue_cha':[(108,354),(1264,351)],
 'blue_sang':[(251,353),(986,352)],
 'blue_horse':[(397,353),(1117,351)],
 'blue_sa':[(543,353),(831,351)],
 'blue_king':[(704,481)],
 'blue_po':[(253,601),(1124,600)],
 'blue_soldier':[(108,735),(397,735),(686,737),(980,732),(1268,724)],
 'red_soldier':[(113,1133),(396,1130),(695,1130),(982,1129),(1285,1120)],
 'red_po':[(246,1266),(1139,1256)],
 'red_king':[(692,1404)],
 'red_cha':[(97,1535),(1290,1524)],
 'red_horse':[(241,1526),(1000,1526)],
 'red_sang':[(394,1530),(1141,1526)],
 'red_sa':[(546,1527),(842,1521)],
}


def extract_face(rgb,center,radius):
    height,width=rgb.shape[:2]
    scale=np.array([width/1368.,height/1824.])
    cx,cy=np.round(np.array(center)*scale).astype(int)
    rx,ry=np.ceil(radius*scale).astype(int)
    x0,y0=max(0,cx-rx),max(0,cy-ry)
    roi=rgb[y0:min(height,cy+ry),x0:min(width,cx+rx)]
    hsv=cv2.cvtColor(roi,cv2.COLOR_RGB2HSV)
    white=((hsv[:,:,1]<40)&(hsv[:,:,2]>195)).astype(np.uint8)*255
    white=cv2.morphologyEx(white,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(white,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    candidates=[c for c in contours if cv2.contourArea(c)>roi.shape[0]*roi.shape[1]*.18]
    if not candidates:raise ValueError(f'No complete white face near {center}')
    contour=max(candidates,key=cv2.contourArea)
    hull=cv2.convexHull(contour)
    x,y,w,h=cv2.boundingRect(hull)
    if x<=0 or y<=0 or x+w>=roi.shape[1] or y+h>=roi.shape[0]:
        raise ValueError(f'Face touches crop boundary near {center}; enlarge extraction radius')
    # Keep exact photographed RGB; only mask the wooden background outside the face.
    mask=np.zeros(roi.shape[:2],np.uint8);cv2.fillConvexPoly(mask,hull,255)
    rgba=np.dstack([roi,mask])[y:y+h,x:x+w]
    native=Image.fromarray(rgba)
    side=max(w,h);square=Image.new('RGBA',(side,side),(0,0,0,0))
    square.paste(native,((side-w)//2,(side-h)//2))
    texture=square.resize((512,512),Image.Resampling.LANCZOS)
    return texture,[int(x0+x),int(y0+y),int(w),int(h)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('photo',nargs='?',type=Path,default=ROOT/'docs/IMG_1183.jpg')
    args=parser.parse_args();source=args.photo.resolve()
    image=Image.open(source).convert('RGB');rgb=np.array(image)
    if abs(image.width/image.height-.75)>.01:raise ValueError('Expected the supplied portrait board photo')
    instances=ASSETS/'instances';instances.mkdir(parents=True,exist_ok=True)
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    common=dict(source=str(source.relative_to(ROOT)),sha256=digest,source_size=list(image.size),
                method='native-resolution photographic RGB crop; white-face convex hull alpha; no recolouring',
                purpose='Gazebo rendered surface only')
    provenance={};manifest={};tiles=[]
    for label,centres in CENTRES.items():
        kind=label.split('_',1)[1]
        radius=74 if kind=='king' else 48 if kind in ('sa','soldier') else 64
        for index,center in enumerate(centres):
            texture,box=extract_face(rgb,center,radius)
            key=f'{label}_{index}'
            texture.save(instances/(key+'.png'))
            record=dict(common,crop_xywh=box,texture_key=key,label=label,instance=index)
            manifest[key]=record
            if index==0:
                texture.save(ASSETS/(label+'.png'));provenance[label]=record
            tile=Image.new('RGB',(180,205),'#dddddd')
            tile.paste(texture.resize((170,170)),(5,5),texture.resize((170,170)).getchannel('A'))
            ImageDraw.Draw(tile).text((5,182),key,fill='black');tiles.append(tile)
    (ASSETS/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+'\n')
    (ASSETS/'photo_instances.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    sheet=Image.new('RGB',(180*8,205*4),'white')
    for i,tile in enumerate(tiles):sheet.paste(tile,((i%8)*180,(i//8)*205))
    sheet.save(ASSETS/'photo_contact_sheet.jpg',quality=93)
    print(f'Extracted {len(manifest)} individual faces and {len(provenance)} class defaults from {source}')


if __name__=='__main__':main()
