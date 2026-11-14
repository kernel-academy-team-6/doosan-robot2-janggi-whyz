"""Independent four-contact bed leveling in robot base millimetres; no ROS."""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import yaml

DEFAULT_HEIGHT_MAP = Path(__file__).resolve().parents[1]/'calibration/config/board_height.yaml'
NAMES=('top_left','top_right','bottom_right','bottom_left')


def bilinear_weights(xy, corners_xy, *, extrapolate=False):
    """Invert the measured XY quadrilateral; clamp outside points by default."""
    xy=np.asarray(xy,float);corners=np.asarray(corners_xy,float)
    if xy.shape!=(2,) or corners.shape!=(4,2) or not np.isfinite(xy).all() or not np.isfinite(corners).all():
        raise ValueError('Board height requires finite XY and four finite corners')
    tl,tr,br,bl=corners
    if abs(np.linalg.det(np.column_stack((tr-tl,bl-tl))))<1e-6:
        raise ValueError('Degenerate board height XY corners')
    uv=np.array([.5,.5])
    for _ in range(30):
        u,v=uv
        actual=tl*(1-u)*(1-v)+tr*u*(1-v)+br*u*v+bl*(1-u)*v
        if np.linalg.norm(xy-actual)<1e-8:break
        jac=np.column_stack(((tr-tl)*(1-v)+(br-bl)*v,(bl-tl)*(1-u)+(br-tr)*u))
        try:uv+=np.linalg.solve(jac,xy-actual)
        except np.linalg.LinAlgError as exc:raise ValueError('Degenerate board height XY corners') from exc
    u,v=uv
    actual=tl*(1-u)*(1-v)+tr*u*(1-v)+br*u*v+bl*(1-u)*v
    if not np.isfinite(uv).all() or np.linalg.norm(xy-actual)>.001:
        raise ValueError('Board height XY interpolation did not converge')
    if not extrapolate:u,v=np.clip(uv,0.,1.)
    return np.array([(1-u)*(1-v),u*(1-v),u*v,(1-u)*v])


@dataclass(frozen=True)
class BoardHeightMap:
    xy_mm: np.ndarray
    flange_z_mm: np.ndarray
    probe_tool_length_mm: float
    support_z_mm: float
    source: str

    @classmethod
    def from_file(cls,path=DEFAULT_HEIGHT_MAP):
        data=yaml.safe_load(Path(path).read_text())
        if data.get('schema_version')!=1 or data.get('frame')!='base_link' or data.get('units')!='mm':
            raise ValueError('Unsupported board height measurement frame/units/schema')
        if not isinstance(data.get('enabled'),bool):raise ValueError('Board height enabled must be boolean')
        if not data['enabled']:return None
        corners={p['name']:p for p in data['corners']}
        xy=np.array([corners[n]['xy_mm'] for n in NAMES],float)
        zs=np.array([corners[n]['flange_z_mm'] for n in NAMES],float)
        length=float(data['probe_tool_length_mm']);support=float(data['support_z_mm'])
        if xy.shape!=(4,2) or zs.shape!=(4,) or not np.isfinite(xy).all() or not np.isfinite(zs).all() or not np.isfinite([length,support]).all() or length<=0:
            raise ValueError('Invalid board height contact data')
        if abs(np.linalg.det(np.column_stack((xy[1]-xy[0],xy[3]-xy[0]))))<1e-6:
            raise ValueError('Degenerate board height XY corners')
        xy.setflags(write=False);zs.setflags(write=False)
        return cls(xy,zs,length,support,str(Path(path).resolve()))

    @property
    def surface_corner_z_mm(self):
        return self.flange_z_mm-self.probe_tool_length_mm

    def surface_z(self,x_mm,y_mm,*,extrapolate=False):
        """Absolute board Z: interpolate contact flange Z minus probe length.
        A deliberate extrapolate=True is available for analysis, not used by picks.
        """
        weights=bilinear_weights([x_mm,y_mm],self.xy_mm,extrapolate=extrapolate)
        return float(weights@self.surface_corner_z_mm)

    def height_above_support(self,x_mm,y_mm,*,extrapolate=False):
        return self.surface_z(x_mm,y_mm,extrapolate=extrapolate)-self.support_z_mm


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('x',type=float);p.add_argument('y',type=float)
    p.add_argument('--extrapolate',action='store_true');p.add_argument('--map',type=Path,default=DEFAULT_HEIGHT_MAP)
    args=p.parse_args();bed=BoardHeightMap.from_file(args.map)
    if bed is None:p.exit(1,'Board height map disabled\n')
    print(f'base XY=({args.x:g}, {args.y:g}) mm; '+('extrapolation allowed' if args.extrapolate else 'outside points clamped to board edges'))
    print(f'board Z={bed.surface_z(args.x,args.y,extrapolate=args.extrapolate):.6f} mm; '
          f'height above support={bed.height_above_support(args.x,args.y,extrapolate=args.extrapolate):.6f} mm')
