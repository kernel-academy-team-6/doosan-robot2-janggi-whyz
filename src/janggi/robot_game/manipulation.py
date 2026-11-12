"""Fresh-vision grasp direction selection and bounded, evidence-based retries.

Pure geometry and injected vision/picker interfaces; no ROS calls at import time.
"""
from dataclasses import dataclass, fields
import math
import numpy as np


@dataclass(frozen=True)
class ManipulationOptions:
    adaptive_grasp_enabled: bool = True
    refresh_before_pick: bool = True
    retry_move_enabled: bool = True
    verification_frames: int = 3
    verification_timeout_sec: float = 3.
    verification_tolerance_mm: float = 12.
    pick_refresh_max_shift_mm: float = 45.
    finger_half_width_mm: float = 13.
    finger_depth_mm: float = 22.
    neighbor_clearance_mm: float = 2.

    @classmethod
    def from_parameters(cls, parameters):
        result=cls(**{f.name:parameters[f.name] for f in fields(cls) if f.name in parameters})
        for name in ('adaptive_grasp_enabled','refresh_before_pick','retry_move_enabled'):
            if not isinstance(getattr(result,name),bool): raise ValueError(f'{name} must be boolean')
        if not isinstance(result.verification_frames,int) or not 1<=result.verification_frames<=6:
            raise ValueError('verification_frames must be an integer from 1 to 6')
        for f in fields(cls):
            value=getattr(result,f.name)
            if not isinstance(value,bool) and (not math.isfinite(value) or value<=0):
                raise ValueError(f'{f.name} must be finite and positive')
        return result


@dataclass
class SeenPiece:
    piece_id: str | None
    cell: tuple | None
    piece: tuple
    point: np.ndarray
    radius: float


def rectangle_distance(x, y, x0, x1, half_y):
    dx=max(x0-x,0.,x-x1); dy=max(abs(y)-half_y,0.)
    if dx or dy:
        return math.hypot(dx,dy)
    return -min(x-x0,x1-x,half_y-abs(y))


def choose_yaw(point, neighbors, grip_width, open_width, preferred, options):
    """Maximize minimum 2-D finger swept-envelope clearance; jaws are 180° symmetric.

    URDF finger x width is 26mm; jaw opening is gripper Y, mounted Rz(90°)
    onto flange. Its undirected base XY axis is the requested ZYZ yaw mod 180.
    """
    best=None
    for yaw in [preferred]+[preferred+step for step in range(-90,90,5)]:
        angle=math.radians(yaw); c,s=math.cos(angle),math.sin(angle)
        score=float('inf')
        for neighbor in neighbors:
            dx,dy=neighbor.point[:2]-np.asarray(point)[:2]
            x,y=dx*c+dy*s,-dx*s+dy*c
            a=grip_width/2; b=open_width/2+options.finger_depth_mm
            distance=min(rectangle_distance(x,y,a,b,options.finger_half_width_mm),
                         rectangle_distance(-x,y,a,b,options.finger_half_width_mm))
            score=min(score,distance-neighbor.radius-options.neighbor_clearance_mm)
        key=(score,-abs(yaw-preferred))
        if best is None or key>best[0]: best=(key,yaw,score)
    return best[1],best[2]


def locate(scene, token, expected, point, max_shift):
    candidates=[o for o in scene if o.piece==expected and np.linalg.norm(o.point[:2]-point[:2])<=max_shift]
    if token is not None:
        identified=[o for o in scene if o.piece_id==token and np.linalg.norm(o.point[:2]-point[:2])<=max_shift
                    and (o.piece==expected or o.piece[0]=='unknown')]
        if len(identified)==1: return identified[0]
    candidates.sort(key=lambda o:float(np.linalg.norm(o.point[:2]-point[:2])))
    if not candidates or (len(candidates)>1 and
            np.linalg.norm(candidates[1].point[:2]-point[:2])-np.linalg.norm(candidates[0].point[:2]-point[:2])<8.):
        raise RuntimeError('집을 말의 새 위치를 유일하게 확인하지 못했습니다. 이전 좌표로 집지 않습니다.')
    return candidates[0]


def move_evidence(scenes, token, expected, source, destination, tolerance):
    """Every fresh frame must agree; absence alone is not verified holding."""
    outcomes=[]
    for scene in scenes:
        identified=[o for o in scene if token and o.piece_id==token]
        matching=identified or [o for o in scene if o.piece==expected and
                                min(np.linalg.norm(o.point[:2]-source[:2]),
                                    np.linalg.norm(o.point[:2]-destination[:2]))<=tolerance]
        src=[o for o in matching if np.linalg.norm(o.point[:2]-source[:2])<=tolerance]
        dst=[o for o in matching if np.linalg.norm(o.point[:2]-destination[:2])<=tolerance]
        # Occupancy (including unknown pieces) must not be mistaken for empty.
        occupied=any(np.linalg.norm(o.point[:2]-destination[:2])<=tolerance for o in scene)
        if dst and not src: outcomes.append('placed')
        elif src and not occupied: outcomes.append('unchanged')
        elif (not src and not occupied and not matching
              and not any(np.linalg.norm(o.point[:2]-source[:2])<=tolerance for o in scene)):
            outcomes.append('missing')
        else: outcomes.append('uncertain')
    return outcomes[0] if outcomes and len(set(outcomes))==1 else 'uncertain'


class ManipulationCoordinator:
    def __init__(self, game, vision, options, check):
        self.g,self.vision,self.o,self.check=game,vision,options,check
        self.p=game.picker

    def scenes(self):
        frames=self.vision.robot_snapshot(samples=self.o.verification_frames,
                                          timeout=self.o.verification_timeout_sec,check=self.check)
        scenes=[]
        for frame in frames:
            scene=[]
            for raw in frame['objects']:
                piece=tuple(raw['piece']); kind=piece[1]
                known=kind in ('sa','soldier','cha','horse','sang','po','king')
                height=self.p.piece_height_for_kind(kind) if known else 8.
                point=np.asarray(self.g.cam.pixel_to_base(*raw['pixel'],height_mm=height),float)
                if point.shape!=(3,) or not np.isfinite(point).all():
                    raise RuntimeError('새 검출 위치를 로봇 좌표로 변환하지 못했습니다.')
                group=2 if kind=='king' else 0 if kind in ('sa','soldier') else 1
                diameter=self.p.settings.piece_min_widths_mm[group] if known else max(self.p.settings.piece_min_widths_mm)
                scene.append(SeenPiece(raw.get('piece_id'),raw.get('cell'),piece,point,
                                       float(diameter)/(2*math.cos(math.pi/8))))
            scenes.append(scene)
        return scenes

    def target(self, scene, expected, point, token=None):
        return locate(scene,token,expected,np.asarray(point),self.o.pick_refresh_max_shift_mm)

    def orient(self, scene, point, kind, token=None):
        if not self.o.adaptive_grasp_enabled:
            self.p.set_operation_yaw(None); return
        group=2 if kind=='king' else 0 if kind in ('sa','soldier') else 1
        width=self.p.settings.piece_grip_widths_mm[group]
        opening=width+max(self.p.settings.default_opening_allowance_mm,self.p.settings.release_extra_width_mm)
        neighbors=[o for o in scene if not (token and o.piece_id==token)
                   and np.linalg.norm(o.point[:2]-np.asarray(point)[:2])>2.]
        yaw,clearance=choose_yaw(point,neighbors,width,opening,self.p.current_yaw(),self.o)
        if clearance<0:
            raise RuntimeError(f'주변 말과 손가락 사이 여유가 부족합니다 (최선 {clearance:.1f}mm).')
        self.g.log(f'[집기 방향] yaw={yaw:.1f}° · 최소 추정 여유={clearance:.1f}mm')
        self.p.set_operation_yaw(yaw)

    def pick(self, expected, point, token=None):
        self.check(); self.p.set_piece_kind(expected[1])
        scene=self.scenes()[-1]
        seen=self.target(scene,expected,point,token) if self.o.refresh_before_pick else None
        target=seen.point if seen else np.asarray(point)
        token=seen.piece_id if seen and seen.piece_id else token
        self.orient(scene,target,expected[1],token)
        actual=[np.asarray(target).copy()]
        def refresh():
            nonlocal token
            fresh=self.scenes()[-1]
            try: updated=self.target(fresh,expected,actual[0],token)
            except RuntimeError:
                # The wrist can occlude the piece. Retract and re-image once;
                # never silently substitute the original planned coordinate.
                self.g.log('[집기 재인식] 가림/위치 불명확 · 관찰 자세에서 다시 확인')
                self.g.go_observe(); fresh=self.scenes()[-1]
                updated=self.target(fresh,expected,actual[0],token)
            actual[0]=updated.point
            if updated.piece_id is not None: token=updated.piece_id
            self.orient(fresh,actual[0],expected[1],token)
            self.g.log(f'[집기 직전 위치] {np.round(actual[0][:2],2).tolist()}mm')
            return actual[0]
        self.p.pick_abs(target,refresh=refresh if self.o.refresh_before_pick else None)
        return token,actual[0]

    def place(self, plan, token, scene):
        point=self.g.geom.board_to_base(*plan.dst)
        if any((not token or o.piece_id!=token) and np.linalg.norm(o.point[:2]-point[:2])<=self.o.verification_tolerance_mm for o in scene):
            raise RuntimeError('놓을 위치에 다른 말이 관측되어 플레이스를 실행하지 않았습니다.')
        self.orient(scene,point,plan.piece[1],token)
        self.p.place(*plan.dst)

    def execute(self, plan):
        initial=self.scenes()[-1]
        own=self.target(initial,plan.piece,plan.pick_mm) if self.o.refresh_before_pick else None
        token=own.piece_id if own else None
        source=own.point if own else plan.pick_mm
        if plan.captured:
            captured=self.target(initial,plan.captured,plan.capture_mm)
            for attempt in range(2 if self.o.retry_move_enabled else 1):
                self.pick(plan.captured,captured.point,captured.piece_id)
                self.p.discard(); self.g.go_observe()
                clear=self.scenes()[-1]
                if not any(np.linalg.norm(o.point[:2]-captured.point[:2])<=self.o.verification_tolerance_mm for o in clear): break
                if attempt==0 and self.o.retry_move_enabled:
                    self.g.log('[포획 재시도] 상대 말이 남아 있어 한 번 더 치웁니다.')
                else: raise RuntimeError('포획 말이 남아 있어 자기 말 착수를 진행하지 않았습니다.')
        token,source=self.pick(plan.piece,source,token)
        self.place(plan,token,self.scenes()[-1])
        self.g.go_observe()
        if self.o.retry_move_enabled:
            destination=self.g.geom.board_to_base(*plan.dst)
            state=move_evidence(self.scenes(),token,plan.piece,source,destination,self.o.verification_tolerance_mm)
            self.g.log(f'[착수 확인] {state}')
            if state=='unchanged':
                self.g.log('[이동 재시도 1/1] 원위치에 남은 말 · 집기부터 다시 수행')
                token,source=self.pick(plan.piece,source,token)
                self.place(plan,token,self.scenes()[-1]); self.g.go_observe()
            elif state=='missing':
                self.g.log('[놓기 재시도 1/1] 미놓임 추정 (파지 확인 아님) · 놓기만 다시 수행')
                scene=self.scenes()[-1]
                if any(np.linalg.norm(o.point[:2]-destination[:2])<=self.o.verification_tolerance_mm for o in scene):
                    raise RuntimeError('재시도 전에 목표 위치에 말이 관측되어 놓기를 반복하지 않았습니다.')
                self.orient(scene,destination,plan.piece[1],token)
                self.p.retry_place(*plan.dst); self.g.go_observe()
            elif state!='placed':
                if hasattr(self.p,'mark_holding_unknown'): self.p.mark_holding_unknown()
                raise RuntimeError('착수 관측이 모호하여 자동 재시도하지 않았습니다.')
            if state in ('unchanged','missing'):
                final=move_evidence(self.scenes(),token,plan.piece,source,destination,self.o.verification_tolerance_mm)
                self.g.log(f'[재시도 결과] {final}')
                if final!='placed':
                    if final!='unchanged' and hasattr(self.p,'mark_holding_unknown'): self.p.mark_holding_unknown()
                    raise RuntimeError('한 번 재시도 후에도 착수를 확인하지 못했습니다.')
        self.g._execution_observed=True
