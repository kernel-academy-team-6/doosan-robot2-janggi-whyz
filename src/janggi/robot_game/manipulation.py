"""Home-only vision targets, fixed grasp attitude, and bounded visual retries.

Pure geometry and injected vision/picker interfaces; no ROS calls at import time.
"""
from dataclasses import dataclass, fields
import math
import numpy as np
from observations import SnapshotTimeout, TargetNotFound


@dataclass(frozen=True)
class ManipulationOptions:
    adaptive_grasp_enabled: bool = False
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
        result=cls(**{f.name:parameters[f.name] for f in fields(cls)
                      if f.name in parameters and f.name!='adaptive_grasp_enabled'})
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


def choose_orthogonal_direction(point,neighbors,grip_width,open_width,home_yaw,options):
    """Count pieces in the two jaw swept corridors; choose only 0/90deg.

    Counts are a preference, never a clearance constraint or a stop condition.
    A tie retains the original flat direction. Far-away pieces do not vote.
    """
    counts=[]
    for offset in (0.,90.):
        angle=math.radians(home_yaw+offset);c,s=math.cos(angle),math.sin(angle)
        count=0
        for neighbor in neighbors:
            dx,dy=neighbor.point[:2]-np.asarray(point)[:2]
            x,y=dx*c+dy*s,-dx*s+dy*c
            a=grip_width/2;b=open_width/2+options.finger_depth_mm
            distance=min(rectangle_distance(x,y,a,b,options.finger_half_width_mm),
                         rectangle_distance(-x,y,a,b,options.finger_half_width_mm))
            if distance<=neighbor.radius+options.neighbor_clearance_mm:count+=1
        counts.append(count)
    return (90. if counts[1]<counts[0] else 0.),tuple(counts)


def locate(scene, token, expected, point, max_shift):
    candidates=[o for o in scene if o.piece==expected and np.linalg.norm(o.point[:2]-point[:2])<=max_shift]
    if token is not None:
        identified=[o for o in scene if o.piece_id==token and np.linalg.norm(o.point[:2]-point[:2])<=max_shift
                    and (o.piece==expected or o.piece[0]=='unknown')]
        if len(identified)==1: return identified[0]
    candidates.sort(key=lambda o:float(np.linalg.norm(o.point[:2]-point[:2])))
    if not candidates or (len(candidates)>1 and
            np.linalg.norm(candidates[1].point[:2]-point[:2])-np.linalg.norm(candidates[0].point[:2]-point[:2])<8.):
        raise TargetNotFound('집을 말의 새 위치를 유일하게 확인하지 못했습니다. 이전 좌표로 집지 않습니다.')
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
        elif (not src and not occupied and (not matching or all(o.cell is None for o in matching))
              and not any(np.linalg.norm(o.point[:2]-source[:2])<=tolerance for o in scene)):
            outcomes.append('missing')
        else: outcomes.append('uncertain')
    return outcomes[0] if outcomes and len(set(outcomes))==1 else 'uncertain'


class ManipulationCoordinator:
    def __init__(self, game, vision, options, check):
        self.g,self.vision,self.o,self.check=game,vision,options,check
        self.p=game.picker

    def notify_retry(self, kind):
        """Optional nonblocking UI notice; audio failure cannot stop manipulation."""
        callback=getattr(self.g,'on_manipulation_retry',None)
        if callback is not None:
            try:callback(kind)
            except Exception as exc:
                self.g.log(f'[재시도 안내 오류] {exc}')

    def scenes(self):
        frames=self.vision.robot_snapshot(samples=self.o.verification_frames,
                                          timeout=getattr(self,'_snapshot_timeout',self.o.verification_timeout_sec),check=self.check)
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
        self.p.set_operation_yaw(None)
        if getattr(self.p,'yaw',None) is not None:
            return  # Explicit absolute-yaw CLI remains an operator override.
        group=2 if kind=='king' else 0 if kind in ('sa','soldier') else 1
        settings=getattr(self.p,'settings',None)
        width=settings.piece_grip_widths_mm[group] if settings else (20.,26.,32.)[group]
        allowance=max(settings.default_opening_allowance_mm,settings.release_extra_width_mm) if settings else 10.
        neighbors=[o for o in scene if not (token and o.piece_id==token)
                   and np.linalg.norm(o.point[:2]-np.asarray(point)[:2])>2.]
        home_yaw=self.p.home_grasp_yaw() if hasattr(self.p,'home_grasp_yaw') else 0.
        offset,counts=choose_orthogonal_direction(point,neighbors,width,width+allowance,home_yaw,self.o)
        self.g.log(f'[집기 방향] 기본 방향 주변 {counts[0]}개 · 직교 방향 {counts[1]}개 · 선택 +{offset:g}°')
        if offset:
            if hasattr(self.p,'set_orthogonal_grasp'):
                self.p.set_orthogonal_grasp(offset)
            else:
                self.p.set_operation_yaw(home_yaw+offset)

    def observe_home(self):
        """Acquire only frames captured after homing has completed."""
        for attempt in range(3):
            self.check()
            self.g.go_observe()  # Motion errors/cancellation are not vision retries.
            self._snapshot_timeout=self.o.verification_timeout_sec*(attempt+1)
            try:return self.scenes()
            except SnapshotTimeout as exc:
                if attempt==2:raise
                self.g.log(f'[영상 자동 복구 {attempt+1}/2] {exc} · 홈 복귀 후 안정화·재인식')
                self.notify_retry('vision')

    def find_at_home(self, frames, expected, point, token=None):
        for attempt in range(3):
            self.check()
            try:return self.target(frames[-1],expected,point,token),frames
            except TargetNotFound as exc:
                if attempt==2:raise
                self.g.log(f'[위치 자동 복구 {attempt+1}/2] {exc} · 홈에서 다시 확인')
                self.notify_retry('vision')
                frames=self.observe_home()

    def pick(self, expected, point, token=None, *, home_scenes=None):
        self.check(); self.p.set_piece_kind(expected[1])
        frames=self.observe_home() if home_scenes is None else home_scenes
        scene=frames[-1]
        if self.o.refresh_before_pick:
            seen,frames=self.find_at_home(frames,expected,point,token)
            scene=frames[-1]
        else:seen=None
        target=seen.point.copy() if seen else np.asarray(point).copy()
        token=seen.piece_id if seen and seen.piece_id else token
        self.orient(scene,target,expected[1],token)
        self.g.log(f'[홈에서 집기 위치 확인] {np.round(target[:2],2).tolist()}mm')
        # No callback / camera lookup after the arm leaves home.
        self.p.pick_abs(target)
        return token,target

    def place(self, plan, token, scene):
        point=self.g.geom.board_to_base(*plan.dst)
        self.orient(scene,point,plan.piece[1],token)
        self.p.place(*plan.dst)

    def capture_only(self, expected, point, *, frames=None):
        """Physical capture -> grave -> HOME observation, including the usual retry."""
        frames=self.observe_home() if frames is None else frames
        captured,frames=self.find_at_home(frames,expected,point)
        for attempt in range(2 if self.o.retry_move_enabled else 1):
            self.pick(expected,captured.point,captured.piece_id,home_scenes=frames)
            self.p.discard()
            frames=self.observe_home()
            remains=any(np.linalg.norm(o.point[:2]-captured.point[:2])<=self.o.verification_tolerance_mm
                        for o in frames[-1])
            if not remains:return frames,True
            if attempt==0 and self.o.retry_move_enabled:
                self.g.log('[포획 재시도] 상대 말이 남아 있어 한 번 더 치웁니다.')
                self.notify_retry('capture')
        self.g.log('[포획 확인] 말이 남아 보입니다.')
        return frames,False

    def execute(self, plan):
        frames=self.observe_home()
        initial=frames[-1]
        if self.o.refresh_before_pick:
            own,frames=self.find_at_home(frames,plan.piece,plan.pick_mm)
        else:own=None
        token=own.piece_id if own else None
        source=own.point if own else plan.pick_mm
        if plan.captured:
            frames,cleared=self.capture_only(plan.captured,plan.capture_mm,frames=frames)
            if not cleared and plan.captured[1]=='king':
                self.g.log('[궁 포획 미확인] 승리를 선언하지 않습니다. 현재 판에서 다시 시도하세요.')
                self.g._execution_observed=True
                return False
        token,source=self.pick(plan.piece,source,token,home_scenes=frames)
        self.attempt_place(lambda:self.place(plan,token,frames[-1]))
        frames=self.observe_home()
        if self.o.retry_move_enabled:
            destination=self.g.geom.board_to_base(*plan.dst)
            move_retries=place_retries=0
            reobserved=False
            while True:
                self.check()
                state=move_evidence(frames,token,plan.piece,source,destination,self.o.verification_tolerance_mm)
                self.g.log(f'[착수 확인] {state}')
                if state=='placed':
                    if hasattr(self.p,'mark_place_observed'):self.p.mark_place_observed()
                    break
                if state=='unchanged' and move_retries<1:
                    move_retries+=1
                    self.g.log('[이동 재시도 1/1] 원위치에 남은 말 · 집기부터 다시 수행')
                    self.notify_retry('move')
                    token,source=self.pick(plan.piece,source,token,home_scenes=frames)
                    self.attempt_place(lambda:self.place(plan,token,frames[-1]))
                elif state=='missing' and place_retries<1:
                    place_retries+=1
                    self.g.log('[놓기 재시도 1/1] 미놓임 추정 (파지 확인 아님) · 놓기만 다시 수행')
                    self.notify_retry('place')
                    self.orient(frames[-1],destination,plan.piece[1],token)
                    self.attempt_place(lambda:self.p.retry_place(*plan.dst))
                elif state=='uncertain' and not reobserved:
                    reobserved=True
                    self.g.log('[착수 재확인] 판단 모호 · 홈에서 안정 영상을 다시 확인합니다.')
                    self.notify_retry('vision')
                else:
                    self.g.log('[재시도 확인] 착수 미확인 · 더 반복하지 않고 계속 진행합니다.')
                    break
                frames=self.observe_home()
        self.g._execution_observed=True
        return True

    def attempt_place(self, action):
        try:return action()
        except Exception as exc:
            # Only a failed gripper release with a completed retreat is
            # recoverable here. Never retry robot motion errors/cancellation.
            if not getattr(exc,'recoverable_release_failure',False) or not self.o.retry_move_enabled:
                raise
            self.check()
            self.g.log(f'[놓기 명령 실패] {exc} · 홈 관측 후 필요한 재시도만 수행합니다.')
