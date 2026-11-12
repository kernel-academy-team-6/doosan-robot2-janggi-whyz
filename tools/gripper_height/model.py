"""Measurement sequence, persistence and geometric circle fitting (no ROS/Tk)."""
from dataclasses import dataclass, asdict
from datetime import datetime
import csv
import json
import math
from pathlib import Path
import tempfile

import numpy as np
from scipy.optimize import least_squares

COMMANDS = tuple(range(700, -1, -50))
COLUMNS = ('command', 'width_mm', 'half_width_mm', 'flange_x_mm', 'flange_y_mm',
           'flange_z_mm', 'a_deg', 'b_deg', 'c_deg', 'relative_z_mm', 'recorded_at')


def number(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name}: 유한한 숫자를 입력하세요.')
    return value


@dataclass(frozen=True)
class Sample:
    command: int
    width_mm: float
    flange_pose: tuple
    recorded_at: str


@dataclass(frozen=True)
class CircleFit:
    center_half_width_mm: float
    center_relative_z_mm: float
    radius_mm: float
    branch: int
    reference_width_mm: float
    reference_flange_z_mm: float
    radial_rmse_mm: float
    radial_max_error_mm: float
    jacobian_condition: float

    def curve_z(self, width_mm):
        width = number(width_mm, '너비')
        if width < 0:
            raise ValueError('너비는 0 이상이어야 합니다.')
        square = self.radius_mm**2-(width/2-self.center_half_width_mm)**2
        if square < -1e-8:
            raise ValueError('이 너비는 피팅한 원의 정의역 밖입니다.')
        return self.center_relative_z_mm+self.branch*math.sqrt(max(0.,square))

    def height_from_open(self, width_mm):
        """Command-0 gap is exactly zero even when the fitted circle has residual."""
        return self.curve_z(width_mm)-self.curve_z(self.reference_width_mm)


class MeasurementSession:
    def __init__(self, directory, *, backend='real', namespace='dsr01'):
        self.directory = Path(directory)
        self.backend, self.namespace = backend, namespace
        self.samples = []
        self.active_command = None  # Only set after the hardware command completes.

    @property
    def next_command(self):
        return COMMANDS[len(self.samples)] if len(self.samples)<len(COMMANDS) else None

    @property
    def reference_z(self):
        return next((s.flange_pose[2] for s in self.samples if s.command==0),None)

    def mark_active(self, command):
        if command != self.next_command:
            raise ValueError('측정 순서와 그리퍼 명령이 일치하지 않습니다.')
        self.active_command = command

    def record(self, width_mm, pose):
        if self.next_command is None:
            raise ValueError('모든 측정을 마쳤습니다.')
        if self.active_command != self.next_command:
            raise ValueError('현재 단계의 그리퍼 명령을 먼저 적용하세요.')
        width = number(width_mm,'측정 너비')
        pose = tuple(number(v,'플랜지 좌표') for v in pose)
        if width < 0 or len(pose)!=6:
            raise ValueError('너비는 0 이상, 플랜지 자세는 6개 값이어야 합니다.')
        if self.samples and math.dist(pose[:2],self.samples[0].flange_pose[:2])>.2:
            raise ValueError('측정 중 XY가 달라졌습니다. 같은 XY에서 다시 측정하세요.')
        sample = Sample(self.next_command,width,pose,datetime.now().astimezone().isoformat())
        self.samples.append(sample)
        try:
            self.save()
        except BaseException:
            self.samples.pop()
            raise
        self.active_command = None
        return sample

    def save(self):
        self.directory.mkdir(parents=True,exist_ok=True)
        baseline = self.reference_z
        with tempfile.NamedTemporaryFile('w',dir=self.directory,delete=False,newline='',encoding='utf-8') as stream:
            temporary = Path(stream.name)
            writer = csv.DictWriter(stream,fieldnames=COLUMNS)
            writer.writeheader()
            for s in self.samples:
                x,y,z,a,b,c = s.flange_pose
                writer.writerow(dict(command=s.command,width_mm=s.width_mm,half_width_mm=s.width_mm/2,
                    flange_x_mm=x,flange_y_mm=y,flange_z_mm=z,a_deg=a,b_deg=b,c_deg=c,
                    relative_z_mm='' if baseline is None else z-baseline,recorded_at=s.recorded_at))
        temporary.replace(self.directory/'measurements.csv')
        metadata = dict(backend=self.backend,namespace=self.namespace,commands=COMMANDS,
                        recorded=len(self.samples),next_command=self.next_command,
                        height_reference_command=0,reference_flange_z_mm=baseline,
                        coordinates='DR_BASE; unnamed J6 flange TCP; mm/deg',
                        automatic_robot_compensation_applied=False)
        (self.directory/'session.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        # A changed measurement table must not leave an old fit looking current.
        for name in ('fit.json','fit.png','report.html','lookup.csv','fit_residuals.csv'):
            (self.directory/name).unlink(missing_ok=True)

    @classmethod
    def load(cls, path):
        path = Path(path)
        metadata_path = path.parent/'session.json'
        metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
        session = cls(path.parent,backend=metadata.get('backend','unknown'),namespace=metadata.get('namespace','dsr01'))
        with path.open(newline='',encoding='utf-8-sig') as stream:
            for index,row in enumerate(csv.DictReader(stream)):
                command = int(row['command'])
                width = number(row['width_mm'],'width')
                pose = tuple(number(row[k],k) for k in ('flange_x_mm','flange_y_mm','flange_z_mm','a_deg','b_deg','c_deg'))
                if index>=len(COMMANDS) or command!=COMMANDS[index] or width<0:
                    raise ValueError('측정 CSV의 순서 또는 너비가 잘못되었습니다.')
                if session.samples and math.dist(pose[:2],session.samples[0].flange_pose[:2])>.2:
                    raise ValueError('측정 CSV의 XY가 일치하지 않습니다.')
                session.samples.append(Sample(command,width,pose,row['recorded_at']))
        return session


def fit_circle(session):
    if tuple(s.command for s in session.samples)!=COMMANDS or session.reference_z is None:
        raise ValueError('700부터 0까지 15개 측정이 모두 필요합니다.')
    points = np.array([[s.width_mm/2,s.flange_pose[2]-session.reference_z] for s in session.samples])
    mean = points.mean(axis=0)
    scale = float(np.max(np.ptp(points,axis=0)))
    if scale<1e-6 or np.linalg.svd(points-mean,compute_uv=False)[-1]<scale*1e-5:
        raise ValueError('측정점이 중복되거나 거의 직선이어서 원을 결정할 수 없습니다.')
    normalized = (points-mean)/scale
    matrix = np.column_stack([2*normalized,np.ones(len(points))])
    solution,_,rank,_ = np.linalg.lstsq(matrix,np.sum(normalized**2,axis=1),rcond=None)
    if rank<3:
        raise ValueError('원을 결정할 수 없는 측정점입니다.')
    radius2 = solution[2]+solution[0]**2+solution[1]**2
    if radius2<=0:
        raise ValueError('양의 반지름을 구할 수 없습니다.')
    def residual(p):
        return np.linalg.norm(normalized-p[:2],axis=1)-p[2]
    result = least_squares(residual,[*solution[:2],math.sqrt(radius2)],
                           bounds=([-np.inf,-np.inf,1e-9],[np.inf,np.inf,np.inf]),max_nfev=5000)
    condition = float(np.linalg.cond(result.jac))
    if not result.success or not np.isfinite(result.x).all() or not math.isfinite(condition) or condition>1e10:
        raise ValueError('원 피팅이 수렴하지 않거나 반지름이 불안정합니다.')
    center = result.x[:2]*scale+mean
    radius = float(result.x[2]*scale)
    errors = residual(result.x)*scale
    branch = 1 if np.mean(points[:,1]-center[1])>=0 else -1
    if np.any((points[:,1]-center[1])*branch < -max(.05,3*np.std(errors))):
        raise ValueError('측정점이 원의 양쪽 가지에 걸쳐 있어 너비만으로 높이를 결정할 수 없습니다.')
    fit = CircleFit(float(center[0]),float(center[1]),radius,branch,
                    session.samples[-1].width_mm,session.reference_z,
                    float(np.sqrt(np.mean(errors**2))),float(np.max(np.abs(errors))),condition)
    for s in session.samples:
        fit.curve_z(s.width_mm)
    return fit
