"""Cartesian waypoint planning with a fixed J6; no hardware commands in this module."""
from dataclasses import dataclass, field
import math

from .geometry import joint6_candidates, orientation_error_deg


def line_distance(point, start, end):
    delta=[b-a for a,b in zip(start,end)]
    length2=sum(v*v for v in delta)
    if length2 == 0:
        return math.dist(point,start)
    t=max(0.,min(1.,sum((p-a)*d for p,a,d in zip(point,start,delta))/length2))
    return math.dist(point,[a+t*d for a,d in zip(start,delta)])


@dataclass
class WristPath:
    start_pose: list
    start_joints: list
    target_pose: list
    joints: list
    duration: float
    timing_limits: dict = field(default_factory=dict)


class LockedWristPlanner:
    """IK solves free task yaw; FK checks fixed-J6 solutions and path interiors."""
    def __init__(self, settings, inverse, forward):
        self.s, self.inverse, self.forward = settings, inverse, forward

    def _solve(self, point, seed, yaw, solution, check):
        target=self.s.joint6_locked_deg
        for _ in range(5):
            check()
            q=self.inverse([*point,yaw,180.,0.],solution,check)
            error=math.remainder(q[5]-target,360.)
            if abs(error) <= .005:
                result=[]
                for i,angle in enumerate(q):
                    bound=155. if i in (2,4) else 360.
                    choices=joint6_candidates(angle,seed[i],-bound,bound)
                    if not choices:
                        raise ValueError(f'fixed-J6 IK has no bounded J{i+1} solution')
                    result.append(choices[0])
                result[5]=target
                if any(abs(a-b)>10. for a,b in zip(result,seed)):
                    raise ValueError('fixed-J6 IK branch changed too far between waypoints')
                actual=self.forward(result,check)
                if (math.dist(actual[:3],point) > .1 or
                        orientation_error_deg(actual[3:],[yaw,180.,0.]) > .1):
                    raise ValueError('FK did not verify the fixed-J6 IK solution')
                return result,actual,yaw
            perturbed=self.inverse([*point,yaw+1.,180.,0.],solution,check)
            derivative=math.remainder(perturbed[5]-q[5],360.)
            if abs(derivative) < .1 or not math.isfinite(derivative):
                raise ValueError('cannot solve free yaw while keeping J6 fixed')
            yaw -= max(-90.,min(90.,error/derivative))
        raise ValueError('fixed-J6 IK did not converge; no arm motion sent')

    def plan(self, start_pose, start_joints, target, solution, check):
        if abs(start_joints[5]-self.s.joint6_locked_deg)>self.s.joint6_lock_tolerance_deg:
            raise ValueError('J6 is not at the locked angle; use the explicit home button first')
        if abs(abs(math.remainder(start_pose[4],360.))-180.)>.1:
            raise ValueError('fixed-J6 path requires flange Z pointing down')
        distance=math.dist(start_pose[:3],target[:3])
        if distance <= .05:
            return WristPath(list(start_pose),list(start_joints),list(start_pose),[],0.)
        count=max(2,math.ceil(distance/self.s.joint6_locked_step_mm))
        if count>100:
            if distance>110*self.s.joint6_locked_step_mm:
                raise ValueError('fixed-J6 path needs more than 100 waypoints; no motion sent')
            # Board diagonal can be just over 500mm. Permit at most 10% spacing
            # growth to fit the SDK's hard 100-point limit, still FK-validated.
            count=100
        points=[];previous=list(start_joints);previous_pose=list(start_pose)
        angular_travel=0.
        yaw=start_pose[3]-start_pose[5]
        for index in range(1,count+1):
            point=[a+(b-a)*index/count for a,b in zip(start_pose[:3],target[:3])]
            yaw += math.degrees(math.remainder(math.atan2(point[1],point[0])-
                                               math.atan2(previous_pose[1],previous_pose[0]),2*math.pi))
            q,actual,yaw=self._solve(point,previous,yaw,solution,check)
            # Controller spline is joint-space. Check intermediate FK as well;
            # runtime feedback separately checks deviation from the Cartesian line.
            for fraction in (.25,.5,.75):
                middle=[a+(b-a)*fraction for a,b in zip(previous,q)]
                pose=self.forward(middle,check)
                if (line_distance(pose[:3],start_pose[:3],target[:3])>self.s.joint6_locked_path_tolerance_mm/2 or
                        abs(abs(math.remainder(pose[4],360.))-180.)>.1):
                    raise ValueError('fixed-J6 interpolated path exceeds Cartesian tolerance')
            angular_travel += orientation_error_deg(previous_pose[3:],actual[3:])
            points.append(q);previous=q;previous_pose=actual
        joint_travel=[sum(abs(b[i]-a[i]) for a,b in zip([start_joints]+points[:-1],points)) for i in range(6)]
        # Time-based movesj ignores velocity/acceleration arguments. Allocate a
        # conservative duration using Cartesian and total joint travel limits.
        limits = dict(minimum=.1,
                      linear_velocity=2*distance/self.s.linear_velocity_mm_s,
                      linear_acceleration=math.sqrt(12*distance/self.s.linear_acceleration_mm_s2),
                      angular_velocity=2*angular_travel/self.s.angular_velocity_deg_s,
                      angular_acceleration=math.sqrt(12*angular_travel/self.s.angular_acceleration_deg_s2),
                      joint_velocity=2*max(joint_travel)/self.s.joint_velocity_deg_s,
                      joint_acceleration=math.sqrt(12*max(joint_travel)/self.s.joint_acceleration_deg_s2))
        return WristPath(list(start_pose),list(start_joints),actual,points,max(limits.values()),limits)
