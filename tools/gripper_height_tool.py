#!/usr/bin/env python3
"""Manual gripper width / flange-Z calibration GUI and offline fit entry point."""
import argparse
from datetime import datetime
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('real','virtual','mock'),default='real')
    parser.add_argument('--ns',default='dsr01')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'calibrations/gripper_height'/datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    parser.add_argument('--resume',type=Path,help='중단한 measurements.csv로 재개')
    parser.add_argument('--fit',type=Path,help='GUI/ROS 없이 measurements.csv 피팅 및 리포트 생성')
    parser.add_argument('--velocity',type=float,default=120.,help='XY 직선 속도 mm/s (기본 120)')
    parser.add_argument('--acceleration',type=float,default=240.,help='XY 직선 가속도 mm/s² (기본 240)')
    parser.add_argument('--angular-velocity',type=float,default=60.,help='회전/홈 속도 °/s (기본 60)')
    parser.add_argument('--angular-acceleration',type=float,default=120.,help='회전/홈 가속도 °/s² (기본 120)')
    parser.add_argument('--z-velocity',type=float,default=10.,help='Z 미세 이동 속도 mm/s (기본 10)')
    parser.add_argument('--z-acceleration',type=float,default=20.,help='Z 미세 이동 가속도 mm/s² (기본 20)')
    args=parser.parse_args()
    from gripper_height.model import number
    for field in ('velocity','acceleration','angular_velocity','angular_acceleration','z_velocity','z_acceleration'):
        if number(getattr(args,field),field)<=0:parser.error('속도·가속도는 양수여야 합니다.')
    if args.fit:
        from gripper_height.model import MeasurementSession
        from gripper_height.report import export_fit
        fit=export_fit(MeasurementSession.load(args.fit))
        print(fit)
        print('Report:',args.fit.parent/'report.html')
        return
    if not args.resume and (args.output/'measurements.csv').exists():
        parser.error('이미 측정 CSV가 있습니다. --resume을 사용하거나 다른 --output을 지정하세요.')
    from gripper_height.app import CalibrationApp
    CalibrationApp(args).run()


if __name__=='__main__':main()
