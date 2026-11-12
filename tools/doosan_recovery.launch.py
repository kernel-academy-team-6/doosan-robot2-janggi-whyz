"""Standalone DRFL recovery request, independent of ros2_control bringup."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def recovery_command(host, enter_recovery, force_access='false', entry_method='control'):
    script = Path(__file__).resolve().with_name('doosan_recovery.sh')
    command = ['bash', str(script), '--host', host]
    if enter_recovery == 'true':
        command.append('--enter-recovery')
    elif enter_recovery != 'false':
        raise ValueError('enter_recovery must be true or false')
    if force_access == 'true':
        if enter_recovery != 'true':
            raise ValueError('force_access requires enter_recovery:=true')
        command.append('--force-access')
    elif force_access != 'false':
        raise ValueError('force_access must be true or false')
    if entry_method not in ('control', 'safety-enter'):
        raise ValueError('entry_method must be control or safety-enter')
    if entry_method == 'safety-enter':
        if enter_recovery != 'true':
            raise ValueError('safety-enter requires enter_recovery:=true')
        command.extend(['--entry-method', entry_method])
    return command


def start_recovery(context):
    command = recovery_command(
        LaunchConfiguration('host').perform(context),
        LaunchConfiguration('enter_recovery').perform(context),
        LaunchConfiguration('force_access').perform(context),
        LaunchConfiguration('entry_method').perform(context),
    )
    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', description='Explicit robot controller IP'),
        DeclareLaunchArgument(
            'enter_recovery', default_value='false', choices=['false', 'true'],
            description='false: read state only; true: request recovery from SAFE_STOP2/SAFE_OFF2',
        ),
        DeclareLaunchArgument(
            'force_access', default_value='false', choices=['false', 'true'],
            description='Explicitly request forced control-authority transfer; requires enter_recovery=true',
        ),
        DeclareLaunchArgument(
            'entry_method', default_value='control', choices=['control', 'safety-enter'],
            description='control: set_robot_control; safety-enter: set_safety_mode(RECOVERY, ENTER) only',
        ),
        OpaqueFunction(function=start_recovery),
    ])
