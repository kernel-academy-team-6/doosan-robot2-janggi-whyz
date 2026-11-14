"""One backend factory shared by the ROS action server and realtime UI."""
from .robot_client import MockRobot, RobotClient, VirtualRobotClient
from .gripper import GripperClient, MockGripper, VirtualGripperClient


def create_clients(node, prefix, settings, parameters, backend):
    if backend == 'mock':
        return MockRobot(settings), MockGripper()
    if backend not in ('virtual', 'real'):
        raise ValueError('backend must be mock, virtual, or real')
    robot_type = VirtualRobotClient if backend == 'virtual' else RobotClient
    robot = robot_type(node, prefix, settings, parameters.get('expected_flange_tcp_name', ''),
                       parameters['service_timeout_sec'], parameters['motion_timeout_sec'])
    if backend == 'virtual':
        # Joint targets feed the Gazebo force controllers. This never moves or
        # attaches a piece; contact and gravity still determine its motion.
        gripper = VirtualGripperClient(node, prefix, parameters['virtual_gripper_delay_sec'])
        if parameters.get('virtual_physics_feedback', False):
            from .simulation_feedback import SimulationFeedback
            feedback = SimulationFeedback(node, prefix, parameters['motion_timeout_sec'])
            robot.physics_feedback = gripper.physics_feedback = feedback
    else:
        gripper = GripperClient(node, prefix, parameters)
    return robot, gripper
