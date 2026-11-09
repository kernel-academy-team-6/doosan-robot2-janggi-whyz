"""CLI/Python example action client; targets must be supplied explicitly."""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from janggi_pick_place2.action import Home, Pick, Place, PickAndPlace


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operation', choices=['home', 'pick', 'place', 'pick_and_place'], default='pick_and_place')
    parser.add_argument('--pick-target', nargs=4, type=float, metavar=('X', 'Y', 'TIP_Z', 'YAW'))
    parser.add_argument('--place-target', nargs=4, type=float, metavar=('X', 'Y', 'TIP_Z', 'YAW'))
    parser.add_argument('--width', type=float, default=-1., help='object width mm; omitted uses server default')
    parser.add_argument('--allowance', type=float, default=-1., help='total extra opening mm')
    parser.add_argument('--current', type=float, default=-1., help='compatibility input only; position_cmd uses existing current settings')
    parser.add_argument('--server', default='/janggi_pick_place2')
    parser.add_argument('--timeout', type=float, default=180.)
    options = parser.parse_args(args)
    if options.operation in ('pick', 'pick_and_place') and options.pick_target is None:
        parser.error('--pick-target is required for pick and pick_and_place')
    if options.operation in ('place', 'pick_and_place') and options.place_target is None:
        parser.error('--place-target is required for place and pick_and_place')
    if options.timeout <= 0:
        parser.error('--timeout must be positive')
    action = {'home': Home, 'pick': Pick, 'place': Place,
              'pick_and_place': PickAndPlace}[options.operation]
    request = action.Goal()
    if options.operation == 'pick_and_place':
        request.pick_target, request.place_target = options.pick_target, options.place_target
    elif options.operation in ('pick', 'place'):
        request.target = options.pick_target if options.operation == 'pick' else options.place_target
    if options.operation in ('pick', 'pick_and_place'):
        request.object_width_mm = options.width
        request.opening_allowance_mm = options.allowance
        request.grip_current_ma = options.current
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('janggi_pick_place2_example')
    client = ActionClient(node, action, options.server.rstrip('/')+'/'+options.operation)
    handle = None
    exit_code = 1
    try:
        if not client.wait_for_server(timeout_sec=10.):
            raise RuntimeError('action server not found; launch pick_place.launch.py first')
        def feedback(message):
            print(f'{message.feedback.stage}: {message.feedback.progress:.0%}', flush=True)
        sending = client.send_goal_async(request, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, sending, timeout_sec=10.)
        if not sending.done():
            raise TimeoutError('goal acknowledgement timed out; inspect server before retrying')
        handle = sending.result()
        if not handle.accepted:
            raise RuntimeError('goal rejected; see server log or /janggi_pick_place2/status')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=options.timeout)
        if not result.done():
            raise TimeoutError('action timeout')
        response = result.result().result
        print(response, flush=True)
        exit_code = 0 if response.success else 1
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Action client stopped: {exc}', file=sys.stderr, flush=True)
        if handle is not None and handle.accepted and rclpy.ok():
            cancelling = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancelling, timeout_sec=5.)
            print('Cancellation requested; inspect final server state before another goal.', file=sys.stderr)
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
