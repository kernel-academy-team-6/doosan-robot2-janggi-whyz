#!/usr/bin/env python3
"""Probe model-register reads only. Run directly; no package rebuild needed.

Requires exclusive flange serial access. Does not command torque, current,
position, or robot motion. Opening/closing the port affects other serial users.
"""
import argparse
import json
import time

import rclpy
from dsr_msgs2.srv import (
    FlangeSerialOpen, FlangeSerialWrite, FlangeSerialRead, FlangeSerialClose,
)


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='dsr01')
    parser.add_argument('--port', type=int, choices=(1, 2), default=1)
    parser.add_argument('--baudrate', type=int, default=57600)
    parser.add_argument('--slave', type=int, choices=range(1, 248), default=1)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('gripper_read_probe')
    prefix = '/' + args.namespace.strip('/') + '/gripper/flange_serial_'
    opened = False

    def call(kind, suffix, **fields):
        client = node.create_client(kind, prefix + suffix)
        start = time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('service unavailable: ' + prefix + suffix)
            future = client.call_async(kind.Request(**fields))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
            if not future.done():
                future.cancel()
                raise TimeoutError('service timeout: ' + prefix + suffix)
            response = future.result()
            record = {'operation': suffix, 'elapsed_sec': round(time.monotonic()-start, 3),
                      'success': bool(response.success)}
            if suffix == 'read':
                record.update(size=response.size, data=list(response.data))
            print(json.dumps(record), flush=True)
            if not response.success:
                raise RuntimeError('service returned success=False: ' + suffix)
            return response
        finally:
            node.destroy_client(client)

    try:
        call(FlangeSerialOpen, 'open', port=args.port, baudrate=args.baudrate,
             bytesize=8, parity=0, stopbits=1)
        opened = True
        time.sleep(0.3)  # 시리얼 초기화 대기: 첫 write 전에 포트 안정화.
        body = bytes([args.slave, 3, 0, 0, 0, 1])
        request = list(body + crc16(body).to_bytes(2, 'little'))
        print('Model-register read only. No motion commands. Request: ' + str(request), flush=True)
        for delay in (0., .25, 1.):
            print(json.dumps({'trial_delay_sec': delay}), flush=True)
            call(FlangeSerialWrite, 'write', port=args.port, data=request)
            time.sleep(delay)
            packet = bytearray()
            for _ in range(2):
                response = call(FlangeSerialRead, 'read', port=args.port, timeout=1.)
                if not 0 <= response.size <= len(response.data):
                    raise RuntimeError('invalid response size')
                packet.extend(response.data[:response.size])
                if len(packet) >= 3:
                    expected = 5 if packet[1] & 0x80 else packet[2] + 5
                    if len(packet) >= expected:
                        break
            valid = (len(packet) == 7 and packet[:3] == bytes([args.slave, 3, 2])
                     and crc16(packet[:-2]) == int.from_bytes(packet[-2:], 'little'))
            print(json.dumps({'trial_delay_sec': delay, 'packet': list(packet),
                              'valid_model_response': valid,
                              'model_number': int.from_bytes(packet[3:5], 'big') if valid else None}),
                  flush=True)
            time.sleep(.25)
    finally:
        try:
            if opened:
                call(FlangeSerialClose, 'close', port=args.port)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
