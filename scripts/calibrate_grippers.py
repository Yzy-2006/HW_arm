#!/usr/bin/env python3
"""Passively capture a gripper endpoint from /gripper/fdb; never sends torque."""
import argparse
import datetime
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'teleop/VR_controller'))
from grippers import CONFIG_PATH, validate_side


def save_endpoint(path, side, endpoint, samples):
    if side not in ('left', 'right') or endpoint not in ('open','closed'):
        raise ValueError('Unknown side/endpoint')
    if len(samples) < 20:
        raise ValueError('Need at least 20 stable feedback samples')
    angles = [float(q) for q,dq in samples]
    speeds = [float(dq) for q,dq in samples]
    if not all(math.isfinite(v) for v in angles+speeds):
        raise ValueError('Feedback contains non-finite values')
    if max(angles)-min(angles) > 1.0 or max(map(abs,speeds)) > .05:
        raise ValueError('Gripper is moving; hold endpoint stable and retry')
    original = path.read_text()
    config = yaml.safe_load(original)
    selected = config[side]
    selected[endpoint+'_deg'] = statistics.median(angles)
    captures = selected.setdefault('captures', {})
    captures[endpoint] = {'time_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
                          'samples':len(samples), 'spread_deg':max(angles)-min(angles)}
    selected['calibrated'] = all(key in captures for key in ('open','closed'))
    validate_side(selected)  # Reject degenerate endpoints BEFORE touching files.
    backup = path.with_name(path.name + f'.{time.time_ns()}.backup')
    backup.write_text(original)
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
    try:
        with os.fdopen(fd,'w') as file:
            yaml.safe_dump(config,file,sort_keys=False,allow_unicode=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary,path)
    finally:
        if os.path.exists(temporary):os.unlink(temporary)
    return selected


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side',required=True,choices=['left','right'])
    parser.add_argument('--endpoint',required=True,choices=['open','closed'])
    parser.add_argument('--config',type=Path,default=CONFIG_PATH)
    parser.add_argument('--duration',type=float,default=1.0)
    parser.add_argument('--timeout',type=float,default=10.0)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not math.isfinite(args.timeout) or args.duration < .5 or args.timeout <= args.duration:
        parser.error('Use finite duration >= 0.5 and timeout > duration')
    import rclpy
    from std_msgs.msg import Float64MultiArray
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node=rclpy.create_node('towel_gripper_calibration')
    samples=[]; first=None; last=None
    def feedback(message):
        nonlocal first, last
        if len(message.data)!=8:return
        now=time.monotonic()
        if last is not None and now-last>.2:
            samples.clear(); first=None
        index=0 if args.side=='left' else 4
        samples.append((message.data[index],message.data[index+1]))
        first=now if first is None else first
        last=now
    node.create_subscription(Float64MultiArray,'/gripper/fdb',feedback,qos_profile_sensor_data)
    print(f'Capturing {args.side} {args.endpoint}. VR must be stopped; keep the gripper stationary at the chosen endpoint.')
    deadline=time.monotonic()+args.timeout
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if first is not None and last-first>=args.duration and len(samples)>=20:
                result=save_endpoint(args.config,args.side,args.endpoint,samples)
                print(f'Saved {result[args.endpoint+"_deg"]:.3f} deg; calibrated={result["calibrated"]}')
                return
        raise RuntimeError('Timed out waiting for a complete, fresh feedback window; config unchanged')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
