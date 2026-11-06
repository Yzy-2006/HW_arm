# Single-Motor Internal Zero

This sets one motor's current physical position to its internal zero. It does not
move the mechanism to a reference pose. The command uses the project's existing
ENCOS `MotorZeroSetting` implementation; verify the permitted motor state and
protocol against the installed motor/firmware. Power-cycle persistence is not
established by this tool and no undocumented save command is issued.

Stop other servo/EtherCAT masters and control programs. Support the mechanism and
place the selected joint at the confirmed mechanical/model zero before starting.
The tool does not disable motors, hold other joints or cancel old control commands.

## Command

From the project root, for the left shoulder roll only:

```bash
make zero-motor SIDE=left JOINT=2
```

`SIDE` must be `left` or `right`; `JOINT` must be one integer from 1 through 7.
There is no default target, `both` side, list or all-motors operation.

| JOINT | Name |
| --- | --- |
| 1 | shoulder_pitch |
| 2 | shoulder_roll |
| 3 | shoulder_yaw |
| 4 | elbow |
| 5 | wrist_roll |
| 6 | wrist_pitch |
| 7 | wrist_yaw |

Left uses CAN channel 0 and right uses channel 1. The motor ID equals JOINT; the
actual PDO slot is resolved from `can_config.yaml`, with exactly one match required.
The equivalent ROS entry for the example is:

```bash
make servo ARGS="--ros-args -p motor_zero_joint:=left_shoulder_roll_joint"
```

This mode cannot be combined with feedback-only, calibration or CAN scan modes.
It exits after the operation and never enters homing or normal motion control.

## Verification

1. Query only the selected motor's raw position. Require at least 10 responses over
   0.2 seconds with total angle spread at most 0.2 degrees and zero reported error.
2. Send one host-side zero request (`0x7FF`, four bytes: ID high, ID low, `00 03`)
   in only the selected channel/slot. All other slots are empty, not zero-current
   commands. The host output buffer is immediately cleared to prevent reuse.
3. Require an ACK from that slot with matching motor ID and payload `IDhi IDlo 01 03`.
4. Query raw position again, requiring at least five readings within 0.5 degrees of
   zero over at least 0.1 seconds. Each waiting phase times out after two seconds.

Wrong IDs, invalid WKC, malformed replies and non-finite positions are rejected.
Success requires both the ACK and raw readback. If interrupted or timed out after
issuing the request, the zero may already have changed: inspect raw feedback before
retrying. The tool never automatically retries a zero command. Gateway repetition
of cached identical replies without sequence numbers cannot be ruled out.

## Software Coordinates

The existing `motorPositionOffsets`, `zeropos.yaml`, model, presets and datasets
are not edited by the zeroing tool. The left shoulder roll software offset is
currently 0 degrees (the temporary -180 degree correction has been disabled).
After hardware zeroing, review the coordinate configuration before restarting any
controller; internal-zero and software-offset corrections can otherwise accumulate.
Readback verification uses raw motor degrees, not the software-corrected joint angle.
Use `make read-motors` to inspect both, and verify persistence using the vendor's
power-cycle procedure before motion testing.

## Build

The servo binary must be rebuilt. Relinking removes its network capabilities;
restore them before use:

```bash
sudo setcap cap_net_raw,cap_net_admin+ep "$(readlink -f ros2_ws/install/servo_control_hw/lib/servo_control_hw/servo_node)"
```

Only offline packet, state-transition and command-selection tests have been run.
No motor was zeroed as part of implementation.
