# fish_delivery — hand the fish to the child

The last step of the Stretch fishing game. After a fish is caught, this node
reaches the arm slowly across the mat so the child can take the fish off the
rod, waits, then retracts and resets so the game master can return to the
start state.

It is intentionally dumb: lift to mid-height, angle the gripper down a bit,
extend the telescoping arm out ~0.4 m **very slowly**, hold, retract. No
perception, no servoing.

The slow reach uses `move_to_pose(..., duration=extend_duration)`, so the
driver must be in **trajectory mode** (`mode:=trajectory`); that's the only
mode in which `move_to_pose` honors `duration`.

## Node: `fish_deliver` (node name `fish_deliverer`)

A persistent `hello_helpers` `HelloNode`, driven over topics — same pattern as
`fish_grab` in `Fish-Grab-Retry/`. The motion runs on a worker thread so the
executor stays free.

### Interface

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/deliver_fish/start` | `std_msgs/Empty` | in | Begin a delivery |
| `/deliver_fish/release` | `std_msgs/Empty` | in | Child has the fish — stop waiting and retract now |
| `/deliver_fish/stop` | `std_msgs/Empty` | in | Abort: retract + reset immediately |
| `/deliver_done` | `std_msgs/Bool` | out | Published when finished (`True` normal/early, `False` on abort) |

The game master publishes `/deliver_fish/start` once the fish is caught and
waits for `/deliver_done` before returning to the start state.

### The 30 s timer vs. the voice command

MVP just waits `wait_sec` (default 30 s) for the child to grab the fish. The
**voice command is already wired in**: publishing on `/deliver_fish/release`
ends the hold early and retracts. When voice lands, point it at that topic and
set `wait_sec` to a large value (or leave it as a safety timeout).

### Parameters (all setup-dependent — tune per session)

| Param | Default | Meaning |
|---|---|---|
| `lift_height` | `0.7` m | Arm lift for the hand-off |
| `wrist_pitch` | `-0.3` rad | Gripper tilt (negative = tip down a bit) |
| `wrist_yaw` | `0.0` rad | Point the wrist straight across the mat |
| `extend_distance` | `0.4` m | How far to reach across the mat |
| `extend_duration` | `20.0` s | Time for the slow reach (speed = distance / duration) |
| `wait_sec` | `30.0` s | How long to hold for the child to grab |

`0.4 m / 20 s = 0.02 m/s`, well under the driver's `0.2 m/s` cap on lift and
arm extension.

## Quick start (real robot)

```bash
ros2 launch stretch_core stretch_driver.launch.py   # T1
stretch_robot_home.py                               # T2 (one-time, for repeatable lift/arm)
ros2 launch fish_delivery fish_delivery.launch.py   # T3
ros2 topic echo /deliver_done                       # T4

# trigger a delivery
ros2 topic pub --once /deliver_fish/start std_msgs/msg/Empty {}

# end the hold early instead of waiting 30 s (the future voice hook)
ros2 topic pub --once /deliver_fish/release std_msgs/msg/Empty {}
```

Tune the reach at launch, e.g. slower and shorter:

```bash
ros2 run fish_delivery fish_deliver --ros-args \
    -p extend_distance:=0.3 -p extend_duration:=30.0 -p wait_sec:=45.0
```

> **Heads up:** lift/arm geometry is only reproducible after
> `stretch_robot_home.py`. Re-check `lift_height`, `wrist_pitch`, and
> `extend_distance` against your mat setup before running with a child.
