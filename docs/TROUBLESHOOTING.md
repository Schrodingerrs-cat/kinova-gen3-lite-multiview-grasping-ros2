# Running this repo on another laptop: setup and troubleshooting

Written after porting the pipeline to a second laptop (Ubuntu 22.04, ROS 2 Humble, CycloneDDS, Kinova Gen3 Lite, RealSense D405).
The code ran unchanged on the original machine. Everything below was measured on the second laptop, and the numbers are given so
you can check whether your machine behaves the same. The measurements are in section 6.

The pipeline itself is not at fault in any of the failures below, except the gripper contact check (section 3.7), which was
extended. The other failures come from the host: network settings, USB link speed and a Python package layout.

## 1. Checklist: the four things that must be right before `./run_ros.sh`

| # | Requirement | How to check | If it fails |
|---|---|---|---|
| 1 | Every ROS terminal has the same DDS settings (`tools/ros_env.sh`) | `echo $ROS_LOCALHOST_ONLY` prints `1` in each terminal | 3.1, 3.2 |
| 2 | The D405 is linked at USB 3.x | `rs-enumerate-devices \| grep "Usb Type"` shows 3.x | 3.3 |
| 3 | The arm driver is running, not only `start_arm.sh` | `ros2 control list_controllers` shows `joint_trajectory_controller` active | 3.5 |
| 4 | The gripper is open | `.venv/bin/python tools/open_gripper.py` prints `0.000` | 3.8 |

## 2. Symptom index

| What you see | Section |
|---|---|
| `view N/10 ...: skipped, camera error: camera stalled: no frames for 4 s (3 tries)` | 3.1 |
| Repeating `No controller active in LOW_LEVEL_SERVOING mode !` in the driver terminal | 3.2 |
| `Failed to find a free participant index for domain`, `rcl node's rmw handle is invalid` | 3.2 |
| Camera node says `connected using a 2.1 port`, 10 FPS, `Frames didn't arrived within 5 seconds` | 3.3 |
| Camera works from `ros2 topic hz` but a script gets nothing (large images) | 3.4 |
| RViz shows the arm in the wrong pose, or an orange arm that does not move | 3.5 |
| `AttributeError: '_NamespacePath' object has no attribute 'insert'` in `perceive.py` | 3.6 |
| `gripper closed to 1.00, contact=False` then `closed on nothing - not lifting` although the object was held | 3.7 |
| `close-range check: only N table points around the object` | 3.8 |
| `close-range object position disagrees with the far-range one` on a thin cable | 3.9 |

## 3. Failures, causes and fixes

### 3.1 `camera stalled: no frames for 4 s (3 tries)`

Symptom: during the scan every view prints `skipped, camera error`, or only the first view after a camera restart works.
`ros2 topic hz` on the same topic shows 25 to 30 Hz, and `lsusb` shows the camera, so it looks like a software bug.

Cause: with `ROS_LOCALHOST_ONLY` unset, CycloneDDS switches a topic to multicast as soon as it has two or more subscribers in
different processes. On the test laptop it multicasts out of the WiFi card, so a 1.2 MB, 30 Hz color image goes over the radio.
The pipeline always has several subscribers on the camera (the keepalive, the recorder in `run_pick.py`, and the grabber), so
every grab stalls. Restarting the camera node briefly resets the connections, which is why exactly one grab works afterwards.

Fix: keep DDS on the loopback. Add both lines to `~/.bashrc` (or `source tools/ros_env.sh` in every terminal):

    export ROS_LOCALHOST_ONLY=1
    export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

Then close every ROS terminal (RViz included), run `ros2 daemon stop`, and start the terminals again. A participant that has
`ROS_LOCALHOST_ONLY=1` cannot talk to one that does not (measured: 0.0 Hz in both directions, 30.0 Hz when both agree), so all terminals must agree. Both lines are needed (see 3.2).

Cost: ROS traffic no longer leaves the machine. The Kortex driver talks to the arm over its own connection, so the arm is
unaffected.

Verify: with the camera running, two subscribers on the color topic should each get about 30 Hz. Before the fix they got 19.4 Hz
each and aligned depth fell to 0 Hz.

### 3.2 `Failed to find a free participant index` and the repeating LOW_LEVEL_SERVOING message

Symptom: after setting `ROS_LOCALHOST_ONLY=1`, the driver terminal prints `No controller active in LOW_LEVEL_SERVOING mode !`
about 1000 times a second, and `ros2 control list_controllers` lacks `joint_trajectory_controller` and the gripper controller.
The driver log shows two spawners dying with `Failed to find a free participant index for domain 42` and
`rcl node's rmw handle is invalid`.

Cause: with multicast off, CycloneDDS finds peers by probing participant-index slots, and allows only about 10 per domain by
default. The Kortex launch starts five spawner processes at once next to MoveIt, RViz and the driver, so some get no slot and die.
The message is the hardware layer saying that no controller is commanding the arm. The arm is never at risk.

Fix: the `CYCLONEDDS_URI` line above raises the limit to 120. Start the driver again.

Verify: 25 programs started at once, 9 succeeded without the line and 25 with it.

### 3.3 D405 on USB 2.1

Symptom: the camera node warns `Device ... is connected using a 2.1 port. Reduced performance is expected`, takes about 35 s to
open its streams, runs at 10 FPS instead of 30 and logs `Frames didn't arrived within 5 seconds`.

Cause: the link negotiated USB 2. On the test setup the camera linked at 2.1 through its own cable and at 3.2 after an extension cable was
added and the port was changed (the two were not isolated). A loose port or a charging-only USB-C cable also gives 2.1.

Fix: use a USB 3 data cable (an extension cable made it USB 3.2 here) and a USB 3 port. Check with
`rs-enumerate-devices | grep "Usb Type"`, which should say 3.2 or 3.x. After the fix: streams open in about 2 s and run at 29.7 Hz.

Notes:
- Restarting the camera node makes the camera reset itself. `journalctl -k` then shows a `USB disconnect` followed by a reconnect.
  That is usually the node, not a cable fault.
- USB autosuspend can drop the camera mid-run. If it happens, a udev rule that sets `power/control` to `on` for vendor 8086,
  product 0b5b prevents it. It was not needed during the test runs.

### 3.4 Large images arrive on `ros2 topic hz` but not in a script

Symptom: a node waiting forever for an image while `ros2 topic hz` shows 25 to 30 Hz.

Cause: `robot_ros.py`, `common.py` and `keepalive.py` subscribe with `qos_profile_sensor_data`, which is BEST_EFFORT.
With the default Linux receive buffer (`net.core.rmem_max` of 212992 bytes) a 1.2 MB image fragmented into hundreds of datagrams
overflows the socket buffer and is dropped with no retransmission. `ros2 topic hz` and `echo` default to RELIABLE, which hides it.
Measured: 0 frames in 8 s with BEST_EFFORT against 132 with RELIABLE, same topic, same moment.

Fix:

    sudo sysctl -w net.core.rmem_max=16777216 net.core.rmem_default=8388608
    echo -e "net.core.rmem_max=16777216\nnet.core.rmem_default=8388608" | sudo tee /etc/sysctl.d/60-ros2.conf

With `ROS_LOCALHOST_ONLY=1` traffic stays on the loopback, but keep the larger buffers.

### 3.5 RViz shows the arm in the wrong pose

Cause 1: only `./start_arm.sh` was started. It starts MoveIt and RViz but not the hardware driver, so `/joint_states` has no
publisher and RViz draws the default pose. Start terminal 2 (`kortex_bringup gen3_lite.launch.py`).

Cause 2: the orange arm and the large ring and arrow marker are MoveIt's query goal state, not the robot. Untick
`Planning Request` then `Query Goal State` to hide them. Do not press `Plan` or `Plan & Execute` in that window, because it would
send the real arm to the orange pose. The pipeline never uses those buttons.

Verify: `ros2 topic info /joint_states` shows `Publisher count: 1`.

### 3.6 `AttributeError: '_NamespacePath' object has no attribute 'insert'`

Cause: `perceive.py` patched `mpl_toolkits.__path__` for a machine where another copy of `mpl_toolkits` shadows the virtualenv's.
With a newer Matplotlib in a venv, `mpl_toolkits` is already correct and its path is a namespace path without `insert`.

Fix (in this branch): the line is wrapped in `if hasattr(mpl_toolkits.__path__, 'insert'):`.

### 3.7 `closed on nothing - not lifting` while the object is actually held

Symptom: a hard, thin or small object (a toy banana, 17 to 36 mm at the grasp) is gripped, but the log prints
`gripper closed to 1.00, contact=False` and the object is skipped.

What the gripper reports: the Gen3 Lite gripper is underactuated. The motor drives only the upper (proximal) link of each finger,
and the lower (distal) segment is passive and flexes around the object. The driver exposes one joint,
`right_finger_bottom_joint`, with these state channels:

| channel | value |
|---|---|
| position | real, about 990 Hz |
| velocity | always exactly 0.0 (never populated) |
| effort | nan |

So the only signal is position, and it cannot see the passive tips. For small or thin objects held at the tips, the driven links
close almost fully and the reading is the same as for an empty gripper.

Why the original check fails: `contact = stalled or pos < GRIP_EMPTY (0.975)`. The stall test needs the measured position to lag
the command by more than 0.05, which cannot happen when contact is at 0.95 or above because the command tops out at 1.0. The
post-lift HOLDING check compares two readings near 1.00 and passes for held and empty alike.

Measured readings (planned object width, then reading after the close): 72 mm 0.52, 64 mm 0.74, 63 mm 0.76, 55 mm 0.84,
53 mm 0.87 and 0.82, 48 mm 0.83, toy banana at 36 mm or less 1.00.

Fix (in this branch): after a lift, ask the wrist camera. The camera rides on the gripper, so a held object stays about 0.11 m from
the lens between the fingertips while the table in the lower image corners moves away. An empty hand sees the table at 0.25 m or
more. `grasp_exec.py` has `object_in_hand(cam)`:

    held = (median depth in rows 400-480, cols 340-510 of the aligned depth) < 0.20 m
           and (median depth of the two lower corner boxes) - (that depth) > 0.08 m

It is a rescue only. It runs when the old finger-based check would have aborted (`contact=False`). In that case the arm lifts, the
camera decides, and:
- held: the pick continues as normal (put back, release);
- not held: the arm lowers to the grasp pose, opens, retreats and skips the object, exactly as before.

Runs where the finger check already said contact take the original path unchanged. Unreadable depth counts as not held, so a
camera glitch can only cause a skip. It is switched by `execution.vision_grasp_check` in `config.yaml` (`true` here, default off in
code). Thresholds are `vision_tip_max_m` (0.20) and `vision_margin_m` (0.08).

The check has to run after a lift. At the grasp pose the object (0.11 m) and the table (0.14 m) are too close to separate.

Verified on the robot: the toy banana, closed to 1.00 with `contact=False`, then after the lift
`depth between the fingertips 0.109 m, table 0.265 m -> HOLDING` and a normal put back, in two separate runs. A three-object run
took the normal path for the first two objects (readings 0.87 and 0.77) and the rescue for the banana.

Not claimed: a small drop of the reading from 0.9997 to 0.989 after the lift was seen once with the banana. It was not checked
against an empty-gripper lift, so it is not used.

### 3.8 `close-range check: only N table points around the object`

Symptom: the arm reaches the pre-grasp pose, prints this, skips the object and returns to the scan pose.

Observed: it happened once, in a run that started with the gripper closed. The closed fingertips show up at the bottom centre of
every scan image (open fingertips are in the bottom corners). After starting the next runs with the gripper open it did not recur
in two runs and four picks. The mechanism is not proven: the executor opens the gripper before the pre-grasp pose, and the scan
clouds filter the fingertips out as single-view points. Treat it as a habit that removes a variable.

Fix: before each run open the gripper: `.venv/bin/python tools/open_gripper.py` (fingers only, never moves the arm).

If it recurs with the gripper open, the executor now prints a `diagnostic:` line before aborting: total close-range points, points
in the table ring, how many are within 20 mm of the far-range plane, and the median height of the ring points above that plane.
A large median height means the close-range table and the merged far-range plane disagree by more than the 20 mm gate.

### 3.9 Thin cables are skipped

`perception.min_height` (12 mm) is used for both the merged far-range cloud and the single close-range cloud. A power cable about
8 to 10 mm thick disappears in the close-range cloud; only the taller plug survives. The grasp point is then far from the planned
one and the check aborts before descending, which is the correct behaviour. In the far-range merge, view-to-view table offsets of
20 to 38 mm smear thin objects into a taller blob, so the cable is detected there but not confirmed up close. This is a limit of
the method, not a bug. Graspable connector housings are taller and work.

## 4. Starting a run on a new laptop

1. `source tools/ros_env.sh` in every terminal, or put its lines in `~/.bashrc`.
2. Terminal 1 `./start_arm.sh`, terminal 2 the driver, terminal 3 the camera (see `README.md`).
3. `rs-enumerate-devices | grep "Usb Type"` shows 3.x.
4. `python3 ros2_iface/smoke_test.py` prints the joints, a camera frame and `IK service up: True`.
5. `.venv/bin/python tools/open_gripper.py`.
6. `./run_ros.sh --single` for the first run. Without `--single` it picks every object with no countdown.

## 5. Reference numbers from healthy runs

Scan: 10 of 10 views saved with 80,000 to 98,000 points each.
Per-view table tilt 0.9 to 6.6 degrees and offset between -4 and +39 mm against the centre view; ICP shifts 1 to 16 mm, rotations
under 2.6 degrees, trimmed RMS 2.8 to 4.8 mm. The merged table plane is tilted about 4.5 degrees although the table is flat; that
is leftover hand-eye error, and the close-range refinement is what makes the grasps land.
Execution: every commanded move ends within 0.7 to 1.6 mm of its target; a full run with one object takes about 5 minutes, and each
further object adds 2 to 3 minutes.

## 6. Measurements behind the claims

| Test | Result |
|---|---|
| One color subscriber, localhost-only | 29.9 Hz, 37 MB/s on the loopback |
| Two color subscribers, multicast on | 19.4 Hz each, 25 MB/s out of the WiFi card, aligned depth and point cloud 0 Hz |
| Synthetic 1.2 MB 30 Hz publisher, 2 to 3 subscribers | 19.6 Hz each without the fix, 30.0 Hz each with `ROS_LOCALHOST_ONLY=1` and 0 MB on WiFi |
| Publisher and subscriber with different `ROS_LOCALHOST_ONLY` settings | 0.0 Hz in both directions; 30.0 Hz when both use the same setting |
| 25 programs started at once | 9 started without the participant limit line, 25 with it |
| D405 on USB 2.1 | 35 s to open, 10 FPS, frame timeouts |
| D405 on USB 3.2 | 2 s to open, 29.7 Hz color, aligned depth and point cloud |
| Pipeline camera load (keepalive, recorder, grabber) after the fixes | 6 of 6 on-demand grabs succeeded in 0.2 s each; before, 4 of 4 stalled for 12 s |
| Finger velocity | exactly 0.00000 in about 15,000 samples per close, while the finger moved 0 to 0.81 rad |
| Empty gripper closed | reading 0.9997 to 0.9999, spread 0.0001 |
| Camera depth, empty hand | 0.38 to 0.41 m in the lower image; anything in front of the gripper reads 0.06 to 0.18 m |
| Banana lift, wrist camera | between the fingertips 0.109 to 0.111 m throughout a lift where the table went from 0.14 m to 0.40 m |
| Vision rule on that recording | HELD for the whole carried period, not held after the drop, 0 false positives in 319 empty-hand frames |

## 7. Diagnostic tools (`tools/`)

| Tool | What it does | Moves the arm? |
|---|---|---|
| `open_gripper.py` | opens the gripper and prints the reading | no, fingers only |
| `gripper_probe.py <label>` | closes the gripper once while logging finger position, with a camera depth snapshot before and after | no, fingers only |
| `flight_recorder.py` | passive: records finger position (about 990 Hz), arm joints and depth in five image regions during a normal run; Ctrl-C saves `tools/results/flight_*.npz` | no, only listens |
| `analyze_flight.py [file]` | prints, around every gripper close, a per-second table of finger position, depth between the tips, and arm motion | no |

Typical use for a grasp that behaves oddly: start `flight_recorder.py`, run `./run_ros.sh --single`, press Ctrl-C in the recorder,
then run `analyze_flight.py`.

Useful one-liners:

    ros2 control list_controllers                        # driver and controllers
    ros2 topic info /joint_states                        # expect Publisher count: 1
    rs-enumerate-devices | grep "Usb Type"               # expect 3.x
    journalctl -k --since "-10min" | grep -i usb         # camera or arm USB drops (needs the adm group)
    tr '\0' '\n' < /proc/<pid>/environ | grep -E 'ROS_LOCALHOST_ONLY|CYCLONEDDS_URI'   # settings of a running node

Do not use `pgrep -f` or `pkill -f` with a pattern that also appears in your own command line: it matches and kills your shell.
Use `ps -eo pid=,args=` and kill by PID.

## 8. What this branch changes in the code

| File | Change |
|---|---|
| `perceive.py` | guard around the `mpl_toolkits.__path__.insert` line (3.6) |
| `grasp_exec.py` | wrist-camera rescue `object_in_hand()` for grasps the finger position calls empty (3.7); a `diagnostic:` line before the close-range abort (3.8) |
| `config.yaml` | one line, `execution.vision_grasp_check: true`; set it to `false` for the original behaviour |
| `tools/` | `ros_env.sh`, `open_gripper.py`, `gripper_probe.py`, `flight_recorder.py`, `analyze_flight.py` |
| `docs/TROUBLESHOOTING.md`, `README.md`, `CLAUDE.md` | this document and pointers to it |

Software safety (`safety.enabled: false` in `config.yaml`) was left as in the original: no software bounds or joint-step caps are
active, so keep the e-stop in reach.
