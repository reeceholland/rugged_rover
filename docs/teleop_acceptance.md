# Reliable teleop acceptance

Status: software checks implemented; physical acceptance is pending.

WSL verification on 2026-09-10: all seven changed packages built. The 19 focused
test cases passed (11 hardware-interface, 5 mode-switch, 1 battery delivery/stale
test, 2 launch configuration tests). C++ formatting and checked copyright headers
passed. Existing Python quote-style lint issues and the hardware-interface API
deprecation warning remain. The manager was built against an extracted Ubuntu
libgpiod-dev/libgpiod2t64 copy under /tmp because system installation needs sudo;
install the normal dependency on the Pi. The new CI workflow has not run remotely.

## Build and start on the Pi

Use the rover's checked-out branch containing these changes. Install dependencies
with rosdep (including libgpiod-dev), rebuild, then restart the manager. Do not run
a second manager alongside the service.

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
sudo systemctl restart rover-manager.service
journalctl -u rover-manager.service -f
```

The GPIO24 switch is active-low in software. The physical switch may be low or already high when the manager starts. If it is already
high, the manager treats it as a pending single-toggle request and starts teleop
after the configured double-toggle window expires. A falling edge always stops.
Two rising edges within two seconds still request autonomous mode. For this
milestone, use only single toggles.

Keyboard teleop is not launched inside the manager because it needs a focused
terminal for stdin. Lidar should only be running while the switch-selected teleop
or autonomous launch is active. Start keyboard teleop separately after the manager
enters teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use a small command first, with the rover raised or in a clear area.

## Stop contract

- Real-hardware URDF requires the manager enable heartbeat by default.
- Manager enables only after fresh battery and platform-debug telemetry arrives.
  Startup timeout is 15 seconds; telemetry timeout is 3 seconds.
- Hardware commands are zero unless enabled, battery state is noncritical and
  fresh (2 seconds), and wheel feedback is fresh (250 ms).
- Missing manager heartbeat inhibits output after 500 ms using a monotonic clock.
- Controller velocity-command timeout remains 250 ms.
- Teensy firmware independently stops on missing motor commands after 500 ms
  and retains its existing battery cutoff. No firmware change is required.
- Manager disables before waiting for child-process shutdown.
- Standalone real bringup now remains motor-inhibited without the manager.
  Unity explicitly opts out of the manager heartbeat requirement.
- Direct publication to /platform/motors/cmd bypasses the host hardware-interface
  gate. Do not use the README's direct motor test as a managed stopping test.

These are software timeout settings, not measured physical braking times.
Allow for the control cycle, transport latency, firmware loop and mechanical coast.

## Physical acceptance (record results)

Start with wheels raised and an accessible motor-power disconnect. Record
/rover/state, /rover/motors_enabled, /platform/motors/cmd,
/platform/motors/feedback, /battery/voltage and /diff_drive_controller/cmd_vel
with rosbag, plus video or a stopwatch for actual wheel stopping.

| Test | Required result | Actual result |
| --- | --- | --- |
| Boot with switch low | Manager remains idle and motors stay disabled | Pending |
| Boot or restart with switch already high | Teleop starts after the double-toggle window; motors enable only after fresh telemetry | Pending |
| Brief high then low within two seconds | Remains stopped after window expires | Pending |
| Start keyboard teleop and press a small forward command | `/cmd_vel` reaches `/diff_drive_controller/cmd_vel`, motor commands become nonzero, and motion is controlled | Pending |
| Switch low while moving | Disable precedes shutdown; commands, wheels, and lidar stop | Pending |
| Stop keyboard teleop while commanding motion | Command stream stops or becomes zero; wheels stop | Pending |
| Stop/kill manager while moving | Heartbeat loss inhibits commands within 500 ms plus a control cycle | Pending |
| Disconnect Pi-Teensy link while moving | Existing firmware watchdog stops wheels; measure latency | Pending |
| Lose feedback while command source stays active | Hardware command becomes zero after 250 ms plus a control cycle | Pending |
| Battery telemetry stops | Battery monitor publishes critical; manager faults; wheels stop | Pending |
| Restart stack after fault with switch left high | Teleop restarts after the double-toggle window once telemetry is healthy | Pending |
| Repeat ten start/stop cycles | No orphan processes, duplicate controller managers or unintended motion | Pending |
| EKF off, then on in separate runs | Exactly one odom-to-base_link TF publisher in each run | Pending |

Inject low battery in an isolated software test, not onto a live shared rover
topic. The automated battery test covers low voltage and missing voltage without
draining the physical battery.

After all raised-wheel checks pass, repeat enable release, switch-off, joystick
disconnect and manager-loss checks at low speed in a clear area. Record stopping
distance and use it to set operational clearance. This milestone is complete only
when those physical results are recorded and acceptable.

## Automated regression checks

```bash
colcon test --packages-select rugged_rover_manager rugged_rover_hardware_interfaces rugged_rover_battery rugged_rover_bringup --ctest-args -R 'test_mode_switch|test_sabertooth_interface|battery_delivery|launch_config'
colcon test-result --verbose
```

The ROS messaging tests force localhost-only discovery and separate domain IDs
178 and 179. They publish synthetic commands only in those isolated test domains.
Old lint results can still appear in colcon test-result from prior runs.
