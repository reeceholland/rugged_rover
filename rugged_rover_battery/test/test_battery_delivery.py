# Copyright 2026 Reece Holland
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32


def test_best_effort_voltage_and_stale_stop():
    rclpy.init()
    node = rclpy.create_node('battery_delivery_test')
    pub = node.create_publisher(Float32, '/battery/voltage', qos_profile_sensor_data)
    states = []
    sub = node.create_subscription(
        Bool, '/platform/battery/is_critical', lambda msg: states.append(msg.data), 10)
    executable = Path(get_package_prefix('rugged_rover_battery')) / (
        'lib/rugged_rover_battery/battery_voltage_monitor')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'stale_timeout:=0.3'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def pump(seconds, voltage=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if voltage is not None:
                pub.publish(Float32(data=voltage))
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        pump(1.2)
        assert states and states[-1], 'No battery reading must inhibit motion'
        pump(1.0, 12.0)
        assert states[-1] is False, 'Best-effort firmware voltage must be received'
        pump(1.0)
        assert states[-1], 'Stale voltage must publish critical motor inhibit'
        pump(0.5, 12.0)
        assert states[-1] is False
        pump(0.5, 10.0)
        assert states[-1], 'Low battery must inhibit motion'
        assert process.poll() is None
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
