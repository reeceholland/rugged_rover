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

"""Watchdogs must share transition state across launch event contexts."""
import importlib.util
from pathlib import Path
from launch import LaunchContext


def module():
    path = Path(__file__).resolve().parents[1] / 'launch/slam.launch.py'
    spec = importlib.util.spec_from_file_location('slam_lifecycle', path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def test_watchdogs_observe_state_across_contexts():
    launch = module()
    state = {'configured': False, 'active': False}
    assert launch._configuration_timeout(LaunchContext(), state)
    launch._mark_state(LaunchContext(), state, 'configured')
    assert launch._configuration_timeout(LaunchContext(), state) == []
    assert launch._activation_timeout(LaunchContext(), state)
    launch._mark_state(LaunchContext(), state, 'active')
    assert launch._activation_timeout(LaunchContext(), state) == []
