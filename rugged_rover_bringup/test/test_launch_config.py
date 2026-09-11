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

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext
from nav2_common.launch import RewrittenYaml
import yaml

LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_exactly_one_odom_tf_owner():
    module = load('bringup')
    rewritten = []

    def capture(*args, **kwargs):
        result = RewrittenYaml(*args, **kwargs)
        rewritten.append(result)
        return result

    with patch.object(module, 'RewrittenYaml', side_effect=capture):
        module.generate_launch_description()
    assert len(rewritten) == 1
    for ekf, expected in [('true', False), ('false', True)]:
        context = LaunchContext()
        context.launch_configurations['use_ekf'] = ekf
        path = Path(rewritten[0].perform(context))
        try:
            config = yaml.safe_load(path.read_text())
            assert config['diff_drive_controller']['ros__parameters']['enable_odom_tf'] is expected
        finally:
            path.unlink()


def test_teleop_includes_keyboard():
    from launch.actions import IncludeLaunchDescription
    description = load('teleop').generate_launch_description()
    context = LaunchContext()
    locations = []
    for action in description.entities:
        if isinstance(action, IncludeLaunchDescription):
            action.launch_description_source.get_launch_description(context)
            locations.append(action.launch_description_source.location)
    assert any('keyboard_teleop.launch.py' in location for location in locations)
