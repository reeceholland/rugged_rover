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


def test_teleop_input_selection():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    description = load('teleop').generate_launch_description()
    declarations = {a.name: a for a in description.entities
                    if isinstance(a, DeclareLaunchArgument)}
    assert declarations['teleop_input'].choices == ['keyboard', 'joypad']
    for selection in ['keyboard', 'joypad']:
        context = LaunchContext()
        context.launch_configurations.update(teleop_input=selection, joy_dev='1')
        locations = []
        for action in description.entities:
            if not isinstance(action, IncludeLaunchDescription):
                continue
            if action.condition is not None and not action.condition.evaluate(context):
                continue
            action.launch_description_source.get_launch_description(context)
            location = action.launch_description_source.location
            locations.append(location)
            if location.endswith('/joy.launch.py'):
                arguments = dict(action.launch_arguments)
                assert arguments['cmd_vel_topic'] == '/cmd_vel'
                assert arguments['publish_stamped_twist'] == 'false'
            if location.endswith('/bringup.launch.py'):
                arguments = dict(action.launch_arguments)
                declarations['enable_motors'].execute(context)
                assert arguments['enable_motors'].perform(context) == 'false'
        assert any(p.endswith('/bringup.launch.py') for p in locations)
        assert any(p.endswith('/joy.launch.py') for p in locations) == (selection == 'joypad')
        assert not any(p.endswith('/keyboard_teleop.launch.py') for p in locations)


def test_lidar_variants_and_mount():
    import subprocess
    import xml.etree.ElementTree as ET
    xacro = LAUNCH.parents[1] / 'rugged_rover_robot_description/urdf/rugged_rover.urdf.xacro'
    for model, expected in [('rplidar', {'laser'}), ('ouster', {'os1_lidar'}), ('none', set())]:
        xml = subprocess.check_output([
            'xacro', str(xacro), 'lidar_model:=' + model,
            'ouster_x:=0.12', 'ouster_z:=0.3', 'ouster_yaw:=1.2',
        ], text=True)
        root = ET.fromstring(xml)
        links = {link.attrib['name'] for link in root.findall('link')}
        assert links & {'laser', 'os1_lidar'} == expected
        if model == 'ouster':
            joint = root.find("joint[@name='base_to_os1_lidar']")
            assert joint.find('parent').attrib['link'] == 'base_link'
            assert joint.find('origin').attrib['xyz'] == '0.12 0 0.3'
            assert joint.find('origin').attrib['rpy'] == '0 0 1.2'


def test_unity_navigation_defaults_and_opt_out():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    description = load('unity_sim').generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['use_slam'] == 'true'
    assert context.launch_configurations['use_nav2'] == 'true'
    found = set()
    for action in description.entities:
        if not isinstance(action, IncludeLaunchDescription):
            continue
        action.launch_description_source.get_launch_description(context)
        location = action.launch_description_source.location
        for option, filename in [('use_slam', 'slam.launch.py'), ('use_nav2', 'nav2.launch.py')]:
            if location.endswith('/' + filename):
                found.add(option)
                assert dict(action.launch_arguments)['use_sim_time'] == 'true'
                assert action.condition.evaluate(context)
                context.launch_configurations[option] = 'false'
                assert not action.condition.evaluate(context)
    assert found == {'use_slam', 'use_nav2'}


def test_nav2_rewrites_all_clock_parameters():
    module = load('nav2')
    rewritten = []
    def capture(*args, **kwargs):
        value = RewrittenYaml(*args, **kwargs)
        rewritten.append(value)
        return value
    with patch.object(module, 'RewrittenYaml', side_effect=capture):
        module.generate_launch_description()
    assert len(rewritten) == 1
    def clocks(value):
        result = []
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'use_sim_time':
                    result.append(child)
                else:
                    result.extend(clocks(child))
        return result
    for setting, expected in [('true', True), ('false', False)]:
        context = LaunchContext()
        context.launch_configurations['use_sim_time'] = setting
        path = Path(rewritten[0].perform(context))
        try:
            values = clocks(yaml.safe_load(path.read_text()))
            assert len(values) >= 10
            assert all(value is expected for value in values)
        finally:
            path.unlink()


def test_unity_odometry_faults_feed_ekf_without_competing_output():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    description = load('unity_sim').generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for action in description.entities:
        if not isinstance(action, IncludeLaunchDescription):
            continue
        action.launch_description_source.get_launch_description(context)
        if not action.launch_description_source.location.endswith('/ekf.launch.py'):
            continue
        args = dict(action.launch_arguments)
        assert args['output_odom_topic'].perform(context) == '/odom'
        assert args['odom_topic'].perform(context) == '/odom_raw'
        context.launch_configurations['use_odom_fault_injection'] = 'true'
        assert args['odom_topic'].perform(context) == '/wheel/odom_faulted'
        return
    raise AssertionError('EKF include missing')


def test_unity_motor_fault_injection_is_opt_in():
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import PythonExpression
    module = load('unity_sim')
    expressions = []
    def capture(*args, **kwargs):
        value = PythonExpression(*args, **kwargs)
        expressions.append(value)
        return value
    with patch.object(module, 'PythonExpression', side_effect=capture):
        description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['use_motor_fault_injection'] == 'false'
    motor = next(value for value in expressions if value.perform(context) == '/platform/motors/cmd')
    context.launch_configurations['use_motor_fault_injection'] = 'true'
    assert motor.perform(context) == '/platform/motors/cmd_raw'


def test_rplidar_condition_preserves_boolean_values():
    from launch.actions import IncludeLaunchDescription
    description = load('bringup').generate_launch_description()
    context = LaunchContext()
    lidar = None
    for action in description.entities:
        if isinstance(action, IncludeLaunchDescription):
            action.launch_description_source.get_launch_description(context)
            if action.launch_description_source.location.endswith('/rplidar_s2.launch.py'):
                lidar = action
    assert lidar is not None
    for model in ('rplidar', 'ouster', 'none'):
        for value in ('true', 'True', '1', 'false', 'False', '0'):
            context.launch_configurations.update(lidar_model=model, use_rplidar=value)
            assert lidar.condition.evaluate(context) == (model == 'rplidar' and value.lower() in ('true', '1'))
