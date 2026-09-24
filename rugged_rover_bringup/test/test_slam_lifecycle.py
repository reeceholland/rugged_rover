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
