import os
from pathlib import Path
import runpy
import subprocess

from launch import LaunchContext
import pytest


@pytest.mark.parametrize('launch_name', ['sim.launch.py', 'sim_real.launch.py'])
def test_create_success_without_entity_fails_spawn(tmp_path, launch_name):
    for name, script in {
            'ros2': '#!/bin/sh\nexit 0\n',
            'gz': '#!/bin/sh\nexit 1\n',
            'sleep': '#!/bin/sh\nexit 0\n'}.items():
        executable = tmp_path / name
        executable.write_text(script)
        executable.chmod(0o755)

    launch_path = Path(__file__).parents[1] / 'launch' / launch_name
    spawn_action = runpy.run_path(str(launch_path))['_spawn_action']
    spec = {'name': 'missing_entity', 'x': '0', 'y': '0', 'z': '0'}
    action = spawn_action('/tmp/invalid.sdf', spec, attempts=1)
    context = LaunchContext()
    command = [
        ''.join(context.perform_substitution(sub) for sub in arg)
        for arg in action.cmd
    ]
    env = dict(os.environ)
    env['PATH'] = str(tmp_path) + os.pathsep + env['PATH']
    result = subprocess.run(command, env=env, capture_output=True, timeout=5)
    assert result.returncode == 1
