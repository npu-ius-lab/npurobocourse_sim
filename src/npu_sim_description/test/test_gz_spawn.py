import os
import shutil
import subprocess
import tempfile
import time

import pytest
import yaml


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(PKG_DIR, 'models')

SPAWN_WORLD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="spawn_test">
    <gravity>0 0 -9.8</gravity>
    <physics name="4ms" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <include><uri>model://robot</uri><pose>2.5 -1.5 0.03 0 0 0</pose></include>
    <include><uri>model://drone</uri><pose>-1.5 -4.0 0.32 0 0 0</pose></include>
    <include><uri>model://ring</uri><pose>-1 4.3 0.001 0 0 0</pose></include>
  </world>
</sdf>
"""

GZ = shutil.which('gz')


@pytest.mark.skipif(GZ is None, reason='gz (Gazebo Harmonic) executable not found')
def test_models_spawn_in_gazebo_harmonic():
    with tempfile.TemporaryDirectory() as tmp:
        resource_root = os.path.join(tmp, 'models')
        os.makedirs(resource_root)
        for name in ['ring', 'robot', 'marker', 'drone']:
            shutil.copytree(os.path.join(MODELS_DIR, name),
                            os.path.join(resource_root, name))

        # Render the drone template with the forward preset for the spawn test
        with open(os.path.join(PKG_DIR, 'config', 'camera_presets',
                               'forward.yaml')) as f:
            preset = yaml.safe_load(f)
        with open(os.path.join(resource_root, 'drone', 'model.sdf.template')) as f:
            template = f.read()
        xyz = ' '.join(f'{v:.10g}' for v in preset['translation'])
        rpy = ' '.join(f'{v:.10g}' for v in preset['rpy'])
        rendered = template.replace('{{ camera_xyz }}', xyz)
        rendered = rendered.replace('{{ camera_rpy }}', rpy)
        with open(os.path.join(resource_root, 'drone', 'model.sdf'), 'w') as f:
            f.write(rendered)

        world_path = os.path.join(tmp, 'spawn_world.sdf')
        with open(world_path, 'w') as f:
            f.write(SPAWN_WORLD)

        env = dict(os.environ)
        env['GZ_SIM_RESOURCE_PATH'] = resource_root + os.pathsep + \
            env.get('GZ_SIM_RESOURCE_PATH', '')
        env['GZ_PARTITION'] = 'npu_test_gz_spawn'
        env['ROS_DOMAIN_ID'] = '121'

        log_path = os.path.join(tmp, 'gz_server.log')
        with open(log_path, 'w') as log:
            server = subprocess.Popen(
                [GZ, 'sim', '-s', '-r', world_path],
                stdout=log, stderr=subprocess.STDOUT, env=env)
        try:
            deadline = time.time() + 120
            found = {}
            for name in ['npu_robot', 'npu_drone', 'ring']:
                while time.time() < deadline:
                    query = subprocess.run(
                        [GZ, 'model', '-m', name, '--pose'],
                        capture_output=True, text=True, env=env, timeout=30)
                    if query.returncode == 0 and 'Pose' in query.stdout:
                        found[name] = True
                        break
                    time.sleep(2)
            missing = {'npu_robot', 'npu_drone', 'ring'} - set(found)
            assert not missing, f'models not found in running sim: {missing}'
        finally:
            server.terminate()
            try:
                server.wait(timeout=30)
            except subprocess.TimeoutExpired:
                server.kill()

        with open(log_path) as f:
            errors = [line for line in f
                      if '[Err]' in line and 'gz_frame_id' not in line]
        assert not errors, f'gazebo reported errors:\n' + ''.join(errors[:10])
