import os

from npu_sim_tools.validate_readme_commands import (
    _workspace_packages,
    validate_readme_commands,
)


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(PKG_DIR))
README_PATH = os.path.join(WORKSPACE_ROOT, 'README.md')
SRC_DIR = os.path.join(WORKSPACE_ROOT, 'src')


def test_workspace_packages_discovered():
    packages = _workspace_packages(SRC_DIR)
    for expected in ['npu_sim_bringup', 'npu_sim_description', 'npu_mobile_robot',
                     'npu_tracker', 'npu_apriltag', 'npu_sim_tools']:
        assert expected in packages


def test_readme_command_references_resolve():
    problems = validate_readme_commands(README_PATH, SRC_DIR)
    assert not problems, 'README command references failed validation:\n' + \
        '\n'.join(problems)


def test_validator_reports_missing_launch(tmp_path):
    readme = tmp_path / 'README.md'
    readme.write_text('```bash\nros2 launch npu_tracker nope.launch.py\n```\n')
    problems = validate_readme_commands(str(readme), SRC_DIR)
    assert any('nope.launch.py' in p for p in problems)


def test_validator_reports_unknown_package(tmp_path):
    readme = tmp_path / 'README.md'
    readme.write_text('ros2 run npu_nonexistent some_node\n')
    problems = validate_readme_commands(str(readme), SRC_DIR)
    assert any('npu_nonexistent' in p for p in problems)


def test_validator_reports_missing_executable(tmp_path):
    readme = tmp_path / 'README.md'
    readme.write_text('ros2 run npu_tracker nope_node\n')
    problems = validate_readme_commands(str(readme), SRC_DIR)
    assert any('nope_node' in p for p in problems)
