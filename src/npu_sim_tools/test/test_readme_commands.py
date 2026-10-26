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


def test_validator_reports_external_missing_launch(tmp_path, monkeypatch):
    pkg = 'external_pkg'
    share_dir = tmp_path / 'share' / pkg
    share_dir.mkdir(parents=True)
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_share_directory',
        lambda package: str(share_dir))
    readme = tmp_path / 'README.md'
    readme.write_text(f'ros2 launch {pkg} missing.launch.py\n')

    problems = validate_readme_commands(str(readme), tmp_path / 'src')

    assert any('missing.launch.py' in p for p in problems)


def test_validator_reports_external_missing_executable(tmp_path, monkeypatch):
    pkg = 'external_pkg'
    share_dir = tmp_path / 'share' / pkg
    share_dir.mkdir(parents=True)
    prefix = tmp_path / 'install'
    prefix.mkdir()
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_share_directory',
        lambda package: str(share_dir))
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_prefix',
        lambda package: str(prefix))
    readme = tmp_path / 'README.md'
    readme.write_text(f'ros2 run {pkg} missing_node\n')

    problems = validate_readme_commands(str(readme), tmp_path / 'src')

    assert any('missing_node' in p for p in problems)


def test_validator_empty_scripts_use_installed_executable(tmp_path, monkeypatch):
    pkg = 'workspace_pkg'
    executable_name = 'workspace_node'
    pkg_dir = tmp_path / 'src' / pkg
    pkg_dir.mkdir(parents=True)
    (pkg_dir / 'package.xml').write_text('<package/>\n')
    (pkg_dir / 'setup.py').write_text("entry_points={'console_scripts': []}\n")
    executable = tmp_path / 'install' / 'lib' / pkg / executable_name
    executable.parent.mkdir(parents=True)
    executable.write_text('#!/usr/bin/python3\n')
    executable.chmod(0o755)
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_prefix',
        lambda package: str(tmp_path / 'install'))
    readme = tmp_path / 'README.md'
    readme.write_text(f'ros2 run {pkg} {executable_name}\n')

    assert validate_readme_commands(str(readme), tmp_path / 'src') == []
    executable.unlink()
    problems = validate_readme_commands(str(readme), tmp_path / 'src')
    assert any(executable_name in p for p in problems)
