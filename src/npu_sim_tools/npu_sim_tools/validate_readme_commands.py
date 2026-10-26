import os
import re
import sys


LAUNCH_RE = re.compile(r'ros2\s+launch\s+([\w]+)\s+([\w.\-]+\.launch\.py)')
RUN_RE = re.compile(r'ros2\s+run\s+([\w]+)\s+([\w.\-]+)')
CONSOLE_SCRIPT_RE = re.compile(r'\'([\w.\-]+)\s*=\s*[\w.]+\s*:\s*main\'')


def _workspace_packages(src_dir):
    packages = {}
    if not os.path.isdir(src_dir):
        return packages
    for name in sorted(os.listdir(src_dir)):
        pkg_dir = os.path.join(src_dir, name)
        if os.path.isfile(os.path.join(pkg_dir, 'package.xml')):
            packages[name] = pkg_dir
    return packages


def _console_scripts(pkg_dir):
    scripts = set()
    setup_py = os.path.join(pkg_dir, 'setup.py')
    if os.path.isfile(setup_py):
        with open(setup_py) as f:
            scripts.update(CONSOLE_SCRIPT_RE.findall(f.read()))
    return scripts


def _ament_share_directory(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        return None


def _has_installed_executable(pkg, executable):
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix(pkg)
    except Exception:
        return False
    candidate = os.path.join(prefix, 'lib', pkg, executable)
    return os.path.isfile(candidate) and os.access(candidate, os.X_OK)


def validate_readme_commands(readme_path, src_dir):
    """Check that every ros2 launch/run command referenced in the README
    resolves to an existing package, launch file, and executable.

    Returns a list of human-readable problems; empty means the README
    command references are consistent with the workspace."""
    with open(readme_path) as f:
        text = f.read()

    problems = []
    packages = _workspace_packages(src_dir)

    for match in LAUNCH_RE.finditer(text):
        pkg, launch_file = match.group(1), match.group(2)
        if pkg in packages:
            candidate = os.path.join(packages[pkg], 'launch', launch_file)
        else:
            share_dir = _ament_share_directory(pkg)
            if share_dir is None:
                problems.append(
                    f'ros2 launch: unknown package {pkg!r} ({launch_file})')
                continue
            candidate = os.path.join(share_dir, 'launch', launch_file)
        if not os.path.isfile(candidate):
            problems.append(
                f'ros2 launch: {pkg!r} has no launch file {launch_file!r}')

    for match in RUN_RE.finditer(text):
        pkg, executable = match.group(1), match.group(2)
        if pkg not in packages and _ament_share_directory(pkg) is None:
            problems.append(f'ros2 run: unknown package {pkg!r} ({executable})')
            continue
        if pkg in packages:
            scripts = _console_scripts(packages[pkg])
            if scripts:
                if executable not in scripts:
                    problems.append(
                        f'ros2 run: {pkg!r} has no executable {executable!r} '
                        f'(declared: {sorted(scripts)})')
                continue
        if not _has_installed_executable(pkg, executable):
            problems.append(
                f'ros2 run: {pkg!r} has no executable {executable!r}')

    return problems


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if len(argv) != 2:
        print('usage: validate_readme_commands <README.md> <workspace_src_dir>',
              file=sys.stderr)
        return 2
    readme_path, src_dir = argv
    problems = validate_readme_commands(readme_path, src_dir)
    if problems:
        for problem in problems:
            print(f'ERROR: {problem}', file=sys.stderr)
        return 1
    print(f'OK: all ros2 commands in {readme_path} resolve to existing '
          f'packages, launch files, and executables')
    return 0


if __name__ == '__main__':
    sys.exit(main())
