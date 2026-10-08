# Copyright (c) 2026, Team SOBITS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""``new_robot`` CLI: start a descriptor from ``config/template.robot.yaml``."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import yaml

from .validate import PLACEHOLDER, find_placeholders

TEMPLATE_NAME = 'template.robot.yaml'
PKG = 'sobits_robot_descriptor'


def template_path() -> Path:
    """Source checkout first, then the installed share directory."""
    cands = [Path(__file__).resolve().parents[1] / 'config' / TEMPLATE_NAME]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory(PKG)) / 'config' / TEMPLATE_NAME)
    except Exception:
        pass
    for entry in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if entry:
            cands.append(Path(entry) / 'share' / PKG / 'config' / TEMPLATE_NAME)
    for c in cands:
        if c.is_file():
            return c
    raise SystemExit(f'{TEMPLATE_NAME} not found; tried: {[str(c) for c in cands]}')


def render(robot_id: str, namespace: str | None = None,
           description_package: str | None = None,
           template: str | os.PathLike | None = None) -> str:
    text = Path(template or template_path()).read_text()
    ns = robot_id if namespace is None else namespace
    subs = {
        '<robot_id>': robot_id,
        '<namespace>': ns or '""',
        '<description_package>': description_package or f'{robot_id}_description',
    }
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog='new_robot', description='Create <robot_id>.robot.yaml from the template.')
    ap.add_argument('--robot-id', required=True)
    ap.add_argument('--namespace', help='topic namespace (default: robot id; "" for none)')
    ap.add_argument('--description-package', help='default: <robot_id>_description')
    ap.add_argument('-o', '--output',
                    help='output .yaml file or directory (default: ./<robot_id>.robot.yaml)')
    ap.add_argument('--force', action='store_true', help='overwrite an existing file')
    ns = ap.parse_args(argv)

    out = Path(ns.output or '.')
    if ns.output is None or out.is_dir() or not out.suffix:
        out = out / f'{ns.robot_id}.robot.yaml'
    if out.exists() and not ns.force:
        print(f'error: {out} exists (use --force)', file=sys.stderr)
        return 1
    text = render(ns.robot_id, ns.namespace, ns.description_package)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    pkg = ns.description_package or f'{ns.robot_id}_description'
    print(f'wrote {out}')

    left = find_placeholders(yaml.safe_load(text))
    keys = sorted({PLACEHOLDER.search(m).group(0) for m in left})
    if left:
        print(f'\n{len(left)} placeholder(s) left to fill in ({len(keys)} distinct):')
        for m in left:
            print(f'  {m}')
    print(f'\nPut it in {pkg}/config/ and install it from {pkg}/CMakeLists.txt:')
    print('  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})')
    print('\nThen check it with:')
    print(f'  ros2 run {PKG} validate {out}')
    print('and add a URDF consistency test like '
          'sobit_home_description/test/test_robot_descriptor.py '
          f'({pkg}/test/test_robot_descriptor.py).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
