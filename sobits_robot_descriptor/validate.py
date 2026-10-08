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

"""Static checks for schema v2 descriptors and the ``validate`` CLI."""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections.abc import Mapping
from typing import Any

from .schema import INTERFACES, KINDS, SCHEMA_VERSION, DescriptorError

PLACEHOLDER = re.compile(r'<[^<>\s][^<>]*>')
_ROBOT_ID = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
_TOP_KEYS = {'schema_version', 'robot_id', 'version', 'namespace',
             'description_package', 'urdf', 'base_frame', 'odom_frame',
             'joint_states_topic', 'groups', 'mobile_base', 'sensors', 'ee',
             'excluded_joints'}
_REQUIRED = ('robot_id', 'namespace', 'urdf', 'base_frame', 'joint_states_topic')
_STREAM_KEYS = {'frame', 'raw_topic', 'compressed_topic', 'info_topic',
                'encoding', 'width', 'height', 'fps', 'points_topic', 'range_m'}
_ENTRY_KEYS = {
    'group': {'name', 'controller', 'interface', 'kind', 'joints',
              'uncommanded_joints', 'command_topic', 'command_action',
              'state_topic'},
    'base controller': {'name', 'controller', 'interface', 'kind', 'joints',
                        'command_topic'},
    'camera': {'name', 'frame', 'model', 'color', 'depth', 'requires'},
    'lidar': {'name', 'frame', 'scan_topic', 'points_topic', 'merged_from',
              'requires'},
    'imu': {'name', 'frame', 'topic', 'requires'},
    'ee': {'name', 'ee_link', 'reference_frame', 'control'},
}


def _placeholders(node: Any, where: str, out: list[str]) -> None:
    if isinstance(node, Mapping):
        for k, v in node.items():
            _placeholders(v, f'{where}.{k}' if where else str(k), out)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            _placeholders(v, f'{where}[{i}]', out)
    elif isinstance(node, str) and PLACEHOLDER.search(node):
        out.append(f'{where}: placeholder {node!r} not filled in')


def find_placeholders(data: Any) -> list[str]:
    out: list[str] = []
    _placeholders(data, '', out)
    return out


class _Checker:

    def __init__(self, data: Mapping[str, Any]):
        self.d = data
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def err(self, msg: str) -> None:
        self.errors.append(msg)

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)

    def topic(self, where: str, value: Any, required: bool = True) -> None:
        if value is None or value == '':
            if required:
                self.err(f'{where}: missing topic')
            return
        if not isinstance(value, str):
            self.err(f'{where}: topic must be a string, got {value!r}')
        elif value.startswith('/'):
            self.err(f'{where}: topic {value!r} must be relative to namespace '
                     '(drop the leading "/")')

    def str_key(self, entry: Mapping[str, Any], key: str, where: str) -> bool:
        v = entry.get(key)
        if not isinstance(v, str) or not v:
            self.err(f'{where}: missing required key {key!r}')
            return False
        return True

    def entries(self, value: Any, kind: str, where: str) -> list[Mapping[str, Any]]:
        """List of mappings with a unique ``name``; bad items are reported."""
        if value is None:
            return []
        if not isinstance(value, list):
            self.err(f'{where}: must be a list')
            return []
        out = []
        for i, e in enumerate(value):
            if not isinstance(e, Mapping):
                self.err(f'{where}[{i}]: must be a mapping')
                continue
            if not self.str_key(e, 'name', f'{where}[{i}]'):
                continue
            extra = set(e) - _ENTRY_KEYS[kind]
            if extra:
                self.warn(f'{kind} {e["name"]!r}: unknown key(s) {sorted(extra)}')
            out.append(e)
        return out

    def unique(self, names: list[str], kind: str) -> None:
        seen = set()
        for n in names:
            if n in seen:
                self.err(f'duplicate {kind} name {n!r}')
            seen.add(n)

    def enum(self, entry: Mapping[str, Any], key: str, allowed: tuple,
             where: str) -> None:
        if key in entry and entry[key] not in allowed:
            self.err(f'{where}: {key} must be one of {list(allowed)}, '
                     f'got {entry[key]!r}')

    def requires(self, entry: Mapping[str, Any], where: str,
                 urdf_args: Mapping[str, Any]) -> None:
        req = entry.get('requires')
        if req is None:
            return
        if not isinstance(req, Mapping):
            self.err(f'{where}: requires must be a mapping of xacro arg -> value')
            return
        for k, v in req.items():
            if isinstance(v, (Mapping, list)):
                self.err(f'{where}: requires.{k} must be a scalar')
            if k not in urdf_args:
                self.warn(f'{where}: requires key {k!r} is not in urdf.args '
                          '(entry is dropped unless the caller passes it)')

    def joints_list(self, value: Any, where: str, required: bool) -> list[str]:
        if value is None:
            if required:
                self.err(f'{where}: missing required key \'joints\'')
            return []
        if not isinstance(value, list) or not all(
                isinstance(j, str) and j for j in value):
            self.err(f'{where}: must be a list of joint names')
            return []
        if required and not value:
            self.err(f'{where}: joints must not be empty')
        return list(value)

    def stream(self, s: Any, where: str) -> None:
        if not isinstance(s, Mapping):
            self.err(f'{where}: must be a mapping')
            return
        extra = set(s) - _STREAM_KEYS
        if extra:
            self.warn(f'{where}: unknown key(s) {sorted(extra)}')
        if not s.get('frame'):
            self.err(f'{where}: missing frame (optical frame of the stream)')
        if not (s.get('raw_topic') or s.get('compressed_topic')):
            self.err(f'{where}: needs raw_topic or compressed_topic')
        for k in ('raw_topic', 'compressed_topic', 'info_topic', 'points_topic'):
            self.topic(f'{where}.{k}', s.get(k), required=False)
        rng = s.get('range_m')
        if rng is not None and not (
                isinstance(rng, list) and len(rng) == 2
                and all(isinstance(v, (int, float)) for v in rng)):
            self.err(f'{where}.range_m: must be [min, max] in metres')

    def run(self) -> None:
        d = self.d
        ph = find_placeholders(d)
        self.errors.extend(ph)

        ver = d.get('schema_version')
        if ver != SCHEMA_VERSION:
            self.err(f'schema_version must be {SCHEMA_VERSION}, got {ver!r}'
                     + (' (v1 descriptor: convert, see README)' if ver == 1 else ''))
            return
        for k in _REQUIRED:
            if k not in d or d[k] is None:
                self.err(f'missing required key {k!r}')
        extra = set(d) - _TOP_KEYS
        if extra:
            self.warn(f'unknown top-level key(s) {sorted(extra)}')
        rid = d.get('robot_id')
        if isinstance(rid, str) and not ph and not _ROBOT_ID.match(rid):
            self.err(f'robot_id {rid!r} must match {_ROBOT_ID.pattern}')
        if not isinstance(d.get('namespace', ''), str):
            self.err('namespace must be a string')
        elif str(d.get('namespace', '')).startswith('/'):
            self.err('namespace must not start with "/"')
        for k in ('base_frame', 'odom_frame'):
            if k in d and not (isinstance(d[k], str) and d[k]):
                self.err(f'{k} must be a non-empty link name')

        urdf = d.get('urdf')
        urdf_args: Mapping[str, Any] = {}
        if 'urdf' in d:
            if not isinstance(urdf, Mapping) or not urdf.get('xacro'):
                self.err('urdf: missing required key \'xacro\'')
            elif isinstance(urdf.get('args') or {}, Mapping):
                urdf_args = urdf.get('args') or {}
            else:
                self.err('urdf.args must be a mapping')
        self.topic('joint_states_topic', d.get('joint_states_topic'),
                   required='joint_states_topic' in d)

        owner: dict[str, str] = {}

        def claim(joints: list[str], who: str) -> None:
            for j in joints:
                if j in owner:
                    self.err(f'joint {j!r} listed in {owner[j]} and {who}'
                             if owner[j] != who
                             else f'joint {j!r} listed twice in {who}')
                else:
                    owner[j] = who

        groups = self.entries(d.get('groups'), 'group', 'groups')
        group_names = [g['name'] for g in groups]
        for g in groups:
            w = f'group {g["name"]!r}'
            self.str_key(g, 'controller', w)
            self.enum(g, 'interface', INTERFACES, w)
            self.enum(g, 'kind', KINDS, w)
            claim(self.joints_list(g.get('joints'), w, True), w)
            claim(self.joints_list(g.get('uncommanded_joints'),
                                   f'{w} uncommanded_joints', False),
                  f'{w} uncommanded_joints')
            for k in ('command_topic', 'command_action', 'state_topic'):
                self.topic(f'{w}.{k}', g.get(k), required=False)

        ctrl_names = list(group_names)
        mb = d.get('mobile_base')
        base_joints: list[str] = []
        if mb is not None:
            if not isinstance(mb, Mapping):
                self.err('mobile_base must be a mapping')
            else:
                for k in ('command_topic', 'odom_topic'):
                    self.topic(f'mobile_base.{k}', mb.get(k))
                for c in self.entries(mb.get('controllers'), 'base controller',
                                      'mobile_base.controllers'):
                    w = f'base controller {c["name"]!r}'
                    self.str_key(c, 'controller', w)
                    self.enum(c, 'interface', INTERFACES, w)
                    self.enum(c, 'kind', KINDS, w)
                    js = self.joints_list(c.get('joints'), w, True)
                    claim(js, w)
                    base_joints.extend(js)
                    self.topic(f'{w}.command_topic', c.get('command_topic'),
                               required=False)
                    ctrl_names.append(c['name'])
        self.unique(ctrl_names, 'group/base controller')

        sensors = d.get('sensors') or {}
        if not isinstance(sensors, Mapping):
            self.err('sensors must be a mapping')
            sensors = {}
        extra = set(sensors) - {'cameras', 'lidars', 'imus'}
        if extra:
            self.warn(f'sensors: unknown key(s) {sorted(extra)}')
        sensor_names: list[str] = []

        for c in self.entries(sensors.get('cameras'), 'camera', 'sensors.cameras'):
            w = f'camera {c["name"]!r}'
            sensor_names.append(c['name'])
            self.str_key(c, 'frame', w)
            if c.get('color') is None and c.get('depth') is None:
                self.err(f'{w}: needs a color or depth stream')
            for kind in ('color', 'depth'):
                if c.get(kind) is not None:
                    self.stream(c[kind], f'{w}.{kind}')
            self.requires(c, w, urdf_args)

        lidars = self.entries(sensors.get('lidars'), 'lidar', 'sensors.lidars')
        lidar_names = [s['name'] for s in lidars]
        for s in lidars:
            w = f'lidar {s["name"]!r}'
            sensor_names.append(s['name'])
            self.str_key(s, 'frame', w)
            self.topic(f'{w}.scan_topic', s.get('scan_topic'))
            self.topic(f'{w}.points_topic', s.get('points_topic'), required=False)
            for src in s.get('merged_from') or []:
                if src not in lidar_names:
                    self.err(f'{w}: merged_from {src!r} is not a lidar '
                             f'(known: {lidar_names})')
                elif src == s['name']:
                    self.err(f'{w}: merged_from must not reference itself')
            self.requires(s, w, urdf_args)

        for s in self.entries(sensors.get('imus'), 'imu', 'sensors.imus'):
            w = f'imu {s["name"]!r}'
            sensor_names.append(s['name'])
            self.str_key(s, 'frame', w)
            self.topic(f'{w}.topic', s.get('topic'))
            self.requires(s, w, urdf_args)
        self.unique(sensor_names, 'sensor')

        ees = self.entries(d.get('ee'), 'ee', 'ee')
        self.unique([e['name'] for e in ees], 'ee')
        for e in ees:
            w = f'ee {e["name"]!r}'
            self.str_key(e, 'ee_link', w)
            self.str_key(e, 'reference_frame', w)
            ctl = e.get('control')
            if ctl is None:
                continue
            if not isinstance(ctl, Mapping):
                self.err(f'{w}.control must be a mapping')
                continue
            for k in ('group', 'command_frame'):
                self.str_key(ctl, k, f'{w}.control')
            self.topic(f'{w}.control.enable_topic', ctl.get('enable_topic'))
            if ctl.get('group') and ctl['group'] not in group_names:
                self.err(f'{w}.control.group {ctl["group"]!r} is not a group '
                         f'(known: {group_names})')

        excl = self.joints_list(d.get('excluded_joints'), 'excluded_joints', False)
        for j in excl:
            if j in base_joints:
                self.warn(f'excluded_joints: {j!r} is a mobile base joint '
                          '(already excluded automatically)')
            elif j in owner:
                self.err(f'excluded_joints: {j!r} is listed in {owner[j]}')
        self.unique(excl, 'excluded_joints')


def check(data: Mapping[str, Any]) -> tuple[list[str], list[str]]:
    """Return ``(errors, warnings)`` for a parsed descriptor mapping."""
    if not isinstance(data, Mapping):
        return ['descriptor is not a YAML mapping'], []
    c = _Checker(data)
    c.run()
    return c.errors, c.warnings


def validate(source: Mapping[str, Any] | str | os.PathLike,
             strict: bool = False) -> list[str]:
    """Problems found in a mapping, file or robot id; warnings count if ``strict``."""
    if not isinstance(source, Mapping):
        from .loader import read_yaml, resolve_path
        source = read_yaml(resolve_path(source))
    errors, warnings = check(source)
    return errors + warnings if strict else errors


def _parse_args(pairs: list[str]) -> dict[str, str]:
    out = {}
    for p in pairs:
        k, sep, v = p.partition('=')
        if not sep or not k:
            raise SystemExit(f'--args expects key=value, got {p!r}')
        out[k] = v
    return out


def main(argv: list[str] | None = None) -> int:
    from .loader import parse, read_yaml, resolve_path
    ap = argparse.ArgumentParser(
        prog='validate', description='Check a <robot>.robot.yaml (schema v2).')
    ap.add_argument('robot', help='robot id or path to a .robot.yaml')
    ap.add_argument('--args', nargs='+', action='extend', default=[],
                    metavar='K=V', help='xacro arg overrides for requires')
    ap.add_argument('--strict', action='store_true',
                    help='treat warnings as errors')
    ns = ap.parse_args(argv)
    try:
        path = resolve_path(ns.robot)
        data = read_yaml(path)
    except DescriptorError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    errors, warnings = check(data)
    print(f'{path}')
    for w in warnings:
        print(f'  warning: {w}')
    for e in errors:
        print(f'  error: {e}')
    if errors or (ns.strict and warnings):
        print(f'FAILED: {len(errors)} error(s), {len(warnings)} warning(s)')
        return 1
    try:
        desc = parse(data, args=_parse_args(ns.args), path=path)
    except DescriptorError as e:
        print(f'  error: {e}')
        return 1
    print(f'OK: {desc.robot_id} v{desc.version} ns={desc.namespace!r} '
          f'args={desc.args}')
    print(f'  groups: {[g.name for g in desc.groups]}')
    print(f'  cameras: {[c.name for c in desc.cameras]} '
          f'lidars: {[s.name for s in desc.lidars]} '
          f'imus: {[s.name for s in desc.imus]}')
    print(f'  excluded joints: {desc.all_excluded_joints}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
