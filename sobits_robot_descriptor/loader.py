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

"""Resolve and parse ``<robot>.robot.yaml`` into a RobotDescriptor."""

from __future__ import annotations

import os
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

import yaml

from .schema import (
    BaseControllerSpec, CameraSpec, DescriptorError, EEControlSpec, EESpec,
    GroupSpec, ImuSpec, LidarSpec, MobileBaseSpec, RobotDescriptor,
    StreamSpec, UrdfSpec, derived_topics,
)

ENV_PATH = 'SOBITS_ROBOT_DESCRIPTOR_PATH'


_TRUE = {'true', 'yes', 'on'}
_FALSE = {'false', 'no', 'off'}


def _str(value: Any) -> str:
    """Arg value as given; YAML booleans become 'true'/'false'."""
    return ('true' if value else 'false') if isinstance(value, bool) else str(value)


def _norm(value: Any) -> str:
    """Compare xacro args as lower-case strings; booleans and yes/no/on/off -> 'true'/'false'."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    s = str(value).strip().lower()
    return 'true' if s in _TRUE else 'false' if s in _FALSE else s


def _probe_dir(directory: Path, robot_id: str) -> list[Path]:
    name = f'{robot_id}.robot.yaml'
    return [directory / name, directory / robot_id / name]


def _probe_prefix(prefix: Path, robot_id: str) -> list[Path]:
    # Merged install (<prefix>/share/<pkg>) or colcon isolated (<prefix>/<pkg>/share/<pkg>).
    pkg = f'{robot_id}_description'
    rel = Path('share') / pkg / 'config' / f'{robot_id}.robot.yaml'
    return [prefix / rel, prefix / pkg / rel]


def _env_list(var: str) -> list[str]:
    return [p for p in os.environ.get(var, '').split(os.pathsep) if p]


def resolve_path(robot_id: str | os.PathLike,
                 search_dirs: Iterable[str | os.PathLike] | None = None) -> str:
    """
    Locate a descriptor file.

    Order: existing file, ``SOBITS_ROBOT_DESCRIPTOR_PATH``, ament index
    ``<id>_description``, AMENT/COLCON prefix scan, ``search_dirs``.
    """
    as_path = Path(robot_id).expanduser()
    if as_path.is_file():
        return str(as_path)
    rid = str(robot_id)
    tried: list[Path] = []

    def first(cands: list[Path]) -> str | None:
        for c in cands:
            tried.append(c)
            if c.is_file():
                return str(c)
        return None

    for entry in _env_list(ENV_PATH):
        p = Path(entry).expanduser()
        if p.is_file():
            tried.append(p)
            if p.name == f'{rid}.robot.yaml':
                return str(p)
        elif hit := first(_probe_dir(p, rid)):
            return hit

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(f'{rid}_description'))
        if hit := first([share / 'config' / f'{rid}.robot.yaml']):
            return hit
    except Exception:  # not installed, or no ament at all (plain venv)
        pass

    for var in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for entry in _env_list(var):
            if hit := first(_probe_prefix(Path(entry), rid)):
                return hit

    for d in search_dirs or []:
        if hit := first(_probe_dir(Path(d).expanduser(), rid)):
            return hit

    listing = '\n  '.join(str(t) for t in tried) or '(none)'
    raise DescriptorError(
        f'Robot descriptor {rid!r} not found. Set {ENV_PATH} or install '
        f'{rid}_description. Tried:\n  {listing}')


def read_yaml(path: str | os.PathLike) -> dict[str, Any]:
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise DescriptorError(f'Cannot read descriptor {path}: {e}') from e
    if not isinstance(data, dict):
        raise DescriptorError(f'Descriptor {path} is not a YAML mapping')
    return data


def _requires(entry: Mapping[str, Any]) -> dict[str, str]:
    return {str(k): _str(v) for k, v in (entry.get('requires') or {}).items()}


def _present(entry: Mapping[str, Any], args: Mapping[str, str]) -> bool:
    return all(k in args and _norm(args[k]) == _norm(v)
               for k, v in _requires(entry).items())


def _stream(kind: str, d: Mapping[str, Any] | None) -> StreamSpec | None:
    if d is None:
        return None
    rng = d.get('range_m')
    return StreamSpec(
        kind=kind, frame=d['frame'], raw_topic=d.get('raw_topic', ''),
        info_topic=d.get('info_topic', ''),
        compressed_topic=d.get('compressed_topic'),
        points_topic=d.get('points_topic'), encoding=d.get('encoding') or '',
        width=d.get('width'), height=d.get('height'), fps=d.get('fps'),
        range_m=[float(v) for v in rng] if rng is not None else None)


def _group(g: Mapping[str, Any]) -> GroupSpec:
    interface = g.get('interface', 'trajectory')
    topics = derived_topics(g['controller'], interface)
    for k in topics:
        if g.get(k):
            topics[k] = g[k]
    return GroupSpec(
        name=g['name'], controller=g['controller'], joints=list(g['joints']),
        interface=interface, kind=g.get('kind', 'position'),
        uncommanded_joints=list(g.get('uncommanded_joints') or []), **topics)


def _opt_float(value: Any) -> float | None:
    return None if value is None else float(value)


def _base_controller(c: Mapping[str, Any], base_cmd: str) -> BaseControllerSpec:
    interface = c.get('interface', 'group')
    # diff_drive consumes the base twist, so it defaults to mobile_base.command_topic.
    topic = c.get('command_topic') or (
        base_cmd if interface == 'diff_drive'
        else derived_topics(c['controller'], interface)['command_topic'])
    return BaseControllerSpec(
        name=c['name'], controller=c['controller'], joints=list(c['joints']),
        interface=interface, kind=c.get('kind', 'position'),
        command_topic=topic, wheel_radius=_opt_float(c.get('wheel_radius')),
        wheel_separation=_opt_float(c.get('wheel_separation')))


def parse(data: Mapping[str, Any], prefix: str | None = None,
          args: Mapping[str, Any] | None = None,
          path: str | None = None) -> RobotDescriptor:
    """Build a descriptor from a parsed mapping, applying ``requires``."""
    try:
        return _parse(data, prefix, args, path)
    except (KeyError, TypeError, ValueError) as e:
        if isinstance(e, DescriptorError):
            raise
        where = f' in {path}' if path else ''
        raise DescriptorError(
            f'Malformed descriptor{where}: {type(e).__name__}: {e}') from e


def _parse(data, prefix, args, path) -> RobotDescriptor:
    u = data['urdf']
    urdf_args = {str(k): _str(v) for k, v in (u.get('args') or {}).items()}
    eff = {**urdf_args, **{str(k): _str(v) for k, v in (args or {}).items()}}
    sensors = data.get('sensors') or {}

    cameras = [
        CameraSpec(name=c['name'], frame=c['frame'], model=c.get('model'),
                   color=_stream('color', c.get('color')),
                   depth=_stream('depth', c.get('depth')),
                   requires=_requires(c))
        for c in sensors.get('cameras') or [] if _present(c, eff)]
    lidars = [
        LidarSpec(name=s['name'], frame=s['frame'], scan_topic=s['scan_topic'],
                  points_topic=s.get('points_topic'),
                  merged_from=list(s.get('merged_from') or []),
                  requires=_requires(s))
        for s in sensors.get('lidars') or [] if _present(s, eff)]
    imus = [
        ImuSpec(name=s['name'], frame=s['frame'], topic=s['topic'],
                requires=_requires(s))
        for s in sensors.get('imus') or [] if _present(s, eff)]

    mb = data.get('mobile_base')
    mobile_base = None
    if mb:
        mobile_base = MobileBaseSpec(
            command_topic=mb['command_topic'], odom_topic=mb['odom_topic'],
            **{k: bool(mb.get(k, False)) for k in
               ('has_vel_x', 'has_vel_y', 'has_vel_z', 'has_vel_theta')},
            controllers=[_base_controller(c, mb['command_topic'])
                         for c in mb.get('controllers') or []])

    ee = []
    for e in data.get('ee') or []:
        ctl = e.get('control')
        ee.append(EESpec(
            name=e['name'], ee_link=e['ee_link'],
            reference_frame=e['reference_frame'],
            control=EEControlSpec(group=ctl['group'],
                                  command_frame=ctl['command_frame'],
                                  enable_topic=ctl['enable_topic'])
            if ctl else None))

    rid = data['robot_id']
    return RobotDescriptor(
        schema_version=int(data['schema_version']), robot_id=rid,
        version=str(data.get('version', '1.0.0')),
        namespace=data.get('namespace') or '',
        description_package=data.get('description_package') or f'{rid}_description',
        urdf=UrdfSpec(xacro=u['xacro'], urdf=u.get('urdf'), args=urdf_args),
        base_frame=data['base_frame'], odom_frame=data.get('odom_frame', 'odom'),
        joint_states_topic=data['joint_states_topic'],
        groups=[_group(g) for g in data.get('groups') or []],
        mobile_base=mobile_base, cameras=cameras, lidars=lidars, imus=imus,
        ee=ee, excluded_joints=list(data.get('excluded_joints') or []),
        prefix=prefix or '', args=eff, path=path)


def load_file(path: str | os.PathLike, prefix: str | None = None,
              args: Mapping[str, Any] | None = None,
              validate: bool = True) -> RobotDescriptor:
    """Parse one descriptor file; ``args`` override ``urdf.args`` for ``requires``."""
    data = read_yaml(path)
    if validate:
        from .validate import check
        errors, _ = check(data)
        if errors:
            raise DescriptorError(
                f'Invalid descriptor {path}:\n  ' + '\n  '.join(errors))
    return parse(data, prefix=prefix, args=args, path=str(path))


def load(id_or_path: str | os.PathLike, prefix: str | None = None,
         args: Mapping[str, Any] | None = None, validate: bool = True,
         search_dirs: Iterable[str | os.PathLike] | None = None) -> RobotDescriptor:
    """``resolve_path`` + ``load_file``."""
    return load_file(resolve_path(id_or_path, search_dirs), prefix=prefix,
                     args=args, validate=validate)
