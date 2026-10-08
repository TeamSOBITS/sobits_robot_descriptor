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

"""Schema v2 dataclasses for ``<robot>.robot.yaml``."""

from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass, replace
from typing import Any

SCHEMA_VERSION = 2
INTERFACES = ('trajectory', 'group')
KINDS = ('position', 'velocity')


class DescriptorError(ValueError):
    """Invalid, unresolvable or inconsistent robot descriptor."""


def derived_topics(controller: str, interface: str) -> dict[str, str | None]:
    """Default command/action/state topics for a controller base name."""
    if interface == 'group':
        return {'command_topic': f'{controller}/commands',
                'command_action': None, 'state_topic': None}
    return {'command_topic': f'{controller}/joint_trajectory',
            'command_action': f'{controller}/follow_joint_trajectory',
            'state_topic': f'{controller}/controller_state'}


def _unknown(kind: str, name: str, known: list[str]) -> DescriptorError:
    return DescriptorError(f'Unknown {kind} {name!r}. Known: {sorted(known)}')


@dataclass
class StreamSpec:
    kind: str                       # 'color' | 'depth'
    frame: str                      # optical frame
    raw_topic: str = ''
    info_topic: str = ''
    compressed_topic: str | None = None
    points_topic: str | None = None
    encoding: str = ''
    width: int | None = None
    height: int | None = None
    fps: float | None = None
    range_m: list[float] | None = None


@dataclass
class CameraSpec:
    name: str
    frame: str                      # mount link
    model: str | None = None
    color: StreamSpec | None = None
    depth: StreamSpec | None = None
    requires: dict[str, str] = field(default_factory=dict)

    def streams(self) -> list[StreamSpec]:
        """Present streams, color first."""
        return [s for s in (self.color, self.depth) if s is not None]


@dataclass
class LidarSpec:
    name: str
    frame: str
    scan_topic: str
    points_topic: str | None = None
    merged_from: list[str] = field(default_factory=list)
    requires: dict[str, str] = field(default_factory=dict)


@dataclass
class ImuSpec:
    name: str
    frame: str
    topic: str
    requires: dict[str, str] = field(default_factory=dict)


@dataclass
class GroupSpec:
    name: str
    controller: str
    joints: list[str]
    interface: str = 'trajectory'
    kind: str = 'position'
    uncommanded_joints: list[str] = field(default_factory=list)
    command_topic: str | None = None
    command_action: str | None = None
    state_topic: str | None = None


@dataclass
class BaseControllerSpec:
    name: str
    controller: str
    joints: list[str]
    interface: str = 'group'
    kind: str = 'position'
    command_topic: str | None = None


@dataclass
class MobileBaseSpec:
    command_topic: str
    odom_topic: str
    has_vel_x: bool = False
    has_vel_y: bool = False
    has_vel_z: bool = False
    has_vel_theta: bool = False
    controllers: list[BaseControllerSpec] = field(default_factory=list)


@dataclass
class EEControlSpec:
    group: str
    command_frame: str
    enable_topic: str


@dataclass
class EESpec:
    name: str
    ee_link: str
    reference_frame: str
    control: EEControlSpec | None = None


@dataclass
class UrdfSpec:
    xacro: str
    urdf: str | None = None
    args: dict[str, str] = field(default_factory=dict)


# to_dict keys holding relative topics / frames, rewritten when resolving.
_TOPIC_KEYS = {'raw_topic', 'info_topic', 'compressed_topic', 'points_topic',
               'scan_topic', 'topic', 'command_topic', 'command_action',
               'state_topic', 'odom_topic', 'enable_topic', 'joint_states_topic'}
_FRAME_KEYS = {'frame', 'ee_link', 'reference_frame', 'command_frame',
               'base_frame', 'odom_frame'}


@dataclass
class RobotDescriptor:
    robot_id: str
    namespace: str
    urdf: UrdfSpec
    base_frame: str
    joint_states_topic: str
    odom_frame: str = 'odom'
    version: str = '1.0.0'
    description_package: str = ''
    groups: list[GroupSpec] = field(default_factory=list)
    mobile_base: MobileBaseSpec | None = None
    cameras: list[CameraSpec] = field(default_factory=list)
    lidars: list[LidarSpec] = field(default_factory=list)
    imus: list[ImuSpec] = field(default_factory=list)
    ee: list[EESpec] = field(default_factory=list)
    excluded_joints: list[str] = field(default_factory=list)
    schema_version: int = SCHEMA_VERSION
    prefix: str = ''
    args: dict[str, str] = field(default_factory=dict)  # effective xacro args
    path: str | None = None

    def topic(self, rel: str) -> str:
        """Absolute topic under ``namespace``; absolute input is returned as is."""
        if rel.startswith('/'):
            return rel
        ns = self.namespace.strip('/')
        return f'/{ns}/{rel}' if ns else f'/{rel}'

    def frame(self, name: str) -> str:
        return f'{self.prefix}{name}'

    def group(self, name: str) -> GroupSpec:
        return self._by_name(self.groups, 'group', name)

    def camera(self, name: str) -> CameraSpec:
        return self._by_name(self.cameras, 'camera', name)

    def lidar(self, name: str) -> LidarSpec:
        return self._by_name(self.lidars, 'lidar', name)

    def imu(self, name: str) -> ImuSpec:
        return self._by_name(self.imus, 'imu', name)

    def end_effector(self, name: str) -> EESpec:
        return self._by_name(self.ee, 'end effector', name)

    @staticmethod
    def _by_name(items: list[Any], kind: str, name: str) -> Any:
        for item in items:
            if item.name == name:
                return item
        raise _unknown(kind, name, [i.name for i in items])

    def joints(self, include_uncommanded: bool = False) -> list[str]:
        """Group joints in descriptor order (dataset feature order)."""
        out: list[str] = []
        for g in self.groups:
            out.extend(g.joints)
            if include_uncommanded:
                out.extend(g.uncommanded_joints)
        return out

    @property
    def all_excluded_joints(self) -> list[str]:
        out = list(self.excluded_joints)
        if self.mobile_base:
            for c in self.mobile_base.controllers:
                out.extend(j for j in c.joints if j not in out)
        return out

    def controllers(self) -> list[GroupSpec | BaseControllerSpec]:
        """Arm/head/... groups followed by mobile base controllers."""
        base = self.mobile_base.controllers if self.mobile_base else []
        return [*self.groups, *base]

    def filtered(
        self,
        exclude_groups: list[str] | None = None,
        exclude_cameras: list[str] | None = None,
        exclude_ee: list[str] | None = None,
        exclude_joints: list[str] | None = None,
    ) -> RobotDescriptor:
        """
        Return a copy with the named components removed.

        Joints of excluded groups and excluded joints fold into
        ``excluded_joints`` so consumers still filter them out of joint_states;
        a group left without joints is dropped. Unknown names raise
        DescriptorError so a typo fails loudly.
        """
        ex_g = list(exclude_groups or [])
        ex_c = list(exclude_cameras or [])
        ex_e = list(exclude_ee or [])
        ex_j = list(exclude_joints or [])
        if not (ex_g or ex_c or ex_e or ex_j):
            return self

        known_j = self.joints(include_uncommanded=True)
        for names, known, kind in (
            (ex_g, [g.name for g in self.groups], 'group'),
            (ex_c, [c.name for c in self.cameras], 'camera'),
            (ex_e, [e.name for e in self.ee], 'ee'),
            (ex_j, known_j, 'joint'),
        ):
            unknown = [n for n in names if n not in known]
            if unknown:
                raise DescriptorError(
                    f'Unknown {kind} name(s) in exclude list: {unknown}. '
                    f'Available: {sorted(known)}')

        removed: list[str] = []
        groups: list[GroupSpec] = []
        for g in self.groups:
            if g.name in ex_g:
                removed.extend(g.joints + g.uncommanded_joints)
                continue
            kept = [j for j in g.joints if j not in ex_j]
            kept_u = [j for j in g.uncommanded_joints if j not in ex_j]
            removed.extend(j for j in g.joints + g.uncommanded_joints
                           if j in ex_j)
            if not kept:
                removed.extend(kept_u)
                continue
            if kept != g.joints or kept_u != g.uncommanded_joints:
                g = replace(g, joints=kept, uncommanded_joints=kept_u)
            groups.append(g)

        ee = [e for e in self.ee if e.name not in ex_e]
        # Zero groups only makes sense when an EE control spec drives the arms.
        if not groups and not any(e.control for e in ee):
            raise DescriptorError(
                'exclude list would remove every joint group; at least one '
                'must remain (or an ee entry with control must survive).')

        excluded = list(self.excluded_joints)
        excluded.extend(j for j in removed if j not in excluded)
        return replace(
            self, groups=groups, ee=ee, excluded_joints=excluded,
            cameras=[c for c in self.cameras if c.name not in ex_c])

    def to_dict(self, absolute_topics: bool = True) -> dict[str, Any]:
        """
        Plain dict in schema layout with derived topics filled in.

        Frames carry ``prefix``; ``path``/``prefix``/``args`` are omitted.
        """
        def conv(obj: Any, key: str | None = None) -> Any:
            if is_dataclass(obj):
                return {f.name: conv(getattr(obj, f.name), f.name)
                        for f in fields(obj)}
            if isinstance(obj, list):
                return [conv(v, key) for v in obj]
            if isinstance(obj, dict):
                return {k: conv(v) for k, v in obj.items()}
            if isinstance(obj, str):
                if absolute_topics and key in _TOPIC_KEYS:
                    return self.topic(obj)
                if key in _FRAME_KEYS:
                    return self.frame(obj)
            return obj

        out = conv(self)
        for k in ('path', 'prefix', 'args'):
            out.pop(k)
        sensors = {k: out.pop(k) for k in ('cameras', 'lidars', 'imus')}
        out['sensors'] = sensors
        return out
