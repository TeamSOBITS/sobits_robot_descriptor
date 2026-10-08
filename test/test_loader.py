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

import subprocess
import sys

import pytest
import yaml
from conftest import DATA, ROOT

from sobits_robot_descriptor.loader import parse
from sobits_robot_descriptor import (
    DescriptorError, SCHEMA_VERSION, load, load_file, resolve_path,
)

WHEELS = [f'wheel_{k}_{c}_joint' for k in ('steer', 'drive')
          for c in ('f_l', 'f_r', 'b_l', 'b_r')]


@pytest.fixture
def desc(sobit_home_path):
    return load_file(sobit_home_path)


def test_import_needs_no_ros():
    code = ('import sys, sobits_robot_descriptor as m; '
            'bad = [k for k in sys.modules if k.startswith(("rclpy", "ament_index"))]; '
            'assert not bad, bad; print(m.SCHEMA_VERSION)')
    out = subprocess.run([sys.executable, '-c', code], cwd=ROOT, check=True,
                         capture_output=True, text=True).stdout
    assert out.strip() == str(SCHEMA_VERSION)


def test_resolve_explicit_path(clean_env, sobit_home_path):
    assert resolve_path(sobit_home_path) == str(sobit_home_path)
    assert load(str(sobit_home_path)).robot_id == 'sobit_home'


def test_resolve_env_dir_and_file(clean_env, tmp_path, sobit_home_path):
    clean_env.setenv('SOBITS_ROBOT_DESCRIPTOR_PATH', f'{tmp_path}:{DATA}')
    assert resolve_path('sobit_home') == str(DATA / 'sobit_home.robot.yaml')
    nested = tmp_path / 'sobit_home'
    nested.mkdir()
    (nested / 'sobit_home.robot.yaml').write_text(sobit_home_path.read_text())
    assert resolve_path('sobit_home') == str(nested / 'sobit_home.robot.yaml')
    clean_env.setenv('SOBITS_ROBOT_DESCRIPTOR_PATH', str(DATA / 'minimal.robot.yaml'))
    assert resolve_path('minimal') == str(DATA / 'minimal.robot.yaml')
    assert load('minimal').robot_id == 'minimal'


def test_resolve_prefix_scan(clean_env, tmp_path, sobit_home_path):
    cfg = tmp_path / 'sobit_home_description' / 'share' / 'sobit_home_description' / 'config'
    cfg.mkdir(parents=True)
    (cfg / 'sobit_home.robot.yaml').write_text(sobit_home_path.read_text())
    clean_env.setenv('COLCON_PREFIX_PATH', str(tmp_path))
    assert resolve_path('sobit_home') == str(cfg / 'sobit_home.robot.yaml')


def test_resolve_search_dirs_and_error(clean_env, tmp_path):
    assert resolve_path('minimal', search_dirs=[tmp_path, DATA]) == str(
        DATA / 'minimal.robot.yaml')
    with pytest.raises(DescriptorError, match='not found') as e:
        resolve_path('no_such_robot', search_dirs=[tmp_path])
    assert str(tmp_path / 'no_such_robot.robot.yaml') in str(e.value)


def test_requires_orbbec_vs_realsense(sobit_home_path):
    orbbec = load_file(sobit_home_path)
    assert orbbec.args == {'head_cam_type': 'orbbec'}
    assert [i.name for i in orbbec.imus] == ['head_camera_imu']
    assert orbbec.imu('head_camera_imu').topic == 'head_camera/gyro_accel/sample'
    realsense = load_file(sobit_home_path, args={'head_cam_type': 'realsense'})
    assert realsense.imus == []
    assert [c.name for c in realsense.cameras] == [c.name for c in orbbec.cameras]
    with pytest.raises(DescriptorError, match="Unknown imu 'head_camera_imu'"):
        realsense.imu('head_camera_imu')


@pytest.mark.parametrize('value, expected', [
    ('ORBBEC', 1), ('Orbbec', 1), ('realsense', 0), ('Realsense', 0)])
def test_requires_case_insensitive(sobit_home_path, value, expected):
    d = load_file(sobit_home_path, args={'head_cam_type': value})
    assert len(d.imus) == expected
    assert d.args['head_cam_type'] == value


def test_requires_bool_forms(tmp_path):
    data = yaml.safe_load(open(DATA / 'sobit_home.robot.yaml'))
    data['sensors']['imus'][0]['requires'] = {'enable_head': 'True'}
    data['urdf'].setdefault('args', {})['enable_head'] = 'true'
    for override, expected in [('True', 1), ('true', 1), (True, 1), ('false', 0),
                               ('False', 0), (False, 0), ('off', 0), ('yes', 1)]:
        assert len(parse(data, args={'enable_head': override}).imus) == expected, override


def test_derived_topics(desc):
    head = desc.group('head')
    assert head.interface == 'trajectory' and head.kind == 'position'
    assert head.command_topic == 'head_position_controller/joint_trajectory'
    assert head.command_action == 'head_position_controller/follow_joint_trajectory'
    assert head.state_topic == 'head_position_controller/controller_state'
    drive = desc.mobile_base.controllers[1]
    assert (drive.interface, drive.kind) == ('group', 'velocity')
    assert drive.command_topic == 'wheel_drive_velocity_controller/commands'


def test_derived_topics_group_interface_and_override(tmp_path):
    data = yaml.safe_load((DATA / 'minimal.robot.yaml').read_text())
    data['groups'][0]['interface'] = 'group'
    data['groups'].append({'name': 'b', 'controller': 'b_ctrl', 'joints': ['j3'],
                           'command_topic': 'custom/cmd'})
    p = tmp_path / 'm.robot.yaml'
    p.write_text(yaml.safe_dump(data))
    d = load_file(p)
    a = d.group('arm')
    assert (a.command_topic, a.command_action, a.state_topic) == (
        'arm_controller/commands', None, None)
    assert d.group('b').command_topic == 'custom/cmd'
    assert d.group('b').command_action == 'b_ctrl/follow_joint_trajectory'


def test_topic_and_frame_prefix(sobit_home_path):
    d = load_file(sobit_home_path, prefix='sobit_home/')
    assert d.topic(d.camera('head_camera').color.raw_topic) == \
        '/sobit_home/head_camera/color/image_raw'
    assert d.topic('/abs/topic') == '/abs/topic'
    assert d.frame(d.base_frame) == 'sobit_home/base_footprint'
    assert load_file(DATA / 'minimal.robot.yaml').topic('joint_states') == '/joint_states'
    assert load_file(sobit_home_path).frame('odom') == 'odom'


def test_lookups_and_streams(desc):
    head = desc.camera('head_camera')
    assert [s.kind for s in head.streams()] == ['color', 'depth']
    assert head.depth.range_m == [0.1, 10.0] and head.depth.encoding == '16UC1'
    assert [s.kind for s in desc.camera('hand_left_camera').streams()] == ['color']
    assert desc.lidar('lidar_merged').merged_from == ['lidar_front', 'lidar_back']
    assert desc.end_effector('left').control.group == 'arm_left'
    with pytest.raises(DescriptorError, match=r"Unknown group 'arm'.*'arm_left'"):
        desc.group('arm')


def test_joints_and_excluded(desc):
    joints = desc.joints()
    assert len(joints) == 2 + 1 + 7 * 4
    assert joints[:3] == ['head_pan_joint', 'head_tilt_joint', 'body_lift_joint']
    assert 'hand_left_finger_r_mcp_joint' not in joints
    full = desc.joints(include_uncommanded=True)
    assert len(full) == len(joints) + 2 and 'hand_right_finger_r_mcp_joint' in full
    assert desc.excluded_joints == []
    assert desc.all_excluded_joints == WHEELS
    assert [c.name for c in desc.controllers()] == [
        'head', 'body', 'arm_left', 'arm_right', 'hand_left', 'hand_right',
        'wheel_steer', 'wheel_drive']


def test_filtered(desc):
    f = desc.filtered(exclude_groups=['arm_right', 'hand_right'],
                      exclude_cameras=['hand_right_camera'], exclude_ee=['right'],
                      exclude_joints=['head_tilt_joint'])
    assert [g.name for g in f.groups] == ['head', 'body', 'arm_left', 'hand_left']
    assert f.group('head').joints == ['head_pan_joint']
    assert [c.name for c in f.cameras] == ['head_camera', 'hand_left_camera']
    assert [e.name for e in f.ee] == ['left']
    ex = f.all_excluded_joints
    assert 'arm_right_elbow_joint' in ex and 'hand_right_finger_r_mcp_joint' in ex
    assert 'head_tilt_joint' in ex and set(WHEELS) <= set(ex)
    assert len(desc.groups) == 6 and desc.excluded_joints == []  # original untouched
    assert desc.filtered() is desc


def test_filtered_drops_empty_group(desc):
    f = desc.filtered(exclude_joints=['body_lift_joint'])
    assert 'body' not in [g.name for g in f.groups]
    assert f.excluded_joints == ['body_lift_joint']


@pytest.mark.parametrize('kw,kind', [
    ({'exclude_groups': ['arm_lft']}, 'group'),
    ({'exclude_cameras': ['nope']}, 'camera'),
    ({'exclude_ee': ['middle']}, 'ee'),
    ({'exclude_joints': ['x_joint']}, 'joint'),
])
def test_filtered_unknown_name(desc, kw, kind):
    with pytest.raises(DescriptorError, match=f'Unknown {kind} name'):
        desc.filtered(**kw)


def test_filtered_all_groups(desc):
    names = [g.name for g in desc.groups]
    assert desc.filtered(exclude_groups=names).groups == []  # ee control survives
    with pytest.raises(DescriptorError, match='every joint group'):
        desc.filtered(exclude_groups=names, exclude_ee=['left', 'right'])


def test_to_dict(desc):
    d = desc.to_dict()
    assert d['schema_version'] == 2 and d['robot_id'] == 'sobit_home'
    assert d['joint_states_topic'] == '/sobit_home/joint_states'
    assert d['groups'][0]['command_topic'] == \
        '/sobit_home/head_position_controller/joint_trajectory'
    assert d['groups'][0]['controller'] == 'head_position_controller'
    assert d['sensors']['cameras'][0]['color']['frame'] == 'head_camera_color_optical_frame'
    assert d['mobile_base']['command_topic'] == '/sobit_home/cmd_vel'
    assert 'path' not in d and 'prefix' not in d
    rel = desc.to_dict(absolute_topics=False)
    assert rel['sensors']['lidars'][2]['scan_topic'] == 'lidar_scan'
    yaml.safe_dump(d)  # plain types only


def test_load_rejects_invalid(tmp_path):
    p = tmp_path / 'bad.robot.yaml'
    p.write_text((DATA / 'minimal.robot.yaml').read_text().replace(
        'joint_states_topic: joint_states', 'joint_states_topic: /joint_states'))
    with pytest.raises(DescriptorError, match='must be relative'):
        load_file(p)
    assert load_file(p, validate=False).joint_states_topic == '/joint_states'
    p.write_text('- not a mapping\n')
    with pytest.raises(DescriptorError, match='not a YAML mapping'):
        load_file(p)


def test_diff_drive(tmp_path):
    d = load_file(DATA / 'diffbot.robot.yaml')
    c = d.mobile_base.controllers[0]
    assert (c.interface, c.kind, c.joints) == (
        'diff_drive', 'velocity', ['wheel_left_joint', 'wheel_right_joint'])
    assert c.command_topic == 'cmd_vel' and d.topic(c.command_topic) == '/diffbot/cmd_vel'
    assert (c.wheel_radius, c.wheel_separation) == (0.05, 0.3)
    assert d.all_excluded_joints == ['wheel_left_joint', 'wheel_right_joint']
    assert d.controllers()[-1] is c
    out = d.to_dict()['mobile_base']['controllers'][0]
    assert out['command_topic'] == '/diffbot/cmd_vel'
    assert (out['wheel_radius'], out['wheel_separation']) == (0.05, 0.3)
    data = yaml.safe_load((DATA / 'diffbot.robot.yaml').read_text())
    data['mobile_base']['controllers'][0]['command_topic'] = 'diff_drive_controller/cmd_vel'
    assert parse(data).mobile_base.controllers[0].command_topic == \
        'diff_drive_controller/cmd_vel'


def test_base_controller_wheel_fields_default(desc):
    steer = desc.mobile_base.controllers[0]
    assert steer.wheel_radius is None and steer.wheel_separation is None
    assert desc.to_dict()['mobile_base']['controllers'][0]['wheel_radius'] is None
