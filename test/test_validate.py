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

import copy
import subprocess
import sys

import pytest
import yaml
from conftest import DATA, ROOT

from sobits_robot_descriptor import check, validate
from sobits_robot_descriptor.scaffold import render


@pytest.fixture(scope='module')
def base():
    return yaml.safe_load((DATA / 'sobit_home.robot.yaml').read_text())


def errors_of(data):
    return check(data)[0]


def mutated(base, fn):
    d = copy.deepcopy(base)
    fn(d)
    return errors_of(d)


def test_valid_files(base):
    assert check(base) == ([], [])
    assert validate(DATA / 'minimal.robot.yaml', strict=True) == []
    assert validate(str(DATA / 'sobit_home.robot.yaml')) == []


def test_schema_version_1(base):
    errs = mutated(base, lambda d: d.update(schema_version=1))
    assert len(errs) == 1 and 'schema_version must be 2, got 1' in errs[0]


def test_absolute_topic(base):
    def fn(d):
        d['sensors']['cameras'][0]['color']['raw_topic'] = '/sobit_home/head_camera/x'
    errs = mutated(base, fn)
    assert errs == ["camera 'head_camera'.color.raw_topic: topic "
                    "'/sobit_home/head_camera/x' must be relative to namespace "
                    '(drop the leading "/")']
    assert any('mobile_base.odom_topic' in e for e in
               mutated(base, lambda d: d['mobile_base'].update(odom_topic='/odom')))


def test_duplicate_joint(base):
    errs = mutated(base, lambda d: d['groups'][1]['joints'].append('head_pan_joint'))
    assert errs == ["joint 'head_pan_joint' listed in group 'head' and group 'body'"]
    errs = mutated(base, lambda d: d['groups'][4]['uncommanded_joints'].append(
        'wheel_drive_f_l_joint'))
    assert any("joint 'wheel_drive_f_l_joint' listed in" in e for e in errs)


def test_unknown_ee_group(base):
    errs = mutated(base, lambda d: d['ee'][0]['control'].update(group='arm_lft'))
    assert len(errs) == 1 and "control.group 'arm_lft' is not a group" in errs[0]


def test_missing_stream_frame(base):
    errs = mutated(base, lambda d: d['sensors']['cameras'][0]['depth'].pop('frame'))
    assert errs == ["camera 'head_camera'.depth: missing frame (optical frame of the stream)"]
    errs = mutated(base, lambda d: [d['sensors']['cameras'][1].pop(k)
                                    for k in ('color',)])
    assert errs == ["camera 'hand_left_camera': needs a color or depth stream"]


def test_other_rules(base):
    assert any('duplicate group/base controller name' in e for e in
               mutated(base, lambda d: d['groups'][1].update(name='head')))
    assert any('duplicate sensor name' in e for e in mutated(
        base, lambda d: d['sensors']['imus'][0].update(name='lidar_front')))
    assert any('interface must be one of' in e for e in
               mutated(base, lambda d: d['groups'][0].update(interface='traj')))
    assert any('kind must be one of' in e for e in mutated(
        base, lambda d: d['mobile_base']['controllers'][0].update(kind='effort')))
    assert any("merged_from 'lidar_side'" in e for e in mutated(
        base, lambda d: d['sensors']['lidars'][2].update(merged_from=['lidar_side'])))
    assert any("excluded_joints: 'body_lift_joint'" in e for e in
               mutated(base, lambda d: d.update(excluded_joints=['body_lift_joint'])))
    assert any("missing required key 'base_frame'" in e for e in
               mutated(base, lambda d: d.pop('base_frame')))
    assert any('needs raw_topic or compressed_topic' in e for e in mutated(
        base, lambda d: [d['sensors']['cameras'][1]['color'].pop(k)
                         for k in ('raw_topic', 'compressed_topic')]))


def test_warnings(base):
    d = copy.deepcopy(base)
    d['sensors']['imus'][0]['requires'] = {'imu_type': 'x'}
    d['excluded_joints'] = ['wheel_steer_f_l_joint']
    d['groups'][0]['max_joint_delta'] = 0.1
    errors, warnings = check(d)
    assert errors == []
    assert len(warnings) == 3
    assert validate(d) == [] and len(validate(d, strict=True)) == 3


def test_template_placeholders():
    errs = validate(ROOT / 'config' / 'template.robot.yaml')
    assert errs and all('placeholder' in e for e in errs)
    rendered = yaml.safe_load(render('my_robot'))
    assert rendered['robot_id'] == 'my_robot' and rendered['namespace'] == 'my_robot'
    assert rendered['description_package'] == 'my_robot_description'
    left = errors_of(rendered)
    assert left and not any('<robot_id>' in e or '<namespace>' in e for e in left)


def run_cli(*args):
    return subprocess.run([sys.executable, *args], cwd=ROOT, capture_output=True, text=True)


def test_validate_cli():
    ok = run_cli('scripts/validate', 'test/data/sobit_home.robot.yaml',
                 '--args', 'head_cam_type=realsense')
    assert ok.returncode == 0, ok.stdout + ok.stderr
    assert "imus: []" in ok.stdout
    bad = run_cli('scripts/validate', 'config/template.robot.yaml')
    assert bad.returncode == 1 and "placeholder '<robot_id>'" in bad.stdout
    assert run_cli('scripts/validate', 'no_such_robot').returncode == 2


def test_validate_cli_requires_case_insensitive():
    on = run_cli('scripts/validate', 'test/data/sobit_home.robot.yaml',
                 '--args', 'head_cam_type=ORBBEC')
    assert on.returncode == 0, on.stdout + on.stderr
    assert "imus: ['head_camera_imu']" in on.stdout


def test_new_robot_cli(tmp_path):
    r = run_cli('scripts/new_robot', '--robot-id', 'my_robot', '--namespace', '',
                '-o', str(tmp_path / 'new_dir'))
    assert r.returncode == 0, r.stderr
    out = tmp_path / 'new_dir' / 'my_robot.robot.yaml'
    assert yaml.safe_load(out.read_text())['namespace'] == ''
    assert 'install(DIRECTORY config DESTINATION share/${PROJECT_NAME})' in r.stdout
    assert '<base_link>' in r.stdout and 'test_robot_descriptor.py' in r.stdout
    assert run_cli('scripts/new_robot', '--robot-id', 'my_robot', '-o',
                   str(out)).returncode == 1


@pytest.fixture(scope='module')
def diffbot():
    return yaml.safe_load((DATA / 'diffbot.robot.yaml').read_text())


def test_diff_drive_valid(diffbot):
    assert check(diffbot) == ([], [])


@pytest.mark.parametrize('fn,msg', [
    (lambda c: c['joints'].append('wheel_caster_joint'),
     'left_joints + right_joints must equal joints'),
    (lambda c: c.pop('left_joints'), "missing required key 'left_joints'"),
    (lambda c: c.update(right_joints=[]), 'right_joints must not be empty'),
    (lambda c: c.update(right_joints=['wheel_left_joint']),
     "left_joints and right_joints overlap: ['wheel_left_joint']"),
    (lambda c: c.update(right_joints=['wheel_rear_joint']),
     'left_joints + right_joints must equal joints'),
    (lambda c: c.update(kind='position'), 'diff_drive needs kind: velocity'),
    (lambda c: c.pop('kind'), 'diff_drive needs kind: velocity'),
    (lambda c: c.pop('wheel_radius'), "missing required key 'wheel_radius'"),
    (lambda c: c.update(wheel_separation=0), 'wheel_separation must be a positive'),
    (lambda c: c.update(wheel_radius='5cm'), 'wheel_radius must be a positive'),
    (lambda c: c.update(interface='diffdrive'), 'interface must be one of'),
])
def test_diff_drive_rules(diffbot, fn, msg):
    errs = mutated(diffbot, lambda d: fn(d['mobile_base']['controllers'][0]))
    assert any(msg in e for e in errs), errs


def test_diff_drive_context_rules(diffbot):
    errs = mutated(diffbot, lambda d: d['mobile_base'].pop('command_topic'))
    assert any('diff_drive needs mobile_base.command_topic' in e for e in errs)
    errs = mutated(diffbot, lambda d: d['groups'][0].update(interface='diff_drive'))
    assert errs == ["group 'head': interface diff_drive is only valid under "
                    'mobile_base.controllers']
    errs = mutated(diffbot, lambda d: d.update(excluded_joints=['wheel_left_joint']))
    assert errs == ["excluded_joints: 'wheel_left_joint' is a diff_drive wheel joint "
                    "of base controller 'wheels' (excluded automatically)"]
    errs = mutated(diffbot, lambda d: d['mobile_base']['controllers'][0].update(
        interface='group', kind='velocity', wheel_radius=None, wheel_separation=None))
    assert errs == ["base controller 'wheels': left_joints/right_joints are only "
                    'valid for interface diff_drive']
    assert any('wheel_radius must be a positive' in e for e in mutated(
        diffbot, lambda d: d['mobile_base']['controllers'][0].update(
            interface='group', wheel_radius=-1)))
