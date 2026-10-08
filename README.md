# sobits_robot_descriptor

One hand-written `<robot>.robot.yaml` per robot, owned by the robot's description
package (`<robot>_description/config/<robot>.robot.yaml`), read by every SOBITS tool
(`urdf2usd_ros`, `sobits_viz`, `sobits_vla_tools`, `sobits_teleop`) through this
package. Tools keep only their own parameters, keyed by descriptor names.

- Python loader + validator (`yaml` and stdlib only; no `rclpy`)
- Header-only C++17 loader (`yaml-cpp` only)
- `validate` and `new_robot` CLIs, `config/template.robot.yaml`

## Schema v2

Topics are **relative** to `namespace` (a leading `/` is rejected) and frames are
bare URDF link names; the loader builds absolute topics (`topic()`) and adds an
optional frame prefix (`frame()`).

| Key | Type | Notes |
|---|---|---|
| `schema_version` | `2` | required |
| `robot_id` | str | required, `[A-Za-z][A-Za-z0-9_]*` |
| `version` | str | default `"1.0.0"` |
| `namespace` | str | required, may be `""` |
| `description_package` | str | default `<robot_id>_description` |
| `urdf` | `{xacro, urdf?, args?}` | paths relative to the description share dir; `args` = xacro defaults |
| `base_frame` | link | required |
| `odom_frame` | frame | default `odom` |
| `joint_states_topic` | topic | required |
| `groups[]` | `{name, controller, interface?, kind?, joints, uncommanded_joints?, command_topic?, command_action?, state_topic?}` | `interface`: `trajectory` (default) \| `group`; `kind`: `position` (default) \| `velocity`; `uncommanded_joints`: mimic joints published but never commanded |
| `mobile_base` | `{command_topic, odom_topic, has_vel_x/y/z/theta, controllers[]}` | optional; `controllers[] {name, controller, interface? (default group), kind?, joints, command_topic?}` hold the wheel joints |
| `sensors.cameras[]` | `{name, frame, model?, color?, depth?, requires?}` | `frame` = mount link; at least one stream |
| stream (`color`/`depth`) | `{frame, raw_topic, compressed_topic?, info_topic, encoding?, width?, height?, fps?, points_topic?, range_m?}` | `frame` = optical frame; `range_m: [min, max]` |
| `sensors.lidars[]` | `{name, frame, scan_topic, points_topic?, merged_from?, requires?}` | `merged_from` names other lidars |
| `sensors.imus[]` | `{name, frame, topic, requires?}` | |
| `*.requires` | `{xacro_arg: value}` | sensor entry exists only when every key equals `urdf.args` merged with caller overrides |
| `ee[]` | `{name, ee_link, reference_frame, control?}` | `control {group, command_frame, enable_topic}`, `group` must be a group name |
| `excluded_joints[]` | joint names | explicit extras; `all_excluded_joints` adds every `mobile_base.controllers[].joints` |

Derived topics when not given:

| interface | `command_topic` | `command_action` | `state_topic` |
|---|---|---|---|
| `trajectory` | `<controller>/joint_trajectory` | `<controller>/follow_joint_trajectory` | `<controller>/controller_state` |
| `group` | `<controller>/commands` | none | none |

Joint order in `groups[].joints` is the dataset feature order; do not reorder.
Package-specific settings (VLA `feature`/limits/`active`, viewer layout, Isaac sensor
optics) live in each tool's own config, keyed by these names.

Converting a v1 (`sobits_vla_common/robots`) file: drop the leading `/<ns>/` from
topics, replace `joints: [{ros_name, feature}]` with plain names, move the wheel
joints from `excluded_joints` into `mobile_base.controllers`, merge the `is_depth`
camera entry into its colour camera as `depth:`, and move `active`/`max_*`/`feature`
keys to the consumer config.

## Resolution order

`resolve_path(robot_id, search_dirs=None)` returns the first hit of:

1. `robot_id` itself, when it is an existing file
2. `SOBITS_ROBOT_DESCRIPTOR_PATH` (`:`-separated files or dirs; a dir is probed for
   `<dir>/<id>.robot.yaml` and `<dir>/<id>/<id>.robot.yaml`)
3. ament index: `share/<id>_description/config/<id>.robot.yaml` (Python only)
4. `AMENT_PREFIX_PATH`, then `COLCON_PREFIX_PATH` (merged and isolated installs)
5. `search_dirs` (same probes as 2)

Otherwise `DescriptorError` (C++: `std::runtime_error`) lists every candidate tried.

## Python API

```python
from sobits_robot_descriptor import load, load_file, resolve_path, validate

desc = load('sobit_home')                                # id or path
desc = load_file(path, prefix=None, args={'head_cam_type': 'realsense'}, validate=True)

desc.topic(desc.cameras[0].color.raw_topic)    # '/sobit_home/head_camera/color/image_raw'
desc.frame(desc.base_frame)                    # prefix + 'base_footprint'
desc.group('arm_left').joints
desc.camera('head_camera').streams()           # [color, depth]
desc.lidar('lidar_merged'), desc.imu('head_camera_imu'), desc.end_effector('left')
desc.joints(include_uncommanded=False)
desc.all_excluded_joints                       # excluded_joints + wheel joints
desc.controllers()                             # groups + mobile base controllers
desc.filtered(exclude_groups=['arm_right'], exclude_cameras=[], exclude_ee=['right'],
              exclude_joints=[])               # unknown names raise DescriptorError
desc.to_dict(absolute_topics=True)

validate('sobit_home')                         # [] when valid; strict=True adds warnings
```

`filtered()` folds the joints of removed groups/joints into `excluded_joints`, drops
groups left empty, and refuses to remove every group unless an `ee` with `control`
survives.

Outside ROS (e.g. the Isaac Lab venv): `uv pip install -e <src>/sobits_robot_descriptor`
and set `SOBITS_ROBOT_DESCRIPTOR_PATH`.

## C++ API

```cmake
find_package(sobits_robot_descriptor REQUIRED)
target_link_libraries(my_node sobits_robot_descriptor::sobits_robot_descriptor)
```

```cpp
#include <sobits_robot_descriptor/loader.hpp>
namespace srd = sobits_robot_descriptor;

srd::RobotDescriptor d = srd::load_file(path, /*prefix=*/"", /*args=*/{{"head_cam_type", "orbbec"}});
d.topic(d.cameras[0].color->raw_topic);
d.frame(d.base_frame);
d.group("arm_left").joints;  d.camera("head_camera").streams();
d.joints(true);  d.all_excluded_joints();
srd::resolve_path("sobit_home");  // env + prefix scan, no ament_index dependency
```

C++ only checks `schema_version` and required keys; run the Python validator for the
full rule set. `namespace` is `namespace_` and `requires` is `requires_` in C++.
Prefer passing the resolved path from the Python launch file.

## CLIs

```sh
ros2 run sobits_robot_descriptor validate sobit_home
ros2 run sobits_robot_descriptor validate sobit_home --args head_cam_type=realsense --strict
ros2 run sobits_robot_descriptor new_robot --robot-id my_robot [--namespace NS] [-o DIR]
```

`validate` exits 0 when valid, 1 on errors (or warnings with `--strict`), 2 when the
file cannot be found or read. Rules: schema v2, required keys, unique names, a joint
in at most one group / base controller / uncommanded list, `interface`/`kind` values,
relative topics, every camera has a stream and every stream a frame and a topic,
`ee.control.group` and `merged_from` resolve, `requires` keys in `urdf.args`
(warning), `excluded_joints` not in a group, no `<...>` placeholders left.

`new_robot` fills `robot_id`/`namespace`/`description_package` from the template and
prints the remaining placeholders plus the CMake install line. Outside ROS the same
commands are `sobits-robot-descriptor-validate` / `sobits-robot-descriptor-new-robot`.

## License

BSD-3-Clause, see [LICENSE](LICENSE).
