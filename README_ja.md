[English](README.md) | [日本語](README_ja.md)

# sobits_robot_descriptor

各ロボットに対して、ロボットの説明パッケージ（`<robot>_description/config/<robot>.robot.yaml`）
に所有される1つの手書き `<robot>.robot.yaml` があり、このパッケージを通じて全てのSOBITSツール
（`urdf2usd_ros`、`sobits_viz`、`sobits_vla_tools`、`sobits_teleop`）に読み込まれます。
ツールは記述子名をキーとして、独自のパラメータのみを保持します。

- Python ローダー + バリデーター（`yaml` および標準ライブラリのみ、`rclpy` 不要）
- ヘッダーのみの C++17 ローダー（`yaml-cpp` のみ）
- `validate` および `new_robot` CLI、`config/template.robot.yaml`

## Schema v2

トピックは `namespace` に対して**相対的**（先頭の `/` は拒否）で、フレームは
素の URDF リンク名です。ローダーは絶対トピック（`topic()`）を構築し、
オプショナルなフレームプレフィックス（`frame()`）を追加します。

| Key | Type | Notes |
|---|---|---|
| `schema_version` | `2` | 必須 |
| `robot_id` | str | 必須、`[A-Za-z][A-Za-z0-9_]*` |
| `version` | str | デフォルト `"1.0.0"` |
| `namespace` | str | 必須、`""` 可 |
| `description_package` | str | デフォルト `<robot_id>_description` |
| `urdf` | `{xacro, urdf?, args?}` | パスは説明共有ディレクトリに相対；`args` = xacro デフォルト |
| `base_frame` | link | 必須 |
| `odom_frame` | frame | デフォルト `odom` |
| `joint_states_topic` | topic | 必須 |
| `groups[]` | `{name, controller, interface?, kind?, joints, uncommanded_joints?, command_topic?, command_action?, state_topic?}` | `interface`: `trajectory` (デフォルト) \| `group`；`kind`: `position` (デフォルト) \| `velocity`；`uncommanded_joints`: 模倣ジョイント（公開されるが命令されない） |
| `mobile_base` | `{command_topic, odom_topic, has_vel_x/y/z/theta, controllers[]}` | オプショナル；`controllers[] {name, controller, interface? (デフォルト group), kind?, joints, command_topic?, wheel_radius?, wheel_separation?, left_joints?, right_joints?}` はホイール関連ジョイント；ここでのみ `interface: diff_drive` も可（下記参照） |
| `sensors.cameras[]` | `{name, frame, model?, color?, depth?, requires?}` | `frame` = マウントリンク；少なくとも1つのストリーム |
| stream (`color`/`depth`) | `{frame, raw_topic, compressed_topic?, info_topic, encoding?, width?, height?, fps?, points_topic?, range_m?}` | `frame` = 光学フレーム；`range_m: [min, max]` |
| `sensors.lidars[]` | `{name, frame, scan_topic, points_topic?, merged_from?, requires?}` | `merged_from` は他の LiDAR を指定 |
| `sensors.imus[]` | `{name, frame, topic, requires?}` | |
| `*.requires` | `{xacro_arg: value}` | センサーエントリは各キーが `urdf.args` と呼び出し元のオーバーライドをマージしたものと等しい場合のみ存在（大文字小文字区別なし；`true/yes/on` および `false/no/off` ブール値として比較） |
| `ee[]` | `{name, ee_link, reference_frame, control?}` | `control {group, command_frame, enable_topic}`、`group` はグループ名である必要 |
| `excluded_joints[]` | joint names | 明示的な追加；`all_excluded_joints` は全ての `mobile_base.controllers[].joints` を追加 |

指定されないときの導出トピック：

| interface | `command_topic` | `command_action` | `state_topic` |
|---|---|---|---|
| `trajectory` | `<controller>/joint_trajectory` | `<controller>/follow_joint_trajectory` | `<controller>/controller_state` |
| `group` | `<controller>/commands` | なし | なし |
| `diff_drive`（ベースのみ） | `mobile_base.command_topic`（cmd_vel twist） | なし | なし |

`diff_drive` のベースコントローラーは `kind: velocity`、`left_joints` と `right_joints`
（空でなく、互いに重ならず、合わせてちょうど `joints`）、`wheel_radius` と `wheel_separation`
（m、> 0）、および `mobile_base.command_topic` が必要です。`left_joints`/`right_joints` は
`diff_drive` でのみ有効です。
そのジョイントは `excluded_joints` に記載しないこと（自動的に除外されます）。

`groups[].joints` 内のジョイント順序はデータセット機能順です。並べ替えないこと。
パッケージ固有の設定（VLA `feature`/limits/`active`、ビューアレイアウト、Isaac センサー
光学）は各ツール独自の設定に存在し、これらの名前をキーとします。

v1（`sobits_vla_common/robots`）ファイルの変換：トピックから先頭の `/<ns>/` を削除、
`joints: [{ros_name, feature}]` を平文名に置き換え、ホイールジョイントを `excluded_joints`
から `mobile_base.controllers` に移動、`is_depth` カメラエントリをその色カメラに `depth:` として
マージ、`active`/`max_*`/`feature` キーをコンシューマー設定に移動します。

## Resolution order

`resolve_path(robot_id, search_dirs=None)` は以下の最初のヒットを返します：

1. `robot_id` 自体（既存ファイルの場合）
2. `SOBITS_ROBOT_DESCRIPTOR_PATH`（`:` で区切られたファイルまたはディレクトリ；
   ディレクトリは `<dir>/<id>.robot.yaml` および `<dir>/<id>/<id>.robot.yaml` を調査）
3. ament インデックス：`share/<id>_description/config/<id>.robot.yaml`（Python のみ）
4. `AMENT_PREFIX_PATH`、次に `COLCON_PREFIX_PATH`（マージおよび分離インストール）
5. `search_dirs`（2 と同じ調査）

そうでなければ、試行された全候補を列挙する `DescriptorError`（C++：`std::runtime_error`）。

## Python API

```python
from sobits_robot_descriptor import load, load_file, resolve_path, validate

desc = load('sobit_home')                                # id または path
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

`filtered()` は削除されたグループ/ジョイントのジョイントを `excluded_joints` に折り込み、
空のグループをドロップし、`ee` が `control` を持つエントリが残っていない場合は全グループの
削除を拒否します。

ROS 外（例：Isaac Lab venv）：`uv pip install -e <src>/sobits_robot_descriptor`
を実行し、`SOBITS_ROBOT_DESCRIPTOR_PATH` を設定します。

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

C++ は `schema_version` と必須キーのみをチェック；Python バリデーターで
全ルールセットを実行します。`namespace` は C++ では `namespace_`、`requires` は `requires_`
です。Python ローチファイルから解決パスを渡すことを推奨します。

## CLIs

```sh
ros2 run sobits_robot_descriptor validate sobit_home
ros2 run sobits_robot_descriptor validate sobit_home --args head_cam_type=realsense --strict
ros2 run sobits_robot_descriptor new_robot --robot-id my_robot [--namespace NS] [-o DIR]
```

`validate` は有効な場合は 0 で終了、エラーで 1（または `--strict` ありの警告）、
ファイルが見つからないまたは読み込めない場合は 2 で終了します。ルール：schema v2、必須キー、
一意の名前、グループ/ベースコントローラー/非命令リストで最大1つのジョイント、`interface`/`kind`
値（`diff_drive` は `mobile_base.controllers` のみ、上記ルール付き）、相対トピック、全カメラはストリームを持ち全ストリームはフレームとトピックを持つ、
`ee.control.group` および `merged_from` が解決、`requires` キーが `urdf.args` に存在
（警告）、`excluded_joints` がグループや `diff_drive` コントローラーに含まれない、プレースホルダー `<...>` が残らない。

`new_robot` はテンプレートから `robot_id`/`namespace`/`description_package` を埋め、
残りのプレースホルダーおよび CMake インストール行を出力します。ROS 外でも同じ
コマンドは `sobits-robot-descriptor-validate` / `sobits-robot-descriptor-new-robot` です。

## License

BSD-3-Clause、[LICENSE](LICENSE) を参照してください。
