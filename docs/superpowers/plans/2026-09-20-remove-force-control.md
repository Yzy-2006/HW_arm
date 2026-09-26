# 移除力传感器与力控链路 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让项目只保留位置遥操、夹爪控制和相机采集，并彻底去除两路六维力传感器及相关力控链路。

**Architecture:** 启动层不再启动或检查 `encos_force_sensor`；采集层不订阅或保存 wrench 数据；VR 与上层控制层不再传递人工力前馈；双臂节点直接把名义笛卡尔位置目标交给既有 IK/位置控制路径，不再读取外力、接触门控或执行阻抗/导纳修正。EtherCAT、重力补偿、关节反馈、位置轨迹和双夹爪控制保持不变。

**Tech Stack:** GNU Make、ROS 2 Jazzy、C++17/rclcpp、Python 3/pytest、YAML、colcon。

**Spec:** `docs/superpowers/specs/2026-09-20-remove-force-control-design.md`

## Global Constraints

- 不改变 EtherCAT 电机通信、自动回零、关节反馈、低层 PD 限幅、重力补偿或急停流程。
- 保留双臂 IK/FK、位置轨迹、VR 左右扳机位置增量、双夹爪 squeeze、录制状态与三路相机采集。
- `make teleop` 与 `make experiment` 在没有 `can0`、`can1` 时仍可启动；它们不得启动力节点。
- 不提供力控运行时开关；从源码、配置、启动、采集和文档路径中移除该功能。
- 每次修改只编辑工作区源码；不覆盖用户已有的非本任务改动，也不自动提交。
- 真机验收仅由现场操作者在急停可触及、工作区清空的条件下完成。

## Review Focus

- 缺少 `can0`/`can1` 时，`make -n teleop` 与 `make -n experiment` 不应出现 `start_force`、`force_sensor_node` 或 `encos_force_sensor`。
- VR 正常 tick 只能发布位置增量与夹爪状态，右手 B 键不得再创建或发布 wrench 消息。
- 采集配置不含 `force_topics` 时，采集器仍注册关节、TCP、目标、夹爪与相机流，且不导入 `WrenchStamped`。
- 双臂节点失去 `/force_sensor/*/wrench` 发布者后仍只走名义位置目标、`controlCal()` 和 `controlOutp()`，不能访问未初始化的力向量。
- 删除源码包后，旧 `build/install/log/encos_force_sensor` 产物不得继续被 preflight、Makefile 或 launch 视为运行时依赖。

---

## File Structure

- `tests/test_force_removal.py` — 静态验证启动、采集、VR、控制源和文档均无已移除的力控接口。
- `Makefile`, `scripts/run.sh`, `scripts/preflight.py`, `ros2_ws/src/towel_bringup/launch/towel_experiment.launch.py` — 控制启动面只保留 servo、arms、robot、VR 和采集器。
- `collection/record.py`, `config/recording.yaml` — 移除 wrench 订阅及数据集字段来源。
- `teleop/VR_controller/vr_pose_increment_publisher.py`, `teleop/VR_controller/config/gripper_pd.yaml`, `ros2_ws/src/robot_control_hw/{include,src}/robot_control/*` — 移除右手 B 键六维力前馈发布、配置、订阅和转发。
- `ros2_ws/src/dual_arm_control_hw/**`, `ros2_ws/src/ctl_msgs_hw/config/admittanceImpPara.yaml` — 移除外力传感器、接触检测、阻抗/导纳类与配置；保留位置、IK/FK 和重力补偿。
- `ros2_ws/src/encos_force_sensor/**` — 删除不再使用的 ROS 包。
- `README.md`, `docs/*.md`, `docs/copied_sources.json` — 删除力传感器操作说明、依赖和来源记录，更新操作顺序。

### Task 1: 删除启动层与独立驱动包

**Files:**
- Create: `tests/test_force_removal.py`
- Modify: `Makefile:29-171`
- Modify: `scripts/run.sh:5-13`
- Modify: `scripts/preflight.py:34-41`
- Modify: `ros2_ws/src/towel_bringup/launch/towel_experiment.launch.py:51-103`
- Delete: `ros2_ws/src/encos_force_sensor/`

**Interfaces:**
- Consumes: 现有 `make teleop`, `make experiment`, `make launch` 入口。
- Produces: `make teleop` 启动 `servo → arms → robot → VR`，`make experiment` 加入采集器；不再有 force/tare-force 命令或 `start_force` Launch 参数。

- [ ] **Step 1: 写出启动层失败测试**

在 `tests/test_force_removal.py` 创建以下检查：

```python
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def make_dry_run(target):
    return subprocess.run(
        ["make", "-n", target], cwd=ROOT, check=False,
        capture_output=True, text=True,
    )

def test_teleop_and_experiment_do_not_start_force_sensor():
    for target in ("teleop", "experiment"):
        result = make_dry_run(target)
        assert result.returncode == 0
        assert "start_force" not in result.stdout
        assert "force_sensor_node" not in result.stdout
        assert "encos_force_sensor" not in result.stdout

def test_makefile_exposes_no_force_sensor_commands_or_driver_package():
    makefile = (ROOT / "Makefile").read_text()
    assert "force:" not in makefile
    assert "tare-force:" not in makefile
    assert not (ROOT / "ros2_ws" / "src" / "encos_force_sensor").exists()
```

- [ ] **Step 2: 运行测试并确认它失败**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_force_removal.py`

Expected: FAIL；当前 Makefile 和源码包仍包含 force 入口。

- [ ] **Step 3: 最小化修改启动层**

实施以下确切变更：

```make
# Makefile: experiment / teleop 只传递下列 Launch 参数
start_servo:=true start_arms:=true start_robot:=true start_record:=true start_vr:=true
start_servo:=true start_arms:=true start_robot:=true start_vr:=true
```

```python
# towel_experiment.launch.py: 仅保留这些声明和定时节点
DeclareLaunchArgument("start_servo", default_value="false")
DeclareLaunchArgument("start_arms", default_value="false")
DeclareLaunchArgument("start_robot", default_value="false")
DeclareLaunchArgument("start_record", default_value="false")
DeclareLaunchArgument("start_vr", default_value="false")
timed_node(0.0, "servo_control_hw", "servo_node", LaunchConfiguration("start_servo"))
timed_node(2.0, "dual_arm_control_hw", "dual_arm_control_node", LaunchConfiguration("start_arms"))
timed_node(4.0, "robot_control_hw", "robot_control_node", LaunchConfiguration("start_robot"))
```

删除 Makefile 的 `force`、`tare-force` 目标和帮助文本，删除 `scripts/run.sh` 的 `force)` 分支，
删除 preflight 的 `encos_force_sensor` 包检查和整个 `ros2_ws/src/encos_force_sensor` 源码目录。

- [ ] **Step 4: 运行启动层测试并确认通过**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_force_removal.py -k 'teleop or makefile'`

Expected: PASS。

- [ ] **Step 5: 核对运行时构建产物**

先使用只读命令确认目录存在：

```bash
find ros2_ws/build ros2_ws/install ros2_ws/log -maxdepth 1 -type d -name encos_force_sensor -print
```

若输出精确列出旧包产物，删除仅这些已确认路径，再重新构建；不得删除整个 `build`、`install` 或 `log` 目录。

### Task 2: 从采集格式移除力字段

**Files:**
- Modify: `collection/record.py:90-134`
- Modify: `config/recording.yaml:1-31`
- Modify: `tests/test_force_removal.py`

**Interfaces:**
- Consumes: `make record` 传入的 YAML 配置与既有 ROS 关节/TCP/夹爪/图像话题。
- Produces: `make_node(config, cameras)` 只为关节、目标、TCP、夹爪和相机注册订阅；采集帧没有 `force_topic_left` 或 `force_topic_right`。

- [ ] **Step 1: 扩展失败测试，固定采集边界**

追加：

```python
def test_recorder_configuration_and_source_do_not_define_wrench_streams():
    config = (ROOT / "config" / "recording.yaml").read_text()
    recorder = (ROOT / "collection" / "record.py").read_text()
    assert "force_topics:" not in config
    assert "WrenchStamped" not in recorder
    assert 'config.get("force_topics"' not in recorder
```

- [ ] **Step 2: 运行该测试并确认失败**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_force_removal.py::test_recorder_configuration_and_source_do_not_define_wrench_streams`

Expected: FAIL；当前采集器导入并订阅 `WrenchStamped`。

- [ ] **Step 3: 移除采集器 wrench 订阅与 YAML 字段**

从 `record.py` 删除：

```python
from geometry_msgs.msg import WrenchStamped
for key, topic in config.get("force_topics", {}).items():
    subs.append((WrenchStamped, topic, lambda m, key=key: self.put(key, ...)))
```

从 `recording.yaml` 删除完整的 `force_topics:` 映射；保留 `cameras:`、时间阈值和其他标签。

- [ ] **Step 4: 验证采集测试**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_force_removal.py::test_recorder_configuration_and_source_do_not_define_wrench_streams tests/test_collection.py`

Expected: PASS。

### Task 3: 删除 VR 人工力前馈与上层转发

**Files:**
- Modify: `teleop/VR_controller/vr_pose_increment_publisher.py:22,136-142,475-503,818-833`
- Modify: `teleop/VR_controller/config/gripper_pd.yaml:1-10`
- Modify: `ros2_ws/src/robot_control_hw/include/robot_control/msgPublisher.h`
- Modify: `ros2_ws/src/robot_control_hw/src/robot_control/trajProcessWrap.cpp:182-186,245-260`
- Modify: `tests/test_grippers.py`
- Modify: `tests/test_vr_debug.py`
- Modify: `tests/test_force_removal.py`

**Interfaces:**
- Consumes: `run_vr_tick(node, wrapper, debug_controller_input=False)` 与现有 VR 位姿/夹爪消息。
- Produces: 正常 tick 只调用 `node.publish_increment(telemetry)` 和 `node.update_gripper(telemetry)`；`robot_control_node` 不订阅 `/vr/right_arm_force_feedforward`。

- [ ] **Step 1: 将现有 VR 测试改成失败的“无力前馈”断言**

删除 `test_vr_right_b_holds_right_arm_feedforward`，并追加：

```python
def test_normal_vr_tick_publishes_only_position_and_gripper(monkeypatch):
    module = load_vr_publisher(monkeypatch)
    calls = []
    node = types.SimpleNamespace(
        publish_increment=lambda data: calls.append(("increment", data)),
        update_gripper=lambda data: calls.append(("gripper", data)),
    )
    wrapper = types.SimpleNamespace(get_motion_state_data=lambda: "telemetry")
    module.run_vr_tick(node, wrapper)
    assert calls == [("increment", "telemetry"), ("gripper", "telemetry")]
```

并在 `test_force_removal.py` 追加：

```python
def test_vr_and_robot_control_have_no_force_feedforward_topic():
    vr = (ROOT / "teleop/VR_controller/vr_pose_increment_publisher.py").read_text()
    robot = (ROOT / "ros2_ws/src/robot_control_hw/src/robot_control/trajProcessWrap.cpp").read_text()
    assert "right_arm_force_feedforward" not in vr
    assert "right_arm_force_feedforward" not in robot
```

- [ ] **Step 2: 运行测试并确认失败**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_grippers.py tests/test_vr_debug.py tests/test_force_removal.py -k 'normal_vr_tick or feedforward'`

Expected: FAIL；当前 `run_vr_tick` 仍调用 `publish_arm_feedforward`。

- [ ] **Step 3: 只删除人工力前馈路径**

实施以下行为变更：

```python
# vr_pose_increment_publisher.py
def run_vr_tick(node, tv_wrapper, debug_controller_input=False):
    telemetry = tv_wrapper.get_motion_state_data()
    if debug_controller_input:
        node.report_controller_debug(tv_wrapper.controller_debug_snapshot())
        return
    node.publish_increment(telemetry)
    node.update_gripper(telemetry)
```

删除 `Wrench` import、`right_arm_feedforward_force` 默认值/YAML 字段、publisher、启动日志和
`publish_arm_feedforward()` 方法。删除 `RobotCtlNode` 的 `geometry_msgs::msg::Wrench` 订阅指针、
`rArm_feedforward_callback` 声明/定义和 `/vr/right_arm_force_feedforward` 订阅。保留机器人其他
`geometry_msgs` 用途，例如底盘 `Twist`。

- [ ] **Step 4: 验证 VR 与上层控制静态接口**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_grippers.py tests/test_vr_debug.py tests/test_force_removal.py -k 'vr or feedforward' && python3 -m py_compile teleop/VR_controller/vr_pose_increment_publisher.py`

Expected: PASS。

### Task 4: 将双臂节点收敛为位置控制路径

**Files:**
- Modify: `ros2_ws/src/dual_arm_control_hw/src/dual_arm_control/msgPublish.cpp:35-90,220-255,850-1165,1440-1500,1535-1640,1741-1767`
- Modify: `ros2_ws/src/dual_arm_control_hw/include/dual_arm_control/msgPublish.h:27,173-179,448-463,585-626`
- Modify: `ros2_ws/src/dual_arm_control_hw/include/controller/ArmInterface.h`
- Modify: `ros2_ws/src/dual_arm_control_hw/src/controller/ArmInterface.cpp`
- Delete: `ros2_ws/src/dual_arm_control_hw/include/controller/ImpedanceAdmittanceController.h`
- Delete: `ros2_ws/src/dual_arm_control_hw/src/controller/ImpedanceAdmittanceController.cpp`
- Modify: `ros2_ws/src/dual_arm_control_hw/include/robot_config/msgDefine.h`
- Modify: `ros2_ws/src/dual_arm_control_hw/include/robot_config/config.h`
- Modify: `ros2_ws/src/dual_arm_control_hw/src/robot_config/config.cpp`
- Modify: `ros2_ws/src/dual_arm_control_hw/CMakeLists.txt`
- Modify: `ros2_ws/src/dual_arm_control_hw/package.xml`
- Delete: `ros2_ws/src/ctl_msgs_hw/config/admittanceImpPara.yaml`
- Modify: `tests/test_force_removal.py`

**Interfaces:**
- Consumes: `modCtlMsg_get`, joint feedback, Cartesian position targets, `controlCal()`, `controlOutp()` and existing gravity-compensation structures.
- Produces: `ArmCtlNode::control_seal()` sends the original nominal Cartesian target through `controlCal()` and `controlOutp()` without subscribing to wrench topics or adding external-force-derived torque.

- [ ] **Step 1: 增加控制源的失败测试**

追加：

```python
def test_dual_arm_control_contains_no_sensor_or_admittance_control_path():
    source = (ROOT / "ros2_ws/src/dual_arm_control_hw/src/dual_arm_control/msgPublish.cpp").read_text()
    header = (ROOT / "ros2_ws/src/dual_arm_control_hw/include/dual_arm_control/msgPublish.h").read_text()
    assert "/force_sensor/" not in source
    assert "lArm_TorSensor_msg" not in source
    assert "rArm_TorSensor_msg" not in source
    assert "AdmImpCtlCore" not in source
    assert "WrenchStamped" not in header
    assert not (ROOT / "ros2_ws/src/ctl_msgs_hw/config/admittanceImpPara.yaml").exists()
```

- [ ] **Step 2: 运行测试并确认失败**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_force_removal.py::test_dual_arm_control_contains_no_sensor_or_admittance_control_path`

Expected: FAIL；节点仍订阅力话题并含导纳控制分支。

- [ ] **Step 3: 移除外力驱动的控制状态和支路**

从 `msgPublish.cpp/.h` 删除两路 `WrenchStamped` 订阅和回调、`lArm_6DTorSensor*`/
`rArm_6DTorSensor*` 缓存、空载偏置轨迹、接触基线/门控状态和初始化/循环中的采样逻辑。
把 `control_seal()` 中以下两块完整删除：

```cpp
if (modCtlMsg_get.transMod == 4 && AdmImpSwitch == 2) {
    // 传感器偏置、坐标旋转、接触检测、导纳修正
}

if (modCtlMsg_get.transMod == 4 && AdmImpSwitch == 1) {
    // 阻抗末端力计算、EndTor_To_Jnt() 映射、关节前馈力矩叠加
}
```

保留名义目标的复制、原有 `controlCal()`、`controlOutp()`、反馈发布和 `l_desPos_last`/
`r_desPos_last` 更新，使笛卡尔位置遥操的输入输出顺序不变。

- [ ] **Step 4: 移除已无调用者的力控类型、配置和依赖**

删除 `ImpedanceAdmittanceController.*` 与 CMake 源列表项；从 `ArmInterface.*` 移除
`admittance_Impendence_Ctl`、`admittance_outp()`、传感器低通状态及仅供力控使用的方法；从
`msgDefine.h`、`config.h`、`config.cpp` 删除 `ImpAdmPara`、`ImpAdmConfig`、
`loadImpAdmFromYaml()`、`yaml_path_impAdm` 和其全局变量；删除 YAML。

仅在 `geometry_msgs` 不再被 `dual_arm_control_hw` 任一源文件包含后，才从该包的
`CMakeLists.txt` 与 `package.xml` 删除此依赖。保留独立于本任务的重力补偿和 IK/FK 类型。

- [ ] **Step 5: 以编译器和静态测试验证位置控制收敛**

Run:

```bash
source .venv-robot/bin/activate
source scripts/env.sh
pytest -q tests/test_force_removal.py::test_dual_arm_control_contains_no_sensor_or_admittance_control_path
colcon build --packages-select dual_arm_control_hw robot_control_hw ctl_msgs_hw --symlink-install
```

Expected: PASS；若编译器报告残留符号，删除该残留的力控声明/调用，重复本步骤，不能以空实现替代。

### Task 5: 清理文档、来源记录并做全量验证

**Files:**
- Modify: `README.md`
- Modify: `docs/REAL_ROBOT_EXPERIMENT.md`
- Modify: `docs/VR_TELEOP_CHECK.md`
- Modify: `docs/DEPENDENCIES.md`
- Modify: `docs/VALIDATION.md`
- Modify: `docs/copied_sources.json`
- Delete: `docs/FORCE_SENSOR.md`
- Modify: `tests/test_force_removal.py`

**Interfaces:**
- Consumes: Task 1–4 的无力控启动和控制接口。
- Produces: 文档只指导 `servo → arms → robot → VR`，README 无 force/tare-force 命令，来源记录无已删除包。

- [ ] **Step 1: 添加文档/来源失败测试**

追加：

```python
def test_user_facing_runtime_docs_do_not_advertise_removed_force_workflow():
    paths = [
        ROOT / "README.md",
        ROOT / "docs/REAL_ROBOT_EXPERIMENT.md",
        ROOT / "docs/VR_TELEOP_CHECK.md",
        ROOT / "docs/DEPENDENCIES.md",
        ROOT / "docs/copied_sources.json",
    ]
    text = "\n".join(path.read_text() for path in paths)
    assert "make force" not in text
    assert "tare-force" not in text
    assert "encos_force_sensor" not in text
    assert not (ROOT / "docs/FORCE_SENSOR.md").exists()
```

- [ ] **Step 2: 运行测试并确认失败**

Run: `source .venv-robot/bin/activate && source scripts/env.sh && pytest -q tests/test_force_removal.py::test_user_facing_runtime_docs_do_not_advertise_removed_force_workflow`

Expected: FAIL；当前 README 和专项文档仍把力传感器列为运行步骤。

- [ ] **Step 3: 更新操作文档与来源记录**

删除 `FORCE_SENSOR.md` 及 README 中的力传感器表格行、`make force`/`make tare-force` 用法和
`force_sensor.yaml` 配置表。把实验和 VR 文档的控制顺序写为：

```text
servo -> arms -> robot -> VR
```

删除右 B 键力前馈、接触验证、去皮、力话题排障说明；保留扳机位置、squeeze 夹爪、录制/
丢弃与空载小幅验证说明。删除 `DEPENDENCIES.md` 与 `VALIDATION.md` 的 force package 条目，
以及 `copied_sources.json` 中 `encos_force_sensor` 对象。

- [ ] **Step 4: 运行全量无硬件验证**

Run:

```bash
source .venv-robot/bin/activate
source scripts/env.sh
make build
make test
python3 -m py_compile collection/record.py teleop/VR_controller/vr_pose_increment_publisher.py
git diff --check
rg -n -i 'start_force|force_sensor|force_topics|tare-force|right_arm_force_feedforward|admittanceImpPara' \
  --glob '!ros2_ws/{build,install,log}/**' \
  --glob '!docs/superpowers/{plans,specs}/**' .
```

Expected: 构建、测试、语法检查和 diff 检查均通过；最后的 `rg` 对已移除运行时接口没有匹配。

- [ ] **Step 5: 交付现场验收顺序**

向操作者交付以下命令顺序，并明确每条命令位于独立终端：

```bash
make servo
make watch-arm SIDE=both RATE=3
make arms
make robot
make vr
```

说明首次仅做空载、小幅位置和 squeeze 夹爪测试；不再运行 `make force` 或任何去皮命令。
