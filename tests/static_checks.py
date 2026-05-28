#!/usr/bin/env python3
import ast
import importlib.util
import pathlib
import py_compile
import sys
import types
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_yaml(path):
    try:
        import yaml
    except ImportError as exc:
        raise AssertionError("PyYAML is required for static checks") from exc

    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_python_files_compile():
    files = [
        ROOT / "bringup.launch.py",
        ROOT / "launch" / "spawn.launch.py",
        ROOT / "launch" / "slam.launch.py",
        ROOT / "launch" / "nav.launch.py",
        ROOT / "launch" / "rtabmap.launch.py",
        ROOT / "scripts" / "waypoint_nav.py",
        ROOT / "scripts" / "forklift_controller.py",
        ROOT / "scripts" / "apriltag_detect.py",
        ROOT / "scripts" / "spawn_robot.py",
    ]
    for path in files:
        py_compile.compile(str(path), doraise=True)


def test_forklift_motion_stays_inside_bounds_and_reverses():
    sys.modules.setdefault("rclpy", types.ModuleType("rclpy"))
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules.setdefault("rclpy.node", rclpy_node)

    gazebo_msgs = types.ModuleType("gazebo_msgs")
    gazebo_msgs_srv = types.ModuleType("gazebo_msgs.srv")
    gazebo_msgs_msg = types.ModuleType("gazebo_msgs.msg")
    gazebo_msgs_srv.SetEntityState = object
    gazebo_msgs_msg.EntityState = object
    sys.modules.setdefault("gazebo_msgs", gazebo_msgs)
    sys.modules.setdefault("gazebo_msgs.srv", gazebo_msgs_srv)
    sys.modules.setdefault("gazebo_msgs.msg", gazebo_msgs_msg)

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Pose = object
    geometry_msgs_msg.Twist = object
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    path = ROOT / "scripts" / "forklift_controller.py"
    spec = importlib.util.spec_from_file_location("forklift_controller", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    y, direction = 3.0, -1
    positions = []
    for _ in range(500):
        y, direction = module.advance_along_y(y, direction, 0.6, 0.05, -3.0, 3.0)
        positions.append(y)

    assert min(positions) >= -3.0
    assert max(positions) <= 3.0
    assert -3.0 in positions
    assert 3.0 in positions


def test_nav2_config_uses_humble_plugin_keys():
    config = load_yaml(ROOT / "config" / "nav2.yaml")

    bt_params = config["bt_navigator"]["ros__parameters"]
    assert "bt_xml_filename" not in bt_params
    assert "default_bt_xml_filename" not in bt_params

    controller = config["controller_server"]["ros__parameters"]
    assert controller["controller_plugins"] == ["FollowPath"]
    assert controller["progress_checker_plugins"] == ["progress_checker"]
    assert controller["goal_checker_plugins"] == ["goal_checker"]
    assert controller["progress_checker"]["plugin"] == "nav2_controller::SimpleProgressChecker"
    assert controller["goal_checker"]["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert "controller_plugin_ids" not in controller

    planner = config["planner_server"]["ros__parameters"]
    assert planner["planner_plugins"] == ["GridBased"]
    assert planner["GridBased"]["plugin"] == "nav2_smac_planner::SmacPlannerHybrid"
    assert "planner_plugin_ids" not in planner

    behavior = config["behavior_server"]["ros__parameters"]
    assert behavior["behavior_plugins"] == ["spin", "backup", "wait"]
    assert behavior["spin"]["plugin"] == "nav2_behaviors::Spin"
    assert behavior["backup"]["plugin"] == "nav2_behaviors::BackUp"
    assert behavior["wait"]["plugin"] == "nav2_behaviors::Wait"
    assert "behavior_plugin_ids" not in behavior


def test_manifest_declares_runtime_dependencies():
    root = ET.parse(ROOT / "package.xml").getroot()
    deps = {element.text for element in root.findall("depend")}
    required = {
        "xacro",
        "rviz2",
        "rtabmap_slam",
        "rtabmap_viz",
        "nav2_smac_planner",
        "dwb_core",
        "dwb_critics",
    }
    assert required <= deps


def test_spawn_launch_uses_world_launch_argument():
    source = (ROOT / "launch" / "spawn.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    assigned_launch_config = any(
        isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "world" for target in node.targets)
        and isinstance(node.value, ast.Call)
        and getattr(node.value.func, "id", "") == "LaunchConfiguration"
        for node in ast.walk(tree)
    )
    assert assigned_launch_config
    assert '"--verbose", world,' in source


def test_bringup_has_modes_and_map_validation():
    source = (ROOT / "bringup.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)

    assert 'choices=["sim", "slam", "nav"]' in source
    assert 'OpaqueFunction(function=validate_nav_map)' in source
    assert 'Shutdown(reason="navigation map is missing")' in source
    assert 'condition=mode_is("slam")' in source
    assert 'condition=mode_is("nav")' in source
    assert 'condition=mode_in(["sim", "nav"])' in source

    assigned_world_launch_config = any(
        isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "world" for target in node.targets)
        and isinstance(node.value, ast.Call)
        and getattr(node.value.func, "id", "") == "LaunchConfiguration"
        for node in ast.walk(tree)
    )
    assert assigned_world_launch_config


def test_nav_launch_checks_map_before_starting_nav2():
    source = (ROOT / "launch" / "nav.launch.py").read_text(encoding="utf-8")

    assert "def validate_map_file(context):" in source
    assert 'OpaqueFunction(function=validate_map_file)' in source
    assert 'Shutdown(reason="navigation map is missing")' in source


def test_referenced_project_files_exist():
    required = [
        ROOT / "config" / "amcl.yaml",
        ROOT / "config" / "cartographer.lua",
        ROOT / "config" / "nav2.yaml",
        ROOT / "config" / "nav.rviz",
        ROOT / "config" / "slam.rviz",
        ROOT / "description" / "robot.xacro",
        ROOT / "worlds" / "warehouse.world",
        ROOT / "launch" / "spawn.launch.py",
        ROOT / "launch" / "slam.launch.py",
        ROOT / "launch" / "nav.launch.py",
        ROOT / "launch" / "rtabmap.launch.py",
        ROOT / "scripts" / "waypoint_nav.py",
        ROOT / "scripts" / "forklift_controller.py",
    ]
    missing = [str(path.relative_to(ROOT)) for path in required if not path.exists()]
    assert not missing, f"missing project files: {missing}"


def test_readme_documents_current_bringup_flow():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    required_snippets = [
        "python3 tests/static_checks.py",
        "ros2 launch project bringup.launch.py mode:=sim",
        "ros2 launch project bringup.launch.py mode:=slam",
        "ros2 launch project bringup.launch.py mode:=nav",
        "ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map",
        "ros2 run project waypoint_nav.py",
        "mode:=nav` 直接退出",
        "PASS test_readme_documents_current_bringup_flow",
        "docs/验收清单.md",
    ]
    missing = [snippet for snippet in required_snippets if snippet not in readme]
    assert not missing, f"README missing snippets: {missing}"


def test_acceptance_checklist_documents_runtime_validation():
    checklist_path = ROOT / "docs" / "验收清单.md"
    assert checklist_path.exists()
    checklist = checklist_path.read_text(encoding="utf-8")

    required_snippets = [
        "python3 tests/static_checks.py",
        "colcon build --symlink-install",
        "ros2 launch project bringup.launch.py mode:=sim",
        "ros2 run tf2_ros tf2_echo odom base_footprint",
        "ros2 launch project bringup.launch.py mode:=slam",
        "ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map",
        "ros2 launch project bringup.launch.py mode:=nav",
        "ros2 run project waypoint_nav.py",
        "ros2 launch project rtabmap.launch.py",
        "10 项测试全部输出 `PASS`",
        "成功标准",
        "失败时优先检查",
    ]
    missing = [snippet for snippet in required_snippets if snippet not in checklist]
    assert not missing, f"acceptance checklist missing snippets: {missing}"


def run_all():
    tests = [
        test_python_files_compile,
        test_forklift_motion_stays_inside_bounds_and_reverses,
        test_nav2_config_uses_humble_plugin_keys,
        test_manifest_declares_runtime_dependencies,
        test_spawn_launch_uses_world_launch_argument,
        test_bringup_has_modes_and_map_validation,
        test_nav_launch_checks_map_before_starting_nav2,
        test_referenced_project_files_exist,
        test_readme_documents_current_bringup_flow,
        test_acceptance_checklist_documents_runtime_validation,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    run_all()
