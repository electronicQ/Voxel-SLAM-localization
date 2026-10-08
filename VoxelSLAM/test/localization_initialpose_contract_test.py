import pathlib
import re
import unittest


VOXELSLAM_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = VOXELSLAM_ROOT.parent
LOCALIZATION_SOURCE = VOXELSLAM_ROOT / "src" / "localization_state.cpp"
RVIZ_CONFIG = VOXELSLAM_ROOT / "rviz_cfg" / "back.rviz"
LOCALIZATION_CONFIG = VOXELSLAM_ROOT / "config" / "localization_mid360.yaml"
README = REPOSITORY_ROOT / "README.md"
PACKAGE_README = VOXELSLAM_ROOT / "README.md"


def function_body(source, signature):
    start = source.find(signature)
    if start < 0:
        raise AssertionError("missing function: {}".format(signature))
    brace = source.find("{", start)
    if brace < 0:
        raise AssertionError("missing body: {}".format(signature))
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError("unterminated body: {}".format(signature))


def if_condition(source, required_token):
    for match in re.finditer(r"\bif\s*\(", source):
        opening = source.find("(", match.start())
        depth = 0
        for index in range(opening, len(source)):
            if source[index] == "(":
                depth += 1
            elif source[index] == ")":
                depth -= 1
                if depth == 0:
                    condition = source[opening + 1:index]
                    if required_token in condition:
                        return condition
                    break
    raise AssertionError(
        "missing if condition containing: {}".format(required_token)
    )


def evaluate_rejection_predicate(frame_id, compared_frames, operators):
    comparisons = [frame_id != allowed for allowed in compared_frames]
    result = comparisons[0]
    for operator, comparison in zip(operators, comparisons[1:]):
        if operator == "&&":
            result = result and comparison
        elif operator == "||":
            result = result or comparison
        else:
            raise AssertionError(
                "unsupported logical operator: {}".format(operator)
            )
    return result


def markdown_section(source, start_heading, end_heading):
    start = source.find(start_heading)
    if start < 0:
        raise AssertionError("missing heading: {}".format(start_heading))
    end = source.find(end_heading, start + len(start_heading))
    if end < 0:
        raise AssertionError("missing heading: {}".format(end_heading))
    return source[start:end]


class LocalizationInitialPoseContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.localization_source = LOCALIZATION_SOURCE.read_text(encoding="utf-8")
        cls.rviz_config = RVIZ_CONFIG.read_text(encoding="utf-8")
        cls.localization_config = LOCALIZATION_CONFIG.read_text(encoding="utf-8")
        cls.readme = README.read_text(encoding="utf-8")
        cls.package_readme = PACKAGE_README.read_text(encoding="utf-8")

    def test_default_rviz_frame_matches_initialpose_contract(self):
        self.assertRegex(
            self.rviz_config,
            r"(?m)^\s+Fixed Frame:\s*camera_init\s*$",
        )
        submit_body = function_body(
            self.localization_source, "bool InitialPoseRequestBuffer::submit("
        )
        predicate = if_condition(submit_body, "frame_id")
        compared_frames = re.findall(r'frame_id\s*!=\s*"([^"]*)"', predicate)
        operators = re.findall(r"&&|\|\|", predicate)

        expected_frames = {"camera_init", "/camera_init", "map", "/map"}
        self.assertEqual(set(compared_frames), expected_frames)
        self.assertEqual(len(compared_frames), len(expected_frames))
        self.assertEqual(operators, ["&&", "&&", "&&"])
        self.assertNotIn("||", predicate)

        for frame in sorted(expected_frames):
            self.assertFalse(
                evaluate_rejection_predicate(frame, compared_frames, operators),
                "allowed frame rejected by extracted predicate: {}".format(frame),
            )
        for frame in ("odom", "base_link", ""):
            self.assertTrue(
                evaluate_rejection_predicate(frame, compared_frames, operators),
                "unsupported frame accepted by extracted predicate: {!r}".format(
                    frame
                ),
            )
        self.assertIn("allowed map frames", submit_body)

    def test_readme_documents_default_and_compatible_initialpose_frames(self):
        self.assertIn("默认 RViz Fixed Frame 是 `camera_init`", self.readme)
        self.assertIn("无需额外发布 `map -> camera_init` TF", self.readme)
        self.assertRegex(
            self.readme,
            r"`map`[^\n]{0,80}(兼容|别名)",
        )

    def test_readme_defines_map_aliases_as_same_coordinate_without_tf_lookup(self):
        for start_heading, end_heading in (
            ("### 4.5 完成初始重定位", "### 4.6 检查定位状态"),
            ("### 6.4 RViz 人工初始化", "### 6.5 两条通道的公共汇合路径"),
        ):
            section = markdown_section(self.readme, start_heading, end_heading)
            self.assertIn("默认且推荐的 frame 是 `camera_init`", section)
            self.assertIn("`map` 和 `/map`", section)
            self.assertIn("数值坐标与 `camera_init` 完全相同", section)
            self.assertIn("`InitialPoseRequestBuffer` 不查询 TF", section)
            self.assertIn("不执行 `map -> camera_init` 变换", section)
            self.assertIn("非恒等变换", section)
            self.assertIn("先把位姿转换到 `camera_init`", section)
            self.assertIn("不能直接使用 `map` 别名", section)

        submit_body = function_body(
            self.localization_source, "bool InitialPoseRequestBuffer::submit("
        )
        self.assertIn(
            "map aliases are same-coordinate compatibility aliases",
            submit_body,
        )
        self.assertIn("no TF transform occurs", submit_body)

    def test_scan_planner_steps_distinguish_existing_session_and_full_stack(self):
        section = markdown_section(
            self.readme,
            "### 4.7 接入 SCAN-Planner（可选）",
            "## 5. 建图与地图准备原理",
        )

        self.assertIn("模式 A：沿用 4.4 已启动的定位会话", section)
        self.assertIn("不得再次启动 `vxlm_localization_mid360.launch`", section)
        self.assertIn("保持现有 Voxel-SLAM 和传感器/rosbag 输入", section)
        self.assertIn("仅新增或确认", section)

        self.assertIn("模式 B：从零启动实机完整栈", section)
        for process in (
            "传感器驱动/输入",
            "用户 URDF",
            "Voxel-SLAM 定位",
            "adapter",
            "裁剪后的 SCAN-Planner",
        ):
            self.assertIn(process, section)
        self.assertIn("roslaunch your_lidar_driver", section)
        self.assertIn("按实际设备替换", section)
        self.assertEqual(
            len(
                re.findall(
                    r"(?m)^roslaunch voxel_slam "
                    r"vxlm_localization_mid360\.launch",
                    section,
                )
            ),
            1,
        )

        self.assertIn("同一个 catkin 工作空间", section)
        self.assertIn("独立 overlay", section)
        self.assertIn("source ~/catkin_ws/devel/setup.bash", section)
        self.assertIn("source ~/scan_planner_ws/devel/setup.bash", section)

    def test_scan_planner_world_cloud_and_pose_extrinsic_are_independent(self):
        for name, readme in (
            ("root README", self.readme),
            ("package README", self.package_readme),
        ):
            self.assertIn("grid_map/cloud_is_world=true", readme, name)
            self.assertIn("grid_map/need_extrinsic=true", readme, name)
            self.assertNotIn("grid_map/need_extrinsic=false", readme, name)
        self.assertIn("两个彼此独立的开关", self.readme)

    def test_scan_planner_mid360_extrinsic_matches_localization_config(self):
        self.assertRegex(
            self.localization_config,
            r"(?m)^\s+extrinsic_tran:\s*\[-0\.011,\s*-0\.02329,\s*0\.04412\]\s*$",
        )
        self.assertRegex(
            self.localization_config,
            r"(?ms)^\s+extrinsic_rota:\s*\[1,\s*0,\s*0,\s*"
            r"0,\s*1,\s*0,\s*0,\s*0,\s*1\]\s*$",
        )
        for name, readme in (
            ("root README", self.readme),
            ("package README", self.package_readme),
        ):
            self.assertIn("[-0.011, -0.02329, 0.04412]", readme, name)
            self.assertIn("单位旋转", readme, name)
            self.assertIn("General/extrinsic_tran", readme, name)
            self.assertIn("General/extrinsic_rota", readme, name)

    def test_scan_planner_is_external_and_documents_safe_launch_integration(self):
        self.assertIn(
            "https://github.com/wuyi2121/SCAN-Planner", self.readme
        )
        self.assertIn("外部工程", self.readme)
        self.assertRegex(self.readme, r"catkin[^\n]{0,120}(overlay|叠加)")
        self.assertIn("go2_robot_state_publisher", self.readme)
        self.assertIn("robot_description", self.readme)
        self.assertRegex(self.readme, r"(复制|拷贝)[^\n]{0,80}(裁剪|删减)")
        self.assertRegex(
            self.readme,
            r"(不要|避免)[^\n]{0,80}(重复|同时)[^\n]{0,80}(启动|发布)",
        )
        self.assertRegex(
            self.readme,
            r"(未|没有)[^\n]{0,80}Ubuntu[^\n]{0,80}(联调|集成验证)",
        )

        self.assertIn("外部工程", self.package_readme)
        self.assertRegex(self.package_readme, r"catkin[^\n]{0,120}(overlay|叠加)")
        self.assertIn("robot_description", self.package_readme)
        self.assertIn("go2_robot_state_publisher", self.package_readme)
        self.assertIn("根目录 `README.md`", self.package_readme)

    def test_dependencies_include_boost_and_rosdep_guidance(self):
        self.assertRegex(
            self.readme,
            r"Boost[^\n]{0,80}(filesystem|FileSystem)[^\n]{0,80}system",
        )
        self.assertIn("rosdep install", self.readme)


if __name__ == "__main__":
    unittest.main()
