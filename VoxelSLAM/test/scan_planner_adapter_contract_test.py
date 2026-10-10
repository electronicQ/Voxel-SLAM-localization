import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "scan_planner_odom_adapter.cpp"
CONFIG = ROOT / "config" / "ros2_scan_planner_adapter.yaml"
LAUNCH = ROOT / "launch" / "scan_planner_adapter.launch.py"
CMAKE = ROOT / "CMakeLists.txt"


class ScanPlannerAdapterContractTest(unittest.TestCase):
    def test_adapter_uses_required_interfaces_and_no_tf_broadcaster(self):
        source = SOURCE.read_text(encoding="utf-8")
        for token in (
            '"/LIO/odom_imu"',
            '"/LIO/odom_vehicle"',
            '"/localization/valid"',
            "composeVehicleOdom",
            "tf::TransformListener",
            "lookupTransform",
            "require_localization_valid",
            "input_timeout_sec",
        ):
            self.assertIn(token, source)
        self.assertNotIn("TransformBroadcaster", source)
        self.assertNotIn("sendTransform", source)
        self.assertIn("sameFrame(message->child_frame_id, imu_frame_)", source)

    def test_adapter_defaults_are_exact(self):
        config = CONFIG.read_text(encoding="utf-8")
        expected = {
            "imu_odom_topic": '"/LIO/odom_imu"',
            "vehicle_odom_topic": '"/LIO/odom_vehicle"',
            "imu_frame": '"aft_mapped"',
            "base_frame": '"base_link"',
            "localization_valid_topic": '"/localization/valid"',
            "require_localization_valid": "true",
            "tf_timeout_sec": "0.2",
            "input_timeout_sec": "0.2",
        }
        for key, value in expected.items():
            value = value.replace('"', '')
            self.assertIn(key + ": " + value, config)

    def test_launch_loads_private_config_and_only_starts_adapter(self):
        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("scan_planner_odom_adapter", launch)
        self.assertIn("ros2_scan_planner_adapter.yaml", launch)
        self.assertIn("Node(", launch)

    def test_cmake_registers_transform_and_adapter_targets(self):
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("src/vehicle_odom_transform.cpp", cmake)
        self.assertIn("add_executable(scan_planner_odom_adapter", cmake)
        self.assertIn("vehicle_odom_transform_test", cmake)


if __name__ == "__main__":
    unittest.main()
