import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CPP_PATH = ROOT / "src" / "voxelslam.cpp"
HPP_PATH = ROOT / "src" / "voxelslam.hpp"
COMPAT_PATH = ROOT / "include" / "ros" / "ros.h"
CONFIG_DIRECTORY = ROOT / "config"


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


def yaml_block(source, heading):
    lines = source.splitlines()
    marker = heading + ":"
    try:
        start = lines.index(marker) + 1
    except ValueError as error:
        raise AssertionError("missing YAML block: {}".format(heading)) from error
    body = []
    for line in lines[start:]:
        if line and not line[0].isspace():
            break
        body.append(line)
    return "\n".join(body)


class HighRateOdometryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp = CPP_PATH.read_text(encoding="utf-8")
        cls.hpp = HPP_PATH.read_text(encoding="utf-8")
        cls.compat = COMPAT_PATH.read_text(encoding="utf-8")
        cls.source = cls.compat + "\n" + cls.hpp + "\n" + cls.cpp

    def test_imu_subscription_uses_dedicated_single_thread_queue(self):
        self.assertIn("rclcpp", self.source)
        self.assertIn("n.subscribe<sensor_msgs::msg::Imu>", self.cpp)
        self.assertIn("imu_callback", self.cpp)
        self.assertIn("&imu_callback_queue", self.cpp)
        self.assertIn("SingleThreadedExecutor", self.source)
        self.assertNotIn("ros::SubscribeOptions", self.cpp)
        self.assertNotRegex(self.source, r"\bvoid\s+imu_handler\s*\(")

    def test_member_callback_buffers_before_predicting_and_handles_rollback(self):
        body = function_body(self.cpp, "void imu_callback(")
        append_position = body.find("imu_buf.push_back")
        predict_position = body.find("imu_rate_predictor.pushImu")
        self.assertGreaterEqual(append_position, 0)
        self.assertGreater(predict_position, append_position)
        self.assertIn("imu_last_time", body)
        self.assertIn("imu_buf.clear()", body)
        self.assertIn("reset_imu_rate_predictor", body)

    def test_high_rate_publish_uses_sensor_time_and_required_state_fields(self):
        body = function_body(self.cpp, "void publish_imu_rate_odometry(")
        self.assertIn("ros::Time::fromSec(prediction.state.t)", body)
        self.assertNotIn("ros::Time::now()", body)
        self.assertIn("prediction.state.R", body)
        self.assertIn("prediction.state.p", body)
        self.assertIn("prediction.state.v", body)
        self.assertIn("prediction.unbiased_angular_velocity", body)
        self.assertIn("pose.covariance", body)
        self.assertIn("twist.covariance", body)
        self.assertIn("imu_tf_broadcaster.sendTransform", body)

    def test_topic_gating_snapshot_refresh_and_reset_hooks_exist(self):
        self.assertIn('"/LIO/odom_imu"', self.cpp)
        self.assertIn("publish_before_localized", self.cpp)
        self.assertIn("localization_valid.load()", self.cpp)
        correction_body = function_body(
            self.cpp, "bool apply_frontend_global_correction("
        )
        reset_body = function_body(self.cpp, "void system_reset(")
        destructor_body = function_body(self.cpp, "~VOXEL_SLAM()")
        self.assertIn("refresh_imu_rate_predictor_snapshot()", correction_body)
        self.assertRegex(
            self.cpp,
            r"scan_quality\.solve_succeeded[\s\S]{0,160}"
            r"refresh_imu_rate_predictor_snapshot\(\)",
        )
        self.assertRegex(
            self.cpp,
            r"x_curr\.R\s*=\s*x_buf\[win_count-1\]\.R;[\s\S]{0,220}"
            r"refresh_imu_rate_predictor_snapshot\(\)",
        )
        self.assertIn("reset_imu_rate_predictor()", reset_body)
        self.assertIn("shutdown_imu_rate_output()", destructor_body)
        self.assertIn("~VOXEL_SLAM()", self.cpp)

    def test_scan_rate_tf_is_sensor_stamped_and_has_high_rate_fallback(self):
        body = function_body(self.cpp, "void pub_odom_func(")
        self.assertIn("ros::Time::fromSec(xc.t)", body)
        self.assertNotIn("ros::Time::now()", body)
        self.assertIn("publish_scan_tf", self.cpp)
        self.assertIn("imu_prediction_ready.load()", self.cpp)

    def test_scan_tf_fallback_remains_enabled_when_snapshot_replay_is_rejected(self):
        body = function_body(
            self.cpp, "void refresh_imu_rate_predictor_snapshot("
        )
        self.assertIn(
            "const bool snapshot_ready =\n"
            "        imu_rate_predictor.updateSnapshot",
            body,
        )
        self.assertIn("imu_prediction_ready.store(snapshot_ready)", body)
        self.assertNotIn("imu_prediction_ready.store(true)", body)

    def test_sync_packages_protects_timestamp_and_empty_buffer(self):
        body = function_body(self.hpp, "bool sync_packages(")
        lock_position = body.find("lock_guard<mutex> lock(mBuf)")
        timestamp_position = body.find("imu_last_time")
        empty_position = body.find("imu_buf.empty()")
        front_position = body.find("imu_buf.front()")
        self.assertGreaterEqual(lock_position, 0)
        self.assertGreater(timestamp_position, lock_position)
        self.assertGreaterEqual(empty_position, 0)
        self.assertGreater(front_position, empty_position)
        self.assertIn("if(imu_time > p_imu.pcl_end_time) break;", body)
        self.assertNotIn("if(imu_time >= p_imu.pcl_end_time) break;", body)

    def test_all_six_odometry_configs_have_exact_defaults(self):
        expected_files = {
            "avia.yaml",
            "avia_fly.yaml",
            "hesai.yaml",
            "localization_mid360.yaml",
            "mid360.yaml",
            "velodyne.yaml",
        }
        expected_values = {
            "imu_rate_output": "true",
            "imu_odom_topic": '"/LIO/odom_imu"',
            "odom_frame": '"camera_init"',
            "imu_child_frame": '"aft_mapped"',
            "publish_before_localized": "false",
            "max_prediction_sec": "0.5",
        }

        for name in sorted(expected_files):
            text = (CONFIG_DIRECTORY / name).read_text(encoding="utf-8")
            body = yaml_block(text, "Odometry")
            for key, value in expected_values.items():
                self.assertRegex(
                    body,
                    r"(?m)^  {}:\s*{}\s*$".format(
                        re.escape(key), re.escape(value)
                    ),
                    "{} missing {}: {}".format(name, key, value),
                )


if __name__ == "__main__":
    unittest.main()
