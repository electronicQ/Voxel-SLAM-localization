#!/usr/bin/env python3
"""Evaluate map-frame localization trajectories and fingerprint a map package.

Both trajectories use the TUM format:
    timestamp tx ty tz qx qy qz qw

ATE is intentionally computed directly in the saved map frame without a
post-hoc SE(3) alignment.  This makes a globally wrong localization visible
instead of aligning the error away.
"""

import argparse
import bisect
import hashlib
import math
import pathlib
import sys
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w


@dataclass(frozen=True)
class Pose:
    stamp: float
    position: Vector3
    quaternion: Quaternion


@dataclass(frozen=True)
class LocalizationMetrics:
    matched_poses: int
    rpe_pairs: int
    ate_rmse_m: float
    rpe_translation_rmse_m: float
    rpe_rotation_rmse_deg: float


@dataclass(frozen=True)
class FileHash:
    relative_path: str
    sha256: str


@dataclass(frozen=True)
class MapHashReport:
    files: Tuple[FileHash, ...]
    tree_sha256: str


def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-15:
        raise ValueError("quaternion norm must be finite and non-zero")
    return tuple(value / norm for value in quaternion)  # type: ignore


def _quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quaternion_multiply(left: Quaternion,
                         right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate(quaternion: Quaternion, vector: Vector3) -> Vector3:
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _relative_pose(first: Pose, second: Pose) -> Tuple[Vector3, Quaternion]:
    inverse_rotation = _quaternion_conjugate(first.quaternion)
    translation = _rotate(
        inverse_rotation, _subtract(second.position, first.position)
    )
    rotation = _normalize_quaternion(
        _quaternion_multiply(inverse_rotation, second.quaternion)
    )
    return translation, rotation


def read_tum(path: pathlib.Path) -> List[Pose]:
    """Read a strictly time-ordered TUM trajectory."""
    path = pathlib.Path(path)
    poses: List[Pose] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 8:
                raise ValueError(
                    "{}:{}: expected 8 TUM fields, found {}".format(
                        path, line_number, len(fields)
                    )
                )
            try:
                values = tuple(float(field) for field in fields)
            except ValueError as exception:
                raise ValueError(
                    "{}:{}: invalid numeric field".format(path, line_number)
                ) from exception
            if not all(math.isfinite(value) for value in values):
                raise ValueError(
                    "{}:{}: non-finite trajectory value".format(
                        path, line_number
                    )
                )
            stamp = values[0]
            if poses and stamp <= poses[-1].stamp:
                raise ValueError(
                    "{}:{}: timestamps must be strictly increasing".format(
                        path, line_number
                    )
                )
            poses.append(
                Pose(
                    stamp=stamp,
                    position=(values[1], values[2], values[3]),
                    quaternion=_normalize_quaternion(
                        (values[4], values[5], values[6], values[7])
                    ),
                )
            )
    if not poses:
        raise ValueError("trajectory is empty: {}".format(path))
    return poses


def associate_poses(
    estimate: Sequence[Pose],
    reference: Sequence[Pose],
    max_time_diff: float,
) -> List[Tuple[Pose, Pose]]:
    """Associate estimate/reference poses monotonically by nearest timestamp."""
    if not math.isfinite(max_time_diff) or max_time_diff < 0.0:
        raise ValueError("max_time_diff must be finite and non-negative")
    reference_stamps = [pose.stamp for pose in reference]
    matches: List[Tuple[Pose, Pose]] = []
    last_reference_index = -1
    for estimated_pose in estimate:
        first_candidate = max(
            last_reference_index + 1,
            bisect.bisect_left(
                reference_stamps, estimated_pose.stamp - max_time_diff
            ),
        )
        best_index = -1
        best_difference = math.inf
        candidate = first_candidate
        while candidate < len(reference):
            difference = abs(reference[candidate].stamp - estimated_pose.stamp)
            if reference[candidate].stamp > estimated_pose.stamp + max_time_diff:
                break
            if difference < best_difference:
                best_difference = difference
                best_index = candidate
            candidate += 1
        if best_index >= 0:
            matches.append((estimated_pose, reference[best_index]))
            last_reference_index = best_index
    return matches


def _rmse(squared_errors: Iterable[float]) -> float:
    values = list(squared_errors)
    if not values:
        raise ValueError("cannot compute RMSE without samples")
    return math.sqrt(sum(values) / float(len(values)))


def compute_metrics(
    matches: Sequence[Tuple[Pose, Pose]], rpe_delta: int = 1
) -> LocalizationMetrics:
    """Compute unaligned map-frame ATE and fixed-index-delta RPE."""
    if rpe_delta <= 0:
        raise ValueError("rpe_delta must be positive")
    if len(matches) <= rpe_delta:
        raise ValueError(
            "at least {} matched poses are required".format(rpe_delta + 1)
        )

    ate_squared = [
        _norm(_subtract(estimated.position, reference.position)) ** 2
        for estimated, reference in matches
    ]
    translation_squared: List[float] = []
    rotation_squared: List[float] = []
    for first_index in range(0, len(matches) - rpe_delta):
        second_index = first_index + rpe_delta
        estimated_first, reference_first = matches[first_index]
        estimated_second, reference_second = matches[second_index]
        estimated_translation, estimated_rotation = _relative_pose(
            estimated_first, estimated_second
        )
        reference_translation, reference_rotation = _relative_pose(
            reference_first, reference_second
        )
        translation_error = _subtract(
            estimated_translation, reference_translation
        )
        rotation_error = _normalize_quaternion(
            _quaternion_multiply(
                _quaternion_conjugate(reference_rotation),
                estimated_rotation,
            )
        )
        rotation_angle = 2.0 * math.acos(
            max(-1.0, min(1.0, abs(rotation_error[3])))
        )
        translation_squared.append(_norm(translation_error) ** 2)
        rotation_squared.append(math.degrees(rotation_angle) ** 2)

    return LocalizationMetrics(
        matched_poses=len(matches),
        rpe_pairs=len(translation_squared),
        ate_rmse_m=_rmse(ate_squared),
        rpe_translation_rmse_m=_rmse(translation_squared),
        rpe_rotation_rmse_deg=_rmse(rotation_squared),
    )


def _sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def hash_map_directory(directory: pathlib.Path) -> MapHashReport:
    """Hash every regular map file plus a deterministic path-aware tree hash."""
    directory = pathlib.Path(directory)
    if not directory.is_dir():
        raise ValueError("map directory does not exist: {}".format(directory))
    paths = sorted(
        (path for path in directory.rglob("*") if path.is_file()),
        key=lambda path: path.relative_to(directory).as_posix(),
    )
    if not paths:
        raise ValueError("map directory contains no files: {}".format(directory))

    files: List[FileHash] = []
    tree_digest = hashlib.sha256()
    for path in paths:
        if path.is_symlink():
            raise ValueError("map directory contains a symbolic link: {}".format(path))
        relative_path = path.relative_to(directory).as_posix()
        file_digest = _sha256_file(path)
        files.append(FileHash(relative_path, file_digest))
        tree_digest.update(relative_path.encode("utf-8"))
        tree_digest.update(b"\0")
        tree_digest.update(bytes.fromhex(file_digest))
        tree_digest.update(b"\n")
    return MapHashReport(tuple(files), tree_digest.hexdigest())


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate an unaligned map-frame TUM trajectory and print "
            "SHA-256 hashes for the immutable map package."
        )
    )
    parser.add_argument("--estimate", required=True, type=pathlib.Path)
    parser.add_argument("--reference", required=True, type=pathlib.Path)
    parser.add_argument("--map-dir", required=True, type=pathlib.Path)
    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=0.02,
        help="maximum timestamp association error in seconds (default: 0.02)",
    )
    parser.add_argument(
        "--rpe-delta",
        type=int,
        default=1,
        help="RPE interval in associated-pose indices (default: 1)",
    )
    return parser


def main(arguments: Sequence[str] = None) -> int:
    parser = _build_argument_parser()
    options = parser.parse_args(arguments)
    try:
        estimate = read_tum(options.estimate)
        reference = read_tum(options.reference)
        matches = associate_poses(
            estimate, reference, max_time_diff=options.max_time_diff
        )
        metrics = compute_metrics(matches, rpe_delta=options.rpe_delta)
        map_hashes = hash_map_directory(options.map_dir)
    except (OSError, ValueError) as exception:
        parser.error(str(exception))

    print("matched_poses: {}".format(metrics.matched_poses))
    print("rpe_pairs: {}".format(metrics.rpe_pairs))
    print("ate_rmse_m: {:.9f}".format(metrics.ate_rmse_m))
    print(
        "rpe_translation_rmse_m: {:.9f}".format(
            metrics.rpe_translation_rmse_m
        )
    )
    print(
        "rpe_rotation_rmse_deg: {:.9f}".format(
            metrics.rpe_rotation_rmse_deg
        )
    )
    print("map_files:")
    for file_hash in map_hashes.files:
        print("  {}  {}".format(file_hash.sha256, file_hash.relative_path))
    print("map_tree_sha256: {}".format(map_hashes.tree_sha256))
    return 0


if __name__ == "__main__":
    sys.exit(main())
