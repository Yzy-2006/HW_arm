"""Metric palm calibration and color-only planar PnP, independent of ROS."""

import json
import os
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


ANCHOR_IDS = (0, 5, 9, 13, 17)


def camera_parameters(intrinsics, serial):
    coefficients = np.asarray(intrinsics.coeffs, dtype=float)
    distortion = str(intrinsics.model).split(".")[-1]
    if np.any(np.abs(coefficients) > 1e-10) and distortion not in ("none", "brown_conrady"):
        raise ValueError(f"PnP does not support nonzero {distortion} camera distortion")
    return {
        "serial": str(serial), "width": int(intrinsics.width), "height": int(intrinsics.height),
        "matrix": [[intrinsics.fx, 0, intrinsics.ppx], [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]],
        "distortion_model": distortion, "distortion_coefficients": coefficients.tolist(),
    }


def validate_calibration(data):
    """Reject malformed or incompatible geometry before opening the camera."""
    try:
        if data["version"] != 1 or data["units"] != "m":
            raise ValueError("Expected calibration version 1 in meters")
        camera = data["camera"]
        matrix = np.asarray(camera["matrix"], dtype=float)
        coefficients = np.asarray(camera["distortion_coefficients"], dtype=float)
        if (
            not isinstance(camera["serial"], str) or not camera["serial"]
            or not isinstance(camera["width"], int) or camera["width"] <= 0
            or not isinstance(camera["height"], int) or camera["height"] <= 0
            or matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or not np.allclose([matrix[0, 1], matrix[1, 0]], 0)
            or coefficients.shape != (5,) or not np.isfinite(coefficients).all()
        ):
            raise ValueError("Invalid camera intrinsics")
        if np.any(np.abs(coefficients) > 1e-10) and camera["distortion_model"] != "brown_conrady":
            raise ValueError("Unsupported camera distortion")
        hands = data["hands"]
        if not isinstance(hands, dict) or not hands or set(hands) - {"Left", "Right"}:
            raise ValueError("Expected calibrated Left and/or Right hands")
        for model in hands.values():
            points = np.asarray(model["points_m"], dtype=float)
            if (
                model["anchor_ids"] != list(ANCHOR_IDS) or points.shape != (5, 3)
                or not np.isfinite(points).all() or np.max(np.abs(points)) > 0.2
                or not np.allclose(points[:, 2], 0, atol=1e-7)
                or not np.allclose(points.mean(axis=0), 0, atol=1e-6)
                or not 0.025 < np.linalg.norm(points[1] - points[4]) < 0.14
                or not 0.025 < np.linalg.norm(points[2] - points[0]) < 0.18
                or (points[1] - points[4])[0] <= 0
                or not np.allclose((points[2] - points[0])[[0, 2]], 0, atol=1e-6)
                or (points[2] - points[0])[1] <= 0
                or np.linalg.svd(points, compute_uv=False)[1] < 0.008
            ):
                raise ValueError("Invalid canonical planar palm geometry")
    except (KeyError, TypeError, IndexError) as error:
        raise ValueError("Malformed hand calibration") from error
    return data


def load_calibration(path):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"Calibration missing: {path}. Run make calibrate first.")
    with path.open(encoding="utf-8") as stream:
        return validate_calibration(json.load(stream))


def check_camera(calibration, current):
    saved = calibration["camera"]
    if any(saved[key] != current[key] for key in ("serial", "width", "height", "distortion_model")):
        raise ValueError("Camera/resolution differs from calibration; recalibrate for this color stream")
    if (
        not np.allclose(saved["matrix"], current["matrix"], rtol=1e-5, atol=1e-5)
        or not np.allclose(saved["distortion_coefficients"], current["distortion_coefficients"], atol=1e-6)
    ):
        raise ValueError("Camera intrinsics changed; recalibrate for this color stream")


def palm_rotation_hint(world_landmarks):
    """Camera-relative orientation from RGB model landmarks, with the RGB-D axis convention."""
    points = np.asarray(world_landmarks, dtype=float)
    if points.shape != (21, 3):
        return None
    anchors = points[list(ANCHOR_IDS)]
    if not np.isfinite(anchors).all():
        return None
    lateral, longitudinal = points[5] - points[17], points[9] - points[0]
    width, length = np.linalg.norm(lateral), np.linalg.norm(longitudinal)
    cross = np.cross(lateral, longitudinal)
    if min(width, length) < 1e-5 or np.linalg.norm(cross) < 0.3 * width * length:
        return None
    _, singular, vectors = np.linalg.svd(anchors - anchors.mean(axis=0), full_matrices=False)
    scale = 0.5 * (width + length)
    if singular[1] < 0.1 * scale or singular[2] / np.sqrt(5) > 0.15 * scale:
        return None
    normal = vectors[-1]
    if np.dot(normal, cross) < 0:
        normal = -normal
    y_axis = longitudinal - np.dot(longitudinal, normal) * normal
    y_axis /= np.linalg.norm(y_axis)
    return Rotation.from_matrix(np.column_stack((np.cross(y_axis, normal), y_axis, normal)))


class PalmCalibrator:
    """Accumulate unsmoothed depth poses in a canonical, approximately rigid palm frame."""

    def __init__(self, camera, labels=("Left", "Right"), samples=60, interval=0.10):
        self.camera = camera
        self.target = samples
        self.interval = interval
        self.samples = {label: [] for label in labels}
        self.last = {}
        self.models = {}
        self.status = {label: "waiting" for label in labels}

    def add(self, label, pose, now):
        if label not in self.samples or label in self.models:
            return False
        self.status[label] = "sampling"
        if now - self.last.get(label, -float("inf")) < self.interval:
            return False
        if pose["quality"] < 0.55:
            self.status[label] = "low depth support"
            return False
        if pose["plane_rms_m"] > 0.006:
            self.status[label] = "palm depth not planar"
            return False
        anchors = np.asarray(pose["joints"])[list(ANCHOR_IDS)]
        if not np.isfinite(anchors).all():
            self.status[label] = "missing palm anchor depth"
            return False
        rotation = Rotation.from_quat(pose["orientation"])
        local = rotation.inv().apply(anchors - pose["position"])
        local[:, 2] = 0
        local -= local.mean(axis=0)
        self.samples[label].append(local)
        self.last[label] = now
        if len(self.samples[label]) >= self.target:
            batch = np.asarray(self.samples[label])
            median = np.median(batch, axis=0)
            errors = np.sqrt(np.mean(np.sum((batch - median) ** 2, axis=2), axis=1))
            inliers = errors < 0.006
            if np.count_nonzero(inliers) < 0.8 * self.target:
                # Keep stable samples and collect replacements instead of saving a poor fit.
                self.samples[label] = list(batch[inliers])
                self.status[label] = "unstable geometry; collecting replacements"
                return True
            points = np.median(batch[inliers], axis=0)
            points -= points.mean(axis=0)
            y_axis = points[2] - points[0]
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, [0, 0, 1])
            points = points @ np.column_stack((x_axis, y_axis, [0, 0, 1]))
            self.models[label] = {
                "anchor_ids": list(ANCHOR_IDS), "points_m": points.tolist(),
                "sample_count": int(np.count_nonzero(inliers)),
                "shape_rms_m": float(np.sqrt(np.mean(errors[inliers] ** 2))),
                "palm_width_m": float(np.linalg.norm(points[1] - points[4])),
                "palm_length_m": float(np.linalg.norm(points[2] - points[0])),
            }
            self.status[label] = "complete"
        return True

    @property
    def complete(self):
        return len(self.models) == len(self.samples)

    def progress(self):
        return "  ".join(f"{label[0]}:{len(samples)}/{self.target}" for label, samples in self.samples.items())

    def save(self, path):
        if not self.complete:
            raise ValueError("Calibration incomplete; existing calibration was not changed")
        data = validate_calibration({
            "version": 1, "units": "m", "camera": self.camera, "hands": self.models,
        })
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        try:
            with temporary.open("w", encoding="utf-8") as stream:
                json.dump(data, stream, indent=2, allow_nan=False)
                stream.write("\n")
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        return data


class RGBPalmEstimator:
    """Metric RGB PnP with an optional RGB-model orientation cue for planar ambiguity."""

    def __init__(self, calibration, max_error=4.0):
        self.calibration = validate_calibration(calibration)
        self.matrix = np.asarray(calibration["camera"]["matrix"], dtype=float)
        self.distortion = np.asarray(calibration["camera"]["distortion_coefficients"], dtype=float)
        self.max_error = max_error
        self.previous = {}

    def estimate(self, label, landmarks, now, min_depth=0.2, max_depth=1.5, world_landmarks=None):
        model = self.calibration["hands"].get(label)
        if model is None:
            return None, "hand not calibrated"
        camera = self.calibration["camera"]
        pixels = np.asarray([(p.x * camera["width"], p.y * camera["height"]) for p in landmarks])
        if pixels.shape != (21, 2) or not np.isfinite(pixels).all():
            return None, "invalid landmarks"
        image = np.ascontiguousarray(pixels[list(ANCHOR_IDS)], dtype=np.float64)
        if np.any(image < 0) or np.any(image >= [camera["width"], camera["height"]]):
            return None, "palm outside image"
        lateral, longitudinal = image[1] - image[4], image[2] - image[0]
        area = abs(lateral[0] * longitudinal[1] - lateral[1] * longitudinal[0])
        if (
            cv2.contourArea(cv2.convexHull(image.astype(np.float32))) < 80
            or area < 0.2 * np.linalg.norm(lateral) * np.linalg.norm(longitudinal)
        ):
            return None, "palm too small or edge-on"
        objects = np.asarray(model["points_m"], dtype=np.float64)
        hint = palm_rotation_hint(world_landmarks) if world_landmarks is not None else None
        seeds = []
        # SQPnP also covers IPPE's numerically unstable near-180-degree initializations.
        for method in (cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_IPPE):
            try:
                count, rotations, translations, _ = cv2.solvePnPGeneric(
                    objects, image, self.matrix, self.distortion, flags=method,
                )
            except cv2.error:
                continue
            if count:
                seeds.extend((r.copy(), t.copy(), True) for r, t in zip(rotations, translations))
        if hint is not None:
            # Solve translation at the RGB-only orientation, then let PnP optimize all six DoFs.
            normalized = cv2.undistortPoints(image[:, None, :], self.matrix, self.distortion).reshape(-1, 2)
            rotated = hint.apply(objects)
            system = np.zeros((2 * len(objects), 3))
            system[0::2, 0], system[1::2, 1] = 1, 1
            system[0::2, 2], system[1::2, 2] = -normalized[:, 0], -normalized[:, 1]
            rhs = (normalized * rotated[:, 2, None] - rotated[:, :2]).ravel()
            translation = np.linalg.lstsq(system, rhs, rcond=None)[0]
            seeds.append((hint.as_rotvec().reshape(3, 1), translation.reshape(3, 1), False))
        if not seeds:
            return None, "PnP failed"
        proposals = []
        for rvec, tvec, retain_raw in seeds:
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                continue
            if retain_raw:
                proposals.append((rvec.copy(), tvec.copy()))
            try:
                proposals.append(cv2.solvePnPRefineLM(
                    objects, image, self.matrix, self.distortion, rvec.copy(), tvec.copy(),
                ))
            except cv2.error:
                continue
        candidates = []
        for rvec, tvec in proposals:
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                continue
            rotation = Rotation.from_rotvec(rvec.ravel())
            position = tvec.ravel()
            joints = rotation.apply(objects) + position
            if not np.isfinite(joints).all() or not np.all((joints[:, 2] >= min_depth) & (joints[:, 2] <= max_depth)):
                continue
            projected = cv2.projectPoints(objects, rvec, tvec, self.matrix, self.distortion)[0].reshape(-1, 2)
            errors = np.linalg.norm(projected - image, axis=1)
            rms = float(np.sqrt(np.mean(errors ** 2)))
            if rms > self.max_error or np.max(errors) > 2 * self.max_error:
                continue
            candidates.append((rms, rotation, position, joints))
        if not candidates:
            return None, "PnP reprojection/depth rejected"
        candidates.sort(key=lambda item: item[0])
        distinct = []
        for candidate in candidates:
            if not any(
                (candidate[1].inv() * other[1]).magnitude() < np.deg2rad(1)
                and np.linalg.norm(candidate[2] - other[2]) < 0.001 for other in distinct
            ):
                distinct.append(candidate)
        candidates = distinct
        best = candidates[0]
        # Keep only similarly fitting branches; RGB orientation must not rescue a poor fit.
        limit = min(best[0] + 0.75, max(2.0 * best[0], best[0] + 0.15))
        plausible = [item for item in candidates if item[0] <= limit]
        prior = self.previous.get(label)
        if prior and not 0 <= now - prior[0] < 0.4:
            prior = None
        hint_used = False
        if hint is not None:
            hinted = min(plausible, key=lambda item: (hint.inv() * item[1]).magnitude())
            hinted_angle = (hint.inv() * hinted[1]).magnitude()
            if hinted_angle > np.deg2rad(45):
                return None, "RGB orientation/PnP inconsistent"
            competitors = [item for item in plausible if (hinted[1].inv() * item[1]).magnitude() > np.deg2rad(15)]
            if competitors and min((hint.inv() * item[1]).magnitude() for item in competitors) - hinted_angle > np.deg2rad(10):
                best, hint_used = hinted, True
        if prior and not hint_used:
            best = min(plausible, key=lambda item: (
                item[0] + (prior[1].inv() * item[1]).magnitude()
                + 10 * np.linalg.norm(item[2] - prior[2])
            ))
        if prior and (prior[1].inv() * best[1]).magnitude() > np.deg2rad(60):
            return None, "PnP orientation jump"
        if prior is None and not hint_used and any(
            item[0] - best[0] < 0.15 and (best[1].inv() * item[1]).magnitude() > np.deg2rad(15)
            for item in plausible
        ):
            return None, "ambiguous planar PnP"
        rms, rotation, position, anchors = best
        self.previous[label] = now, rotation, position.copy()
        joints = np.full((21, 3), np.nan)
        joints[list(ANCHOR_IDS)] = anchors
        return {
            "position": position, "orientation": rotation.as_quat(), "joints": joints,
            "quality": float(np.exp(-rms / self.max_error)), "plane_rms_m": None,
            "pose_source": "rgb_pnp", "reprojection_rms_px": rms,
            "pnp_orientation_hint_used": hint_used,
            "pnp_hint_angle_deg": float(np.degrees((hint.inv() * rotation).magnitude())) if hint is not None else None,
            "pnp_candidate_count": len(candidates),
        }, None
