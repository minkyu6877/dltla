#!/usr/bin/env python3
"""Direction-dependent DW1000 range-bias model using only the Python standard library."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from statistics import fmean, median, pstdev
from typing import Any, Iterable, Sequence


DEFAULT_ANCHORS = ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0))


def wrap_radians(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def tag_from_center(
    center_x: float, center_y: float, heading_rad: float, forward_m: float, left_m: float
) -> tuple[float, float]:
    return (
        center_x + forward_m * math.cos(heading_rad) - left_m * math.sin(heading_rad),
        center_y + forward_m * math.sin(heading_rad) + left_m * math.cos(heading_rad),
    )


def center_from_tag(
    tag_x: float, tag_y: float, heading_rad: float, forward_m: float, left_m: float
) -> tuple[float, float]:
    return (
        tag_x - forward_m * math.cos(heading_rad) + left_m * math.sin(heading_rad),
        tag_y - forward_m * math.sin(heading_rad) - left_m * math.cos(heading_rad),
    )


def solve_position(
    ranges_m: Sequence[float], anchors: Sequence[Sequence[float]] = DEFAULT_ANCHORS
) -> tuple[float, float, float]:
    if len(ranges_m) != 4 or len(anchors) != 4:
        raise ValueError("Four ranges and four anchors are required")
    d0 = float(ranges_m[0])
    x0, y0 = (float(value) for value in anchors[0])
    rows: list[tuple[float, float, float]] = []
    for index in range(1, 4):
        xi, yi = (float(value) for value in anchors[index])
        di = float(ranges_m[index])
        rows.append(
            (
                2.0 * (xi - x0),
                2.0 * (yi - y0),
                d0 * d0 - di * di + xi * xi + yi * yi - x0 * x0 - y0 * y0,
            )
        )
    ata00 = sum(row[0] * row[0] for row in rows)
    ata01 = sum(row[0] * row[1] for row in rows)
    ata11 = sum(row[1] * row[1] for row in rows)
    atb0 = sum(row[0] * row[2] for row in rows)
    atb1 = sum(row[1] * row[2] for row in rows)
    determinant = ata00 * ata11 - ata01 * ata01
    if abs(determinant) < 1e-9:
        raise ValueError("Anchor geometry is singular")
    x = (atb0 * ata11 - atb1 * ata01) / determinant
    y = (ata00 * atb1 - ata01 * atb0) / determinant
    residuals = []
    for measured, anchor in zip(ranges_m, anchors):
        predicted = math.hypot(x - float(anchor[0]), y - float(anchor[1]))
        residuals.append(float(measured) - predicted)
    rmse = math.sqrt(fmean(value * value for value in residuals))
    return x, y, rmse


def relative_bearing(
    tag_x: float, tag_y: float, heading_rad: float, anchor_x: float, anchor_y: float
) -> float:
    world_bearing = math.atan2(anchor_y - tag_y, anchor_x - tag_x)
    return wrap_radians(world_bearing - heading_rad)


def _circular_fill(values: list[float | None]) -> list[float]:
    populated = [index for index, value in enumerate(values) if value is not None]
    if not populated:
        raise ValueError("No samples are available for an anchor")
    result = [0.0] * len(values)
    size = len(values)
    for index, value in enumerate(values):
        if value is not None:
            result[index] = float(value)
            continue
        previous = min(populated, key=lambda other: (index - other) % size)
        following = min(populated, key=lambda other: (other - index) % size)
        span = (following - previous) % size
        if span == 0:
            result[index] = float(values[previous])
        else:
            fraction = ((index - previous) % size) / span
            result[index] = float(values[previous]) * (1.0 - fraction) + float(values[following]) * fraction
    return result


@dataclass(frozen=True)
class DirectionalCorrection:
    tag_x: float
    tag_y: float
    range_rmse: float
    corrected_ranges: tuple[float, float, float, float]
    applied_biases: tuple[float, float, float, float]


class DirectionalUwbModel:
    def __init__(self, document: dict[str, Any]):
        if int(document.get("version", 0)) != 1:
            raise ValueError("Unsupported UWB directional model version")
        self.document = document
        self.anchors = tuple(tuple(float(value) for value in pair) for pair in document["anchors_m"])
        if len(self.anchors) != 4:
            raise ValueError("Directional model must contain four anchors")
        self.bin_width_deg = float(document["bin_width_deg"])
        self.bias_bins = tuple(
            tuple(float(value) for value in anchor["bias_m"]) for anchor in document["anchor_models"]
        )
        if len(self.bias_bins) != 4 or any(not values for values in self.bias_bins):
            raise ValueError("Directional model has invalid bias bins")

    @classmethod
    def load(cls, path: Path) -> "DirectionalUwbModel":
        return cls(json.loads(path.read_text(encoding="utf-8")))

    def bias(self, anchor_index: int, bearing_rad: float) -> float:
        values = self.bias_bins[anchor_index]
        position = (math.degrees(bearing_rad) % 360.0) / self.bin_width_deg
        lower = math.floor(position) % len(values)
        fraction = position - math.floor(position)
        upper = (lower + 1) % len(values)
        return values[lower] * (1.0 - fraction) + values[upper] * fraction

    def correct(
        self,
        measured_ranges: Sequence[float],
        heading_rad: float,
        initial_tag: tuple[float, float],
        iterations: int = 4,
    ) -> DirectionalCorrection:
        if len(measured_ranges) != 4:
            raise ValueError("Four ranges are required")
        x, y = initial_tag
        corrected = tuple(float(value) for value in measured_ranges)
        biases = (0.0, 0.0, 0.0, 0.0)
        for _ in range(max(1, iterations)):
            current_biases = []
            current_ranges = []
            for index, (measured, anchor) in enumerate(zip(measured_ranges, self.anchors)):
                bearing = relative_bearing(x, y, heading_rad, anchor[0], anchor[1])
                bias = self.bias(index, bearing)
                current_biases.append(bias)
                current_ranges.append(max(0.15, float(measured) - bias))
            x, y, rmse = solve_position(current_ranges, self.anchors)
            corrected = tuple(current_ranges)
            biases = tuple(current_biases)
        return DirectionalCorrection(x, y, rmse, corrected, biases)  # type: ignore[arg-type]


def build_directional_model(
    captures: Iterable[dict[str, Any]],
    anchors: Sequence[Sequence[float]] = DEFAULT_ANCHORS,
    tag_forward_m: float = 0.110,
    tag_left_m: float = 0.025,
    bin_width_deg: float = 30.0,
) -> dict[str, Any]:
    captures = list(captures)
    bin_count = int(round(360.0 / bin_width_deg))
    if bin_count < 4 or abs(bin_count * bin_width_deg - 360.0) > 1e-6:
        raise ValueError("bin_width_deg must divide 360 degrees")
    collected: list[list[list[float]]] = [[[] for _ in range(bin_count)] for _ in range(4)]
    prepared: list[dict[str, Any]] = []
    for capture in captures:
        heading = math.radians(float(capture["heading_deg"]))
        center_x = float(capture["center_x"])
        center_y = float(capture["center_y"])
        ranges = tuple(float(value) for value in capture["range_means_m"])
        if len(ranges) != 4:
            raise ValueError("Each capture must contain four mean ranges")
        tag_x, tag_y = tag_from_center(center_x, center_y, heading, tag_forward_m, tag_left_m)
        for index, (measured, anchor) in enumerate(zip(ranges, anchors)):
            expected = math.hypot(tag_x - float(anchor[0]), tag_y - float(anchor[1]))
            bearing = relative_bearing(tag_x, tag_y, heading, float(anchor[0]), float(anchor[1]))
            bin_index = int(round((math.degrees(bearing) % 360.0) / bin_width_deg)) % bin_count
            collected[index][bin_index].append(measured - expected)
        prepared.append({**capture, "expected_tag": (tag_x, tag_y), "ranges": ranges})

    anchor_models = []
    for anchor_index in range(4):
        raw_biases: list[float | None] = []
        counts = []
        standard_deviations = []
        for values in collected[anchor_index]:
            counts.append(len(values))
            raw_biases.append(None if not values else median(values))
            standard_deviations.append(0.0 if len(values) < 2 else pstdev(values))
        filled = _circular_fill(raw_biases)
        anchor_models.append(
            {
                "anchor_id": anchor_index + 1,
                "bias_m": [round(value, 6) for value in filled],
                "sample_counts": counts,
                "sample_std_m": [round(value, 6) for value in standard_deviations],
            }
        )

    document: dict[str, Any] = {
        "version": 1,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "method": "per_anchor_relative_bearing_bias",
        "anchors_m": [[float(value) for value in pair] for pair in anchors],
        "tag_forward_m": tag_forward_m,
        "tag_left_m": tag_left_m,
        "bin_width_deg": bin_width_deg,
        "bin_centers_deg": [index * bin_width_deg for index in range(bin_count)],
        "anchor_models": anchor_models,
    }
    model = DirectionalUwbModel(document)
    raw_errors = []
    corrected_errors = []
    capture_errors = []
    for capture in prepared:
        heading = math.radians(float(capture["heading_deg"]))
        expected_tag = capture["expected_tag"]
        initial_tag = (float(capture["tag_x_mean"]), float(capture["tag_y_mean"]))
        raw_center = center_from_tag(initial_tag[0], initial_tag[1], heading, tag_forward_m, tag_left_m)
        corrected = model.correct(capture["ranges"], heading, initial_tag)
        corrected_center = center_from_tag(corrected.tag_x, corrected.tag_y, heading, tag_forward_m, tag_left_m)
        raw_error = math.hypot(raw_center[0] - float(capture["center_x"]), raw_center[1] - float(capture["center_y"]))
        corrected_error = math.hypot(
            corrected_center[0] - float(capture["center_x"]),
            corrected_center[1] - float(capture["center_y"]),
        )
        raw_errors.append(raw_error)
        corrected_errors.append(corrected_error)
        capture_errors.append(
            {
                "point_id": capture.get("point_id", ""),
                "heading_deg": capture["heading_deg"],
                "raw_center_error_m": round(raw_error, 6),
                "corrected_center_error_m": round(corrected_error, 6),
                "expected_tag_x_m": round(expected_tag[0], 6),
                "expected_tag_y_m": round(expected_tag[1], 6),
            }
        )
    document["training_stats"] = {
        "capture_count": len(prepared),
        "raw_error_mean_m": fmean(raw_errors),
        "raw_error_max_m": max(raw_errors),
        "corrected_error_mean_m": fmean(corrected_errors),
        "corrected_error_max_m": max(corrected_errors),
        "capture_errors": capture_errors,
        "warning": "Training errors are optimistic; validate at locations not used for calibration.",
    }
    return document
