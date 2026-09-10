from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any

from lesson12_approach_speed import TofSample
from lesson14_fusion_pipeline import OfflineFusionPipeline


@dataclass(frozen=True)
class TemporalResult:
    valid: bool
    detected: bool
    present: bool
    hit_streak: int
    miss_streak: int
    invalid_streak: int
    reason: str


class TemporalVisionFilter:
    """
    Turn single-frame detections into a stable vision-present decision.

    The rules are the same as lesson08/lesson09: two detections confirm,
    two valid misses clear. Invalid frames do not prove that the obstacle
    disappeared, but two consecutive invalid frames also clear the state.
    """

    def __init__(
        self,
        *,
        confirm_frames: int = 2,
        clear_frames: int = 2,
    ) -> None:
        if confirm_frames < 1 or clear_frames < 1:
            raise ValueError("confirm_frames and clear_frames must be positive")

        self.confirm_frames = confirm_frames
        self.clear_frames = clear_frames
        self._present = False
        self._hit_streak = 0
        self._miss_streak = 0
        self._invalid_streak = 0

    def update(self, *, valid: bool, detected: bool) -> TemporalResult:
        if not valid:
            self._invalid_streak += 1
            self._hit_streak = 0
            self._miss_streak = 0

            if self._present and self._invalid_streak >= self.clear_frames:
                self._present = False
                reason = "invalid_cleared"
            elif self._present:
                reason = "invalid_hold"
            else:
                reason = "invalid_no_present"

        elif detected:
            self._invalid_streak = 0
            self._hit_streak += 1
            self._miss_streak = 0

            if not self._present and self._hit_streak >= self.confirm_frames:
                self._present = True
                reason = "confirmed"
            elif self._present:
                reason = "still_present"
            else:
                reason = "suspect"

        else:
            self._invalid_streak = 0
            self._miss_streak += 1
            self._hit_streak = 0

            if self._present and self._miss_streak >= self.clear_frames:
                self._present = False
                reason = "cleared"
            elif self._present:
                reason = "hold_during_miss"
            else:
                reason = "no_obstacle"

        return TemporalResult(
            valid=valid,
            detected=detected,
            present=self._present,
            hit_streak=self._hit_streak,
            miss_streak=self._miss_streak,
            invalid_streak=self._invalid_streak,
            reason=reason,
        )


class FrameSampler:
    """Select frames by media time, not by source-frame number."""

    def __init__(self, *, target_fps: float) -> None:
        if target_fps < 0:
            raise ValueError("target_fps cannot be negative")
        self._interval_us = (
            int(1_000_000 / target_fps) if target_fps > 0 else None
        )
        self._next_timestamp_us: int | None = None

    def should_process(self, timestamp_us: int) -> bool:
        if self._interval_us is None:
            return True
        if self._next_timestamp_us is None or timestamp_us >= self._next_timestamp_us:
            self._next_timestamp_us = timestamp_us + self._interval_us
            return True
        return False


def make_simulated_tof_samples(
    duration_us: int,
    *,
    mode: str = "approaching",
    interval_us: int = 100_000,
) -> list[TofSample]:
    """
    Create clearly-labelled fake ToF data for recorded-video replay.

    A video file has no ESP32 ToF track. These samples let us test the whole
    decision chain before the camera and ToF streams are connected.
    """

    if duration_us < 0:
        raise ValueError("duration_us cannot be negative")
    if interval_us <= 0:
        raise ValueError("interval_us must be positive")
    if mode not in {"approaching", "near", "far", "invalid", "none"}:
        raise ValueError(f"unknown ToF mode: {mode}")

    if mode == "none":
        return []

    samples: list[TofSample] = []
    timestamp_us = 0
    seq = 0
    while timestamp_us <= duration_us:
        if mode == "approaching":
            range_mm = max(1300, 2200 - int(800 * timestamp_us / 1_000_000))
            status = "ok"
        elif mode == "near":
            range_mm = 1600
            status = "ok"
        elif mode == "far":
            range_mm = 2500
            status = "ok"
        else:
            range_mm = 0
            status = "invalid"

        samples.append(
            TofSample(
                seq=seq,
                timestamp_us=timestamp_us,
                range_mm=range_mm,
                status=status,
            )
        )
        seq += 1
        timestamp_us += interval_us

    return samples


def load_tof_samples(path: Path) -> list[TofSample]:
    """Load real or previously recorded ToF data from a JSONL file."""

    samples: list[TofSample] = []
    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            try:
                item = json.loads(line)
                samples.append(
                    TofSample(
                        seq=int(item["tof_seq"]),
                        timestamp_us=int(item["timestamp_us"]),
                        range_mm=int(item["range_mm"]),
                        status=str(item.get("status", "ok")),
                    )
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                raise ValueError(f"invalid ToF JSONL at line {line_number}: {error}") from error

    return samples


def geometry_from_bbox(
    bbox: tuple[int, int, int, int] | None,
    frame_shape: tuple[int, int],
) -> dict[str, float]:
    height, width = frame_shape
    x0 = int(width * 0.30)
    x1 = int(width * 0.70)
    roi_width = x1 - x0
    roi_area = roi_width * height

    if bbox is None:
        return {
            "area_ratio": 0.0,
            "width_ratio": 0.0,
            "height_ratio": 0.0,
            "bottom_row_ratio": 0.0,
        }

    x, y, w, h = bbox
    return {
        "area_ratio": (w * h) / roi_area,
        "width_ratio": w / width,
        "height_ratio": h / height,
        "bottom_row_ratio": (y + h) / height,
    }


def center_crop_bounds(
    frame_shape: tuple[int, int],
    *,
    output_width: int,
    output_height: int,
) -> tuple[int, int, int, int]:
    """Return x0, x1, y0, y1 for a center crop matching the output ratio."""

    height, width = frame_shape
    if width <= 0 or height <= 0:
        raise ValueError("frame shape is invalid")
    if output_width <= 0 or output_height <= 0:
        raise ValueError("output size is invalid")

    target_ratio = output_width / output_height
    source_ratio = width / height

    if source_ratio > target_ratio:
        cropped_width = int(height * target_ratio)
        x0 = (width - cropped_width) // 2
        return x0, x0 + cropped_width, 0, height

    if source_ratio < target_ratio:
        cropped_height = int(width / target_ratio)
        y0 = (height - cropped_height) // 2
        return 0, width, y0, y0 + cropped_height

    return 0, width, 0, height


def normalize_frame(
    frame: Any,
    *,
    output_width: int = 320,
    output_height: int = 240,
) -> Any:
    """Center-crop to the output aspect ratio, then resize without distortion."""

    import cv2

    x0, x1, y0, y1 = center_crop_bounds(
        (frame.shape[0], frame.shape[1]),
        output_width=output_width,
        output_height=output_height,
    )
    cropped = frame[y0:y1, x0:x1]
    if cropped.shape[1] == output_width and cropped.shape[0] == output_height:
        return cropped
    return cv2.resize(
        cropped,
        (output_width, output_height),
        interpolation=cv2.INTER_AREA,
    )


def read_frame_timestamp_us(
    capture: Any,
    *,
    source_frame_index: int,
    source_fps: float,
    previous_timestamp_us: int | None,
) -> tuple[int, str]:
    """Prefer the video's own timestamp; fall back to frame index / FPS."""

    import cv2

    fallback_us = int(round(source_frame_index / source_fps * 1_000_000))
    position_ms = float(capture.get(cv2.CAP_PROP_POS_MSEC))

    if math.isfinite(position_ms) and position_ms >= 0:
        candidate_us = int(round(position_ms * 1000))
        if previous_timestamp_us is None or candidate_us >= previous_timestamp_us:
            return candidate_us, "video_position"

    timestamp_us = fallback_us
    if previous_timestamp_us is not None and timestamp_us <= previous_timestamp_us:
        timestamp_us = previous_timestamp_us + int(round(1_000_000 / source_fps))
    return timestamp_us, "frame_index_fallback"


def _draw_overlay(
    frame_bgr: Any,
    *,
    detected: bool,
    present: bool,
    candidate_kind: str,
    bbox: tuple[int, int, int, int] | None,
    command: str | None,
    range_mm: int | None,
    speed_mm_s: float | None,
    detect_ms: float,
    rejected_bbox: tuple[int, int, int, int] | None = None,
    show_rejected: bool = False,
) -> None:
    import cv2

    height, width = frame_bgr.shape[:2]
    x0 = int(width * 0.30)
    x1 = int(width * 0.70)
    cv2.rectangle(frame_bgr, (x0, 0), (x1 - 1, height - 1), (0, 255, 255), 2)

    if bbox is not None:
        color = (0, 0, 255) if present else (0, 165, 255)
        thickness = 3 if present else 2
        x, y, w, h = bbox
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), color, thickness)
    elif rejected_bbox is not None and show_rejected:
        x, y, w, h = rejected_bbox
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (255, 0, 0), 1)

    range_text = "-" if range_mm is None else f"{range_mm}mm"
    speed_text = "-" if speed_mm_s is None else f"{int(speed_mm_s)}mm/s"
    command_text = command if command is not None else "DISCARD"
    lines = [
        f"detected={int(detected)} present={int(present)} kind={candidate_kind}",
        f"tof={range_text} speed={speed_text}",
        f"command={command_text} detect={detect_ms:.1f}ms",
    ]
    for index, text in enumerate(lines):
        y = 26 + index * 24
        cv2.rectangle(
            frame_bgr,
            (6, y - 16),
            (min(width - 6, 340), y + 5),
            (0, 0, 0),
            thickness=-1,
        )
        cv2.putText(
            frame_bgr,
            text,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def run_video(args: argparse.Namespace) -> int:
    import cv2
    import numpy as np
    from lesson08_pipeline import detect_one_frame_detailed
    from lesson16_morphology_debug import detect_edge_block_frame
    from lesson17_wall_texture_debug import analyze_wall_texture

    video_path = Path(args.video)
    if not video_path.is_file():
        print(f"Video not found: {video_path}", file=sys.stderr)
        return 2

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        print(f"Cannot open video: {video_path}", file=sys.stderr)
        return 2

    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(source_fps) or source_fps <= 0:
        print("Warning: video FPS is invalid; using 30 FPS as fallback.")
        source_fps = 30.0

    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    duration_us = int(frame_count / source_fps * 1_000_000) if frame_count > 0 else 60_000_000

    if args.tof_mode == "file":
        tof_samples = load_tof_samples(Path(args.tof_jsonl))
    else:
        tof_samples = make_simulated_tof_samples(
            duration_us,
            mode=args.tof_mode,
            interval_us=args.tof_interval_us,
        )

    pipeline = OfflineFusionPipeline()
    for sample in tof_samples:
        pipeline.feed_tof(sample)

    temporal_filter = TemporalVisionFilter(
        confirm_frames=args.confirm_frames,
        clear_frames=args.clear_frames,
    )

    if args.fps < 0:
        raise ValueError("--fps cannot be negative")
    if args.transport_delay_us < 0:
        raise ValueError("--transport-delay-us cannot be negative")
    if args.confirm_frames < 1 or args.clear_frames < 1:
        raise ValueError("--confirm-frames and --clear-frames must be positive")
    if args.canny_low < 0 or args.canny_high <= args.canny_low:
        raise ValueError("--canny-high must be greater than --canny-low")

    frame_sampler = FrameSampler(target_fps=args.fps)

    writer = None
    if args.record:
        record_path = Path(args.record)
        record_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(record_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            args.fps if args.fps > 0 else source_fps,
            (args.process_width, args.process_height),
        )

    debug_writer = None
    if args.debug_record:
        debug_path = Path(args.debug_record)
        debug_path.parent.mkdir(parents=True, exist_ok=True)
        debug_writer = cv2.VideoWriter(
            str(debug_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            args.fps if args.fps > 0 else source_fps,
            (args.process_width, args.process_height),
        )

    jsonl_file = None
    if args.jsonl_output:
        output_path = Path(args.jsonl_output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        jsonl_file = output_path.open("w", encoding="utf-8", newline="\n")

    print(
        f"video={video_path} source_fps={source_fps:.2f} "
        f"target_fps={args.fps if args.fps > 0 else source_fps:.2f} "
        f"tof_mode={args.tof_mode} detector={args.detector} "
        f"wall_branch={int(args.enable_wall_branch)}"
    )
    print(
        "frame  time_s  det pres  range  speed  command  reason"
    )
    print("-" * 92)

    processed_frame_id = 0
    source_frame_index = 0
    source_frames_read = 0
    command_counts: dict[str, int] = {"S:0": 0, "S:1": 0, "S:2": 0, "DISCARD": 0}
    detect_times_ms: list[float] = []
    previous_timestamp_us: int | None = None
    first_timestamp_us: int | None = None
    timestamp_sources: dict[str, int] = {"video_position": 0, "frame_index_fallback": 0}

    while True:
        read_ok, frame = capture.read()
        if not read_ok:
            break

        timestamp_us, timestamp_source = read_frame_timestamp_us(
            capture,
            source_frame_index=source_frame_index,
            source_fps=source_fps,
            previous_timestamp_us=previous_timestamp_us,
        )
        timestamp_sources[timestamp_source] += 1
        previous_timestamp_us = timestamp_us
        if first_timestamp_us is None:
            first_timestamp_us = timestamp_us
        source_frame_index += 1
        source_frames_read += 1

        if not frame_sampler.should_process(timestamp_us):
            continue

        if args.max_frames is not None and processed_frame_id >= args.max_frames:
            break

        frame_valid = frame is not None and frame.ndim in (2, 3) and frame.size > 0
        detected = False
        bbox = None
        rejected_bbox = None
        detection_debug: dict[str, Any] | None = None
        wall_debug: dict[str, Any] | None = None
        debug_edges = None
        detect_ms = 0.0
        candidate_kind = "invalid" if not frame_valid else "none"

        if frame_valid:
            source_height, source_width = frame.shape[:2]
            frame = normalize_frame(
                frame,
                output_width=args.process_width,
                output_height=args.process_height,
            )
            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            start_ns = cv2.getTickCount()
            if args.detector == "edge-block":
                detection = detect_edge_block_frame(
                    gray,
                    canny_low=args.canny_low,
                    canny_high=args.canny_high,
                    include_debug_image=debug_writer is not None,
                )
            else:
                detection = detect_one_frame_detailed(
                    gray,
                    canny_low=args.canny_low,
                    canny_high=args.canny_high,
                    include_debug_image=debug_writer is not None,
                )
            end_ns = cv2.getTickCount()
            detect_ms = (end_ns - start_ns) * 1000.0 / cv2.getTickFrequency()
            detect_times_ms.append(detect_ms)
            detected = bool(detection["detected"])
            bbox = detection["bbox"]
            candidate_kind = "edge_block" if detected and args.detector == "edge-block" else "contour" if detected else "none"
            detection_debug = {
                key: value
                for key, value in detection.items()
                if key != "debug_edges"
            }
            debug_edges = detection.get("debug_edges")
            largest_candidate = detection.get("largest_candidate")
            if bbox is None and largest_candidate is not None:
                rejected_bbox = largest_candidate["bbox"]

            if args.enable_wall_branch and not detected:
                wall_result = analyze_wall_texture(
                    gray,
                    canny_low=args.canny_low,
                    canny_high=args.canny_high,
                    min_brightness=args.wall_min_brightness,
                    max_brightness=args.wall_max_brightness,
                    min_brightness_std=args.wall_min_brightness_std,
                    max_edge_pixel_ratio=args.wall_max_edge_ratio,
                    max_cell_edge_pixel_ratio=args.wall_max_cell_edge_ratio,
                    min_low_texture_cell_ratio=args.wall_min_low_texture_cell_ratio,
                )
                wall_debug = {
                    key: value
                    for key, value in wall_result.items()
                    if key not in {"cells", "debug_edges"}
                }
                if wall_result["candidate_wall"]:
                    detected = True
                    bbox = tuple(wall_result["roi_bbox"])
                    candidate_kind = "low_texture_wall"

        temporal = temporal_filter.update(valid=frame_valid, detected=detected)
        geometry = geometry_from_bbox(
            bbox,
            (frame.shape[0], frame.shape[1]) if frame_valid else (1, 1),
        )
        vision_result = {
            "schema": "vision.v1",
            "frame_id": processed_frame_id,
            "timestamp_us": timestamp_us,
            "valid": temporal.valid,
            "present": temporal.present,
            "area_ratio": geometry["area_ratio"],
            "width_ratio": geometry["width_ratio"],
            "bottom_row_ratio": geometry["bottom_row_ratio"],
            "consecutive_present": temporal.hit_streak,
            "candidate_kind": candidate_kind,
            "reason": temporal.reason,
        }

        # For video replay, receive time is also simulated. It is not a real
        # PC receive time and must not be used in the final online system.
        received_timestamp_us = timestamp_us + args.transport_delay_us
        fusion_result = pipeline.process_vision(
            vision_result,
            received_timestamp_us=received_timestamp_us,
            tof_samples=tof_samples,
        )

        command_key = fusion_result.command if fusion_result.command is not None else "DISCARD"
        command_counts[command_key] += 1

        range_text = "-" if fusion_result.range_mm is None else str(fusion_result.range_mm)
        speed_text = (
            "-"
            if fusion_result.filtered_speed_mm_s is None
            else str(int(fusion_result.filtered_speed_mm_s))
        )
        reason = fusion_result.discard_reason if fusion_result.discarded else fusion_result.reason
        print(
            f"{processed_frame_id:>5d}  {timestamp_us / 1_000_000:>6.2f}  "
            f"{int(detected):>3d} {int(temporal.present):>4d}  "
            f"{range_text:>5s}  {speed_text:>5s}  "
            f"{command_key:<7s}  {reason}"
        )
        if args.print_debug and detection_debug is not None:
            candidate = detection_debug.get("largest_candidate")
            if candidate is None:
                print("        debug: no contour")
            elif args.detector == "edge-block":
                print(
                    "        debug: "
                    f"roi_edges={detection_debug['edge_pixel_ratio']:.4f} "
                    f"components={detection_debug['contour_count']} "
                    f"component_area={candidate['component_area_ratio']:.4f} "
                    f"bbox_area={candidate['bbox_area_ratio']:.4f} "
                    f"fill={candidate['component_fill_ratio']:.4f} "
                    f"bbox_edges={candidate['bbox_edge_pixel_ratio']:.4f} "
                    f"w={candidate['width_ratio']:.3f} "
                    f"h={candidate['height_ratio']:.3f} "
                    f"bottom={candidate['bottom_row_ratio']:.3f} "
                    f"reject={candidate['reject_reason']}"
                )
            else:
                print(
                    "        debug: "
                    f"roi_edges={detection_debug['edge_pixel_ratio']:.4f} "
                    f"contours={detection_debug['contour_count']} "
                    f"contour_area={candidate['contour_area_ratio']:.4f} "
                    f"bbox_area={candidate['bbox_area_ratio']:.4f} "
                    f"fill={candidate['contour_fill_ratio']:.4f} "
                    f"bbox_edges={candidate['bbox_edge_pixel_ratio']:.4f} "
                    f"w={candidate['width_ratio']:.3f} "
                    f"h={candidate['height_ratio']:.3f} "
                    f"bottom={candidate['bottom_row_ratio']:.3f} "
                    f"reject={candidate['reject_reason']}"
                )

        if args.print_debug and wall_debug is not None:
            print(
                "        wall: "
                f"candidate={int(wall_debug['candidate_wall'])} "
                f"brightness={wall_debug['mean_brightness']:.1f} "
                f"std={wall_debug['brightness_std']:.1f} "
                f"edges={wall_debug['edge_pixel_ratio']:.4f} "
                f"low_cells={wall_debug['low_texture_cell_count']}/"
                f"{wall_debug['grid_rows'] * wall_debug['grid_cols']} "
                f"reject={wall_debug['reject_reason']}"
            )

        if (writer is not None or args.display) and frame_valid:
            annotated = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
            _draw_overlay(
                annotated,
                detected=detected,
                present=temporal.present,
                candidate_kind=candidate_kind,
                bbox=bbox,
                command=fusion_result.command,
                range_mm=fusion_result.range_mm,
                speed_mm_s=fusion_result.filtered_speed_mm_s,
                detect_ms=detect_ms,
                rejected_bbox=rejected_bbox,
                show_rejected=args.show_rejected,
            )
            if writer is not None:
                writer.write(annotated)
            if args.display:
                cv2.imshow("lesson15 real vision pipeline", annotated)
                if cv2.waitKey(max(1, args.delay_ms)) & 0xFF == ord("q"):
                    break

        if debug_writer is not None and frame_valid and debug_edges is not None:
            edge_bgr = cv2.cvtColor(debug_edges, cv2.COLOR_GRAY2BGR)
            debug_frame = np.zeros(
                (frame.shape[0], frame.shape[1], 3),
                dtype=np.uint8,
            )
            debug_x0 = int(frame.shape[1] * 0.30)
            debug_x1 = int(frame.shape[1] * 0.70)
            debug_frame[:, debug_x0:debug_x1] = edge_bgr
            cv2.rectangle(
                debug_frame,
                (debug_x0, 0),
                (debug_x1 - 1, frame.shape[0] - 1),
                (0, 255, 255),
                1,
            )
            if rejected_bbox is not None:
                x, y, w, h = rejected_bbox
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 0, 255), 1)
            debug_writer.write(debug_frame)

        if jsonl_file is not None:
            record = {
                "schema": "lesson15_frame.v1",
                "source_frame_index": source_frame_index - 1,
                **fusion_result.to_dict(),
                "detected": detected,
                "candidate_kind": candidate_kind,
                "bbox": list(bbox) if bbox is not None else None,
                "geometry": geometry,
                "detection_debug": detection_debug,
                "wall_debug": wall_debug,
                "detect_ms": detect_ms,
            "tof_mode": args.tof_mode,
            "source_frame": {
                "shape": [source_height, source_width],
                "center_crop_bounds": list(
                    center_crop_bounds(
                        (source_height, source_width),
                        output_width=args.process_width,
                        output_height=args.process_height,
                    )
                ),
            },
            "processed_shape": [frame.shape[0], frame.shape[1]],
        }
            jsonl_file.write(json.dumps(record, ensure_ascii=False) + "\n")

        processed_frame_id += 1

    capture.release()
    if writer is not None:
        writer.release()
    if debug_writer is not None:
        debug_writer.release()
    if jsonl_file is not None:
        jsonl_file.close()
    if args.display:
        cv2.destroyAllWindows()

    average_detect_ms = statistics.mean(detect_times_ms) if detect_times_ms else 0.0
    max_detect_ms = max(detect_times_ms) if detect_times_ms else 0.0
    elapsed_us = max(
        0,
        (previous_timestamp_us or 0) - (first_timestamp_us or 0),
    )
    actual_processed_fps = (
        (processed_frame_id - 1) * 1_000_000 / elapsed_us
        if processed_frame_id > 1 and elapsed_us > 0
        else 0.0
    )
    print("-" * 92)
    print(
        f"source_frames={source_frames_read} processed={processed_frame_id} "
        f"processed_fps={actual_processed_fps:.2f}"
    )
    print(
        f"detect_avg={average_detect_ms:.2f}ms detect_max={max_detect_ms:.2f}ms "
        f"timestamps={timestamp_sources}"
    )
    print(
        f"commands: S:0={command_counts['S:0']} S:1={command_counts['S:1']} "
        f"S:2={command_counts['S:2']} discard={command_counts['DISCARD']}"
    )
    return 0


def check_video_metadata(
    video_path: Path,
    *,
    process_width: int,
    process_height: int,
    target_fps: float,
) -> None:
    """Read a video without running vision detection and check its timing/shape."""

    import cv2

    capture = cv2.VideoCapture(str(video_path))
    assert capture.isOpened(), f"cannot open video: {video_path}"

    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    assert math.isfinite(source_fps) and source_fps > 0, source_fps

    timestamps: list[int] = []
    processed_timestamps: list[int] = []
    sampler = FrameSampler(target_fps=target_fps)
    frame_index = 0
    previous_timestamp: int | None = None

    while frame_index < 600:
        read_ok, frame = capture.read()
        if not read_ok:
            break

        timestamp_us, _ = read_frame_timestamp_us(
            capture,
            source_frame_index=frame_index,
            source_fps=source_fps,
            previous_timestamp_us=previous_timestamp,
        )
        timestamps.append(timestamp_us)
        previous_timestamp = timestamp_us

        if frame is not None and frame.size > 0:
            normalized = normalize_frame(
                frame,
                output_width=process_width,
                output_height=process_height,
            )
            assert normalized.shape[:2] == (process_height, process_width), normalized.shape

        if sampler.should_process(timestamp_us):
            processed_timestamps.append(timestamp_us)

        frame_index += 1

    capture.release()
    assert timestamps == sorted(timestamps), "video timestamps moved backwards"
    assert len(timestamps) > 1, "video did not contain enough frames"

    if target_fps > 0 and len(processed_timestamps) > 1:
        elapsed_us = processed_timestamps[-1] - processed_timestamps[0]
        measured_fps = (len(processed_timestamps) - 1) * 1_000_000 / elapsed_us
        assert target_fps * 0.8 <= measured_fps <= target_fps * 1.2, measured_fps


def run_self_check(
    *,
    video_path: Path | None = None,
    process_width: int = 320,
    process_height: int = 240,
    target_fps: float = 5.0,
) -> None:
    from lesson08_pipeline import detect_one_frame_detailed, make_test_frame
    from lesson16_morphology_debug import (
        detect_edge_block_frame,
        make_edge_block_test_frame,
    )
    from lesson17_wall_texture_debug import (
        analyze_wall_texture,
        make_test_frame as make_wall_test_frame,
    )

    detection = detect_one_frame_detailed(make_test_frame("obstacle"))
    candidate = detection["largest_candidate"]
    assert detection["detected"], detection
    assert candidate is not None, detection
    assert candidate["contour_area_ratio"] == candidate["area_ratio"], candidate
    assert candidate["bbox_area_ratio"] > candidate["contour_area_ratio"], candidate
    assert 0 <= candidate["contour_fill_ratio"] <= 1, candidate
    assert 0 <= candidate["bbox_edge_pixel_ratio"] <= 1, candidate

    edge_detection = detect_edge_block_frame(
        make_edge_block_test_frame("obstacle"),
        canny_low=50,
        canny_high=150,
    )
    assert edge_detection["detected"], edge_detection
    assert edge_detection["selected_candidate"] is not None, edge_detection
    empty_edge_detection = detect_edge_block_frame(
        make_edge_block_test_frame("empty"),
        canny_low=50,
        canny_high=150,
    )
    assert not empty_edge_detection["detected"], empty_edge_detection

    wall_detection = analyze_wall_texture(
        make_wall_test_frame("wall"),
        canny_low=20,
        canny_high=60,
    )
    assert wall_detection["candidate_wall"], wall_detection
    textured_wall_detection = analyze_wall_texture(
        make_wall_test_frame("textured"),
        canny_low=20,
        canny_high=60,
    )
    assert not textured_wall_detection["candidate_wall"], textured_wall_detection

    temporal = TemporalVisionFilter()
    states = []
    for valid, detected in [
        (True, False),
        (True, False),
        (True, True),
        (True, True),
        (True, False),
        (True, False),
    ]:
        states.append(temporal.update(valid=valid, detected=detected).present)
    assert states == [False, False, False, True, True, False], states

    invalid_filter = TemporalVisionFilter()
    invalid_states = []
    for valid, detected in [(True, True), (True, True), (False, False), (False, False)]:
        invalid_states.append(invalid_filter.update(valid=valid, detected=detected).present)
    assert invalid_states == [False, True, True, False], invalid_states

    samples = make_simulated_tof_samples(1_000_000, mode="approaching")
    assert len(samples) == 11
    assert samples[0].range_mm == 2200
    assert samples[-1].range_mm == 1400
    assert all(sample.status == "ok" for sample in samples)
    assert [sample.timestamp_us for sample in samples] == sorted(
        sample.timestamp_us for sample in samples
    )
    assert all(
        left.range_mm > right.range_mm
        for left, right in zip(samples, samples[1:])
        if left.status == "ok" and right.status == "ok"
    )

    landscape_crop = center_crop_bounds(
        (1080, 1920),
        output_width=320,
        output_height=240,
    )
    assert landscape_crop == (240, 1680, 0, 1080), landscape_crop

    portrait_crop = center_crop_bounds(
        (1920, 1080),
        output_width=320,
        output_height=240,
    )
    assert portrait_crop == (0, 1080, 555, 1365), portrait_crop

    geometry = geometry_from_bbox((160, 190, 50, 70), (240, 320))
    assert geometry["area_ratio"] == (50 * 70) / (128 * 240), geometry

    sampler = FrameSampler(target_fps=6.0)
    timestamps_30fps = [round(index * 1_000_000 / 30) for index in range(300)]
    selected = [timestamp for timestamp in timestamps_30fps if sampler.should_process(timestamp)]
    assert 59 <= len(selected) <= 61, len(selected)
    assert selected == sorted(selected), selected

    if video_path is not None:
        check_video_metadata(
            video_path,
            process_width=process_width,
            process_height=process_height,
            target_fps=target_fps,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the offline real-video vision-to-fusion pipeline."
    )
    parser.add_argument("video", nargs="?", help="input video file")
    parser.add_argument("--video", dest="video_option", help="alternative input video option")
    parser.add_argument("--self-check", action="store_true", help="run tests without OpenCV")
    parser.add_argument(
        "--detector",
        choices=("contour", "edge-block"),
        default="contour",
        help="contour is the original closed-contour rule; edge-block is experimental",
    )
    parser.add_argument(
        "--enable-wall-branch",
        action="store_true",
        help="fall back to a low-texture wall candidate when the primary detector misses",
    )
    parser.add_argument("--wall-min-brightness", type=float, default=35.0)
    parser.add_argument("--wall-max-brightness", type=float, default=225.0)
    parser.add_argument("--wall-min-brightness-std", type=float, default=1.5)
    parser.add_argument("--wall-max-edge-ratio", type=float, default=0.003)
    parser.add_argument("--wall-max-cell-edge-ratio", type=float, default=0.006)
    parser.add_argument("--wall-min-low-texture-cell-ratio", type=float, default=0.75)
    parser.add_argument(
        "--tof-mode",
        choices=["approaching", "near", "far", "invalid", "none", "file"],
        default="approaching",
        help="fake ToF source for replay, or file for --tof-jsonl",
    )
    parser.add_argument("--tof-jsonl", help="ToF JSONL file used with --tof-mode file")
    parser.add_argument("--tof-interval-us", type=int, default=100_000)
    parser.add_argument("--canny-low", type=int, default=50)
    parser.add_argument("--canny-high", type=int, default=150)
    parser.add_argument(
        "--fps",
        type=float,
        default=6.0,
        help="processed frames per second; 0 uses every source frame",
    )
    parser.add_argument("--transport-delay-us", type=int, default=100_000)
    parser.add_argument("--process-width", type=int, default=320)
    parser.add_argument("--process-height", type=int, default=240)
    parser.add_argument("--confirm-frames", type=int, default=2)
    parser.add_argument("--clear-frames", type=int, default=2)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--display", dest="display", action="store_true", default=True)
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--delay-ms", type=int, default=50)
    parser.add_argument("--record", help="write annotated MP4 video")
    parser.add_argument("--jsonl-output", help="write per-frame decision records")
    parser.add_argument(
        "--debug-record",
        help="write an MP4 showing the Canny edges used by the detector",
    )
    parser.add_argument(
        "--show-rejected",
        action="store_true",
        help="draw the largest rejected candidate in blue",
    )
    parser.add_argument(
        "--print-debug",
        action="store_true",
        help="print metrics for the largest rejected contour",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.video and args.video_option and args.video != args.video_option:
        parser.error("positional video and --video must refer to the same file")
    if args.video_option:
        args.video = args.video_option

    if args.self_check:
        run_self_check(
            video_path=Path(args.video) if args.video else None,
            process_width=args.process_width,
            process_height=args.process_height,
            target_fps=args.fps,
        )
        print("Self check passed.")
        return 0
    if not args.video:
        parser.print_help()
        return 2
    if args.tof_mode == "file" and not args.tof_jsonl:
        parser.error("--tof-mode file requires --tof-jsonl")
    if not 0 <= args.wall_min_brightness <= args.wall_max_brightness <= 255:
        parser.error("wall brightness must satisfy 0 <= min <= max <= 255")
    if args.wall_min_brightness_std < 0:
        parser.error("--wall-min-brightness-std cannot be negative")
    if not 0 <= args.wall_max_edge_ratio <= 1:
        parser.error("--wall-max-edge-ratio must be between 0 and 1")
    if not 0 <= args.wall_max_cell_edge_ratio <= 1:
        parser.error("--wall-max-cell-edge-ratio must be between 0 and 1")
    if not 0 <= args.wall_min_low_texture_cell_ratio <= 1:
        parser.error("--wall-min-low-texture-cell-ratio must be between 0 and 1")

    return run_video(args)


if __name__ == "__main__":
    raise SystemExit(main())
