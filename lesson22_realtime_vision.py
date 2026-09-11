from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from lesson12_approach_speed import TofSample
from lesson14_fusion_pipeline import OfflineFusionPipeline
from lesson15_real_vision_pipeline import (
    TemporalVisionFilter,
    geometry_from_bbox,
)
from lesson16_morphology_debug import (
    detect_edge_block_frame,
    make_edge_block_test_frame,
)
from lesson17_wall_texture_debug import (
    analyze_wall_texture,
    make_test_frame as make_wall_test_frame,
)
from lesson21_esp32_camera_client import FrameClient, percentile


def draw_overlay(
    frame_bgr: Any,
    *,
    detected: bool,
    present: bool,
    candidate_kind: str,
    bbox: tuple[int, int, int, int] | None,
    command: str,
    reason: str,
    request_ms: float,
    detect_ms: float,
    end_to_end_ms: float,
    rejected_bbox: tuple[int, int, int, int] | None = None,
    show_rejected: bool = False,
) -> None:
    height, width = frame_bgr.shape[:2]
    x0 = int(width * 0.30)
    x1 = int(width * 0.70)
    cv2.rectangle(frame_bgr, (x0, 0), (x1 - 1, height - 1), (255, 255, 0), 2)

    if bbox is not None:
        color = (0, 0, 255) if present else (0, 165, 255)
        thickness = 3 if present else 2
        x, y, w, h = bbox
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), color, thickness)
    elif rejected_bbox is not None and show_rejected:
        x, y, w, h = rejected_bbox
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (255, 0, 0), 1)

    lines = [
        f"det={int(detected)} pres={int(present)} kind={candidate_kind}",
        f"cmd={command} reason={reason}",
        f"req={request_ms:.1f}ms det={detect_ms:.1f}ms total={end_to_end_ms:.1f}ms",
    ]
    for row, text in enumerate(lines):
        y = 24 + row * 24
        cv2.rectangle(frame_bgr, (6, y - 17), (min(width - 6, 460), y + 5), (0, 0, 0), -1)
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


def make_fake_tof_samples(
    duration_s: float,
    *,
    mode: str,
    interval_us: int,
    base_timestamp_us: int = 0,
) -> list[TofSample]:
    """Generate labelled synthetic ToF only for a pre-hardware visual demo."""

    if mode == "none":
        return []
    if mode not in {"far", "near", "approaching", "invalid"}:
        raise ValueError(f"unknown ToF mode: {mode}")

    samples: list[TofSample] = []
    seq = 0
    timestamp_us = 0
    duration_us = int(duration_s * 1_000_000)
    while timestamp_us <= duration_us:
        if mode == "far":
            range_mm = 2500
            status = "ok"
        elif mode == "near":
            range_mm = 1600
            status = "ok"
        elif mode == "invalid":
            range_mm = 0
            status = "invalid"
        else:
            range_mm = max(1300, 2200 - int(800 * timestamp_us / 1_000_000))
            status = "ok"

        samples.append(
            TofSample(
                seq=seq,
                timestamp_us=base_timestamp_us + timestamp_us,
                range_mm=range_mm,
                status=status,
            )
        )
        seq += 1
        timestamp_us += interval_us
    return samples


def run_self_check() -> None:
    detection = detect_edge_block_frame(
        make_edge_block_test_frame("obstacle"),
        canny_low=50,
        canny_high=150,
    )
    assert detection["detected"], detection
    assert detection["selected_candidate"] is not None, detection

    empty_detection = detect_edge_block_frame(
        make_edge_block_test_frame("empty"),
        canny_low=50,
        canny_high=150,
    )
    assert not empty_detection["detected"], empty_detection

    wall_detection = analyze_wall_texture(
        make_wall_test_frame("wall"),
        canny_low=20,
        canny_high=60,
    )
    assert wall_detection["candidate_wall"], wall_detection

    temporal = TemporalVisionFilter()
    states = [
        temporal.update(valid=True, detected=bool(value)).present
        for value in (False, False, True, True, False, False)
    ]
    assert states == [False, False, False, True, True, False], states


def run_realtime(args: argparse.Namespace) -> int:
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.canny_low < 0 or args.canny_high <= args.canny_low:
        raise ValueError("--canny-high must be greater than --canny-low")
    if args.confirm_frames < 1 or args.clear_frames < 1:
        raise ValueError("--confirm-frames and --clear-frames must be positive")

    interval_s = 1.0 / args.fps
    interval_ns = int(interval_s * 1_000_000_000)
    timeout_s = max(0.20, interval_s * 4)
    tof_base_timestamp_us = time.perf_counter_ns() // 1000

    duration_s = args.duration_s
    if args.tof_mode == "approaching":
        fake_tof = make_fake_tof_samples(
            duration_s if duration_s is not None else 60.0,
            mode=args.tof_mode,
            interval_us=args.tof_interval_us,
            base_timestamp_us=tof_base_timestamp_us,
        )
    elif args.tof_mode == "none":
        fake_tof = []
    elif args.tof_mode == "far":
        fake_tof = make_fake_tof_samples(
            duration_s if duration_s is not None else 3600.0,
            mode=args.tof_mode,
            interval_us=args.tof_interval_us,
            base_timestamp_us=tof_base_timestamp_us,
        )
    else:
        fake_tof = make_fake_tof_samples(
            duration_s if duration_s is not None else 3600.0,
            mode=args.tof_mode,
            interval_us=args.tof_interval_us,
            base_timestamp_us=tof_base_timestamp_us,
        )

    pipeline = OfflineFusionPipeline(max_image_age_us=args.max_image_age_us)
    for sample in fake_tof:
        pipeline.feed_tof(sample)

    temporal = TemporalVisionFilter(
        confirm_frames=args.confirm_frames,
        clear_frames=args.clear_frames,
    )
    client = FrameClient(args.url, "keepalive")

    if args.display:
        cv2.namedWindow("ESP32-S3 realtime vision", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ESP32-S3 realtime vision", 960, 720)

    writer = None
    if args.record:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(args.record),
            cv2.VideoWriter_fourcc(*"mp4v"),
            args.fps,
            (args.process_width, args.process_height),
        )

    jsonl_file = None
    if args.jsonl_output:
        args.jsonl_output.parent.mkdir(parents=True, exist_ok=True)
        jsonl_file = args.jsonl_output.open("w", encoding="utf-8", newline="\n")

    print(f"camera={args.url}/capture target_fps={args.fps:.2f} tof_mode={args.tof_mode}")
    print("Press q or Esc in the image window to stop.")

    attempted = 0
    valid_frames = 0
    detected_frames = 0
    present_frames = 0
    failed_frames = 0
    command_counts = {"S:0": 0, "S:1": 0, "S:2": 0, "DISCARD": 0}
    request_times_ms: list[float] = []
    decode_times_ms: list[float] = []
    detect_times_ms: list[float] = []
    end_to_end_times_ms: list[float] = []

    start_ns = time.perf_counter_ns()
    next_due_ns = start_ns
    last_report_ns = start_ns
    report_fps = 0.0
    report_count = 0
    tof_match_window_us = 50_000

    try:
        while True:
            now_ns = time.perf_counter_ns()
            elapsed_s = (now_ns - start_ns) / 1_000_000_000
            if args.max_frames is not None and attempted >= args.max_frames:
                break
            if duration_s is not None and elapsed_s >= duration_s:
                break

            if now_ns < next_due_ns:
                if args.display:
                    wait_ms = max(1, int((next_due_ns - now_ns) / 1_000_000))
                    if cv2.waitKey(wait_ms) & 0xFF in (ord("q"), 27):
                        break
                else:
                    time.sleep((next_due_ns - now_ns) / 1_000_000_000)
                continue

            next_due_ns += interval_ns
            if time.perf_counter_ns() > next_due_ns + interval_ns:
                next_due_ns = time.perf_counter_ns() + interval_ns

            attempted += 1
            requested_ns = time.perf_counter_ns()
            try:
                jpeg_bytes, request_ms, status = client.capture(timeout_s)
                if status != 200:
                    raise RuntimeError(f"HTTP {status}")
                decode_start_ns = time.perf_counter_ns()
                decoded = cv2.imdecode(
                    np.frombuffer(jpeg_bytes, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                decode_ms = (time.perf_counter_ns() - decode_start_ns) / 1_000_000
                if decoded is None:
                    raise RuntimeError("JPEG decode failed")
            except Exception as error:
                failed_frames += 1
                request_ms = (time.perf_counter_ns() - requested_ns) / 1_000_000
                temporal.update(valid=False, detected=False)
                print(f"frame={attempted:5d} FAILED request={request_ms:.1f}ms {error}")

                if jsonl_file is not None:
                    record = {
                        "schema": "lesson22_realtime.v1",
                        "frame_id": attempted,
                        "valid": False,
                        "local_time_s": time.time(),
                        "request_ms": round(request_ms, 3),
                        "failed": True,
                    }
                    jsonl_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                    jsonl_file.flush()
                continue

            frame = decoded
            if (frame.shape[1], frame.shape[0]) != (args.process_width, args.process_height):
                frame = cv2.resize(
                    frame,
                    (args.process_width, args.process_height),
                    interpolation=cv2.INTER_AREA,
                )
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detect_start_ns = time.perf_counter_ns()
            detection = detect_edge_block_frame(
                gray,
                canny_low=args.canny_low,
                canny_high=args.canny_high,
                kernel_size=args.kernel_size,
                iterations=args.morph_iterations,
            )
            detected = bool(detection["detected"])
            bbox = detection.get("bbox")
            candidate_kind = "edge_block" if detected else "none"
            largest = detection.get("largest_candidate")
            rejected_bbox = largest.get("bbox") if largest is not None else None

            if args.enable_wall_branch and not detected:
                wall = analyze_wall_texture(
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
                if wall["candidate_wall"]:
                    detected = True
                    bbox = tuple(wall["roi_bbox"])
                    candidate_kind = "low_texture_wall"

            detect_ms = (time.perf_counter_ns() - detect_start_ns) / 1_000_000
            temporal_result = temporal.update(valid=True, detected=detected)
            geometry = geometry_from_bbox(bbox, (gray.shape[0], gray.shape[1]))

            # The current ESP32 HTTP response has no hardware capture timestamp.
            # Until we add one, use monotonic receive time minus measured request
            # time as a clearly-labelled approximation of capture time.
            now_us = time.perf_counter_ns() // 1000
            capture_timestamp_us = now_us - int(round(request_ms * 1000 + decode_ms * 1000))
            vision_result = {
                "schema": "vision.v1",
                "frame_id": attempted,
                "timestamp_us": capture_timestamp_us,
                "valid": True,
                "present": temporal_result.present,
                "area_ratio": geometry["area_ratio"],
                "width_ratio": geometry["width_ratio"],
                "bottom_row_ratio": geometry["bottom_row_ratio"],
                "consecutive_present": temporal_result.hit_streak,
                "candidate_kind": candidate_kind,
                "reason": temporal_result.reason,
            }
            fusion_result = pipeline.process_vision(
                vision_result,
                received_timestamp_us=now_us,
                tof_samples=[
                    sample
                    for sample in fake_tof
                    if abs(sample.timestamp_us - capture_timestamp_us) <= tof_match_window_us
                ],
            )

            command = fusion_result.command or "DISCARD"
            command_counts[command] = command_counts.get(command, 0) + 1
            valid_frames += 1
            detected_frames += int(detected)
            present_frames += int(temporal_result.present)
            request_times_ms.append(request_ms)
            decode_times_ms.append(decode_ms)
            detect_times_ms.append(detect_ms)
            end_to_end_ms = request_ms + decode_ms + detect_ms
            end_to_end_times_ms.append(end_to_end_ms)

            report_count += 1
            report_ns = time.perf_counter_ns()
            if report_ns - last_report_ns >= 1_000_000_000:
                report_fps = report_count / ((report_ns - last_report_ns) / 1_000_000_000)
                report_count = 0
                last_report_ns = report_ns

            if args.print_frames:
                print(
                    f"frame={attempted:5d} fps={report_fps:4.2f} "
                    f"det={int(detected)} pres={int(temporal_result.present)} "
                    f"kind={candidate_kind:<16s} cmd={command:4s} "
                    f"req={request_ms:6.1f}ms det={detect_ms:5.1f}ms"
                )

            if args.display or writer is not None:
                draw_overlay(
                    frame,
                    detected=detected,
                    present=temporal_result.present,
                    candidate_kind=candidate_kind,
                    bbox=bbox,
                    command=command,
                    reason=fusion_result.reason,
                    request_ms=request_ms,
                    detect_ms=detect_ms,
                    end_to_end_ms=end_to_end_ms,
                    rejected_bbox=rejected_bbox,
                    show_rejected=args.show_rejected,
                )
                if writer is not None:
                    writer.write(frame)
                if args.display:
                    cv2.imshow("ESP32-S3 realtime vision", frame)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break

            if jsonl_file is not None:
                record = {
                    "schema": "lesson22_realtime.v1",
                    "frame_id": attempted,
                    "valid": True,
                    "local_time_s": time.time(),
                    "timestamp_us": capture_timestamp_us,
                    "detected": detected,
                    "present": temporal_result.present,
                    "candidate_kind": candidate_kind,
                    "command": command,
                    "state": fusion_result.state.value if fusion_result.state else None,
                    "reason": fusion_result.reason,
                    "request_ms": round(request_ms, 3),
                    "decode_ms": round(decode_ms, 3),
                    "detect_ms": round(detect_ms, 3),
                    "end_to_end_ms": round(end_to_end_ms, 3),
                    "bbox": list(bbox) if bbox is not None else None,
                    "geometry": geometry,
                }
                jsonl_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                jsonl_file.flush()

    finally:
        client.close()
        if writer is not None:
            writer.release()
        if jsonl_file is not None:
            jsonl_file.close()
        if args.display:
            cv2.destroyAllWindows()

    elapsed_s = (time.perf_counter_ns() - start_ns) / 1_000_000_000
    actual_fps = valid_frames / elapsed_s if elapsed_s > 0 else 0.0
    average_request = statistics.mean(request_times_ms) if request_times_ms else 0.0
    p95_request = percentile(request_times_ms, 0.95) if request_times_ms else 0.0
    max_request = max(request_times_ms) if request_times_ms else 0.0
    average_detect = statistics.mean(detect_times_ms) if detect_times_ms else 0.0
    max_detect = max(detect_times_ms) if detect_times_ms else 0.0
    average_total = statistics.mean(end_to_end_times_ms) if end_to_end_times_ms else 0.0
    max_total = max(end_to_end_times_ms) if end_to_end_times_ms else 0.0

    print("-" * 84)
    print(
        f"attempted={attempted} valid={valid_frames} failed={failed_frames} "
        f"elapsed={elapsed_s:.2f}s actual_fps={actual_fps:.2f}"
    )
    print(
        f"detected={detected_frames} present={present_frames} "
        f"commands={command_counts}"
    )
    print(
        f"request_avg={average_request:.1f}ms request_p95={p95_request:.1f}ms "
        f"request_max={max_request:.1f}ms"
    )
    print(
        f"detect_avg={average_detect:.1f}ms detect_max={max_detect:.1f}ms "
        f"total_avg={average_total:.1f}ms total_max={max_total:.1f}ms"
    )

    args.summary_output.parent.mkdir(parents=True, exist_ok=True)
    args.summary_output.write_text(
        json.dumps(
            {
                "url": args.url,
                "target_fps": args.fps,
                "elapsed_s": round(elapsed_s, 3),
                "attempted": attempted,
                "valid": valid_frames,
                "failed": failed_frames,
                "actual_fps": round(actual_fps, 3),
                "detected_frames": detected_frames,
                "present_frames": present_frames,
                "commands": command_counts,
                "request_avg_ms": round(average_request, 3),
                "request_p95_ms": round(p95_request, 3),
                "request_max_ms": round(max_request, 3),
                "detect_avg_ms": round(average_detect, 3),
                "detect_max_ms": round(max_detect, 3),
                "total_avg_ms": round(average_total, 3),
                "total_max_ms": round(max_total, 3),
            },
            ensure_ascii=False,
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    print(f"summary JSON: {args.summary_output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run realtime ESP32-S3 JPEG capture through the existing vision pipeline.",
    )
    parser.add_argument("--url", default="http://192.168.49.17")
    parser.add_argument("--fps", type=float, default=6.0)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--detector", choices=("edge-block",), default="edge-block")
    parser.add_argument("--canny-low", type=int, default=20)
    parser.add_argument("--canny-high", type=int, default=60)
    parser.add_argument("--kernel-size", type=int, default=3)
    parser.add_argument("--morph-iterations", type=int, default=1)
    parser.add_argument("--enable-wall-branch", action="store_true")
    parser.add_argument("--wall-min-brightness", type=float, default=35.0)
    parser.add_argument("--wall-max-brightness", type=float, default=225.0)
    parser.add_argument("--wall-min-brightness-std", type=float, default=1.5)
    parser.add_argument("--wall-max-edge-ratio", type=float, default=0.003)
    parser.add_argument("--wall-max-cell-edge-ratio", type=float, default=0.006)
    parser.add_argument("--wall-min-low-texture-cell-ratio", type=float, default=0.75)
    parser.add_argument("--confirm-frames", type=int, default=2)
    parser.add_argument("--clear-frames", type=int, default=2)
    parser.add_argument("--max-image-age-us", type=int, default=300_000)
    parser.add_argument("--tof-mode", choices=("none", "far", "near", "approaching", "invalid"), default="none")
    parser.add_argument("--tof-interval-us", type=int, default=100_000)
    parser.add_argument("--process-width", type=int, default=320)
    parser.add_argument("--process-height", type=int, default=240)
    parser.add_argument("--display", action="store_true", default=True)
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--print-frames", action="store_true")
    parser.add_argument(
        "--show-rejected",
        action="store_true",
        help="draw the largest rejected edge-block candidate in blue",
    )
    parser.add_argument("--record", type=Path)
    parser.add_argument("--jsonl-output", type=Path, default=Path("output/lesson22_realtime.jsonl"))
    parser.add_argument("--summary-output", type=Path, default=Path("output/lesson22_realtime_summary.json"))
    parser.add_argument("--self-check", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.self_check:
        run_self_check()
        print("Self check passed.")
        return 0
    try:
        return run_realtime(args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
