"""
Same lightweight profiler as the original single-process script, just
pulled out so both nav_process.py and lidar_process.py can import it.
Each process gets its OWN _profile_stats dict automatically (separate
processes = separate memory), so each prints its own summary
independently — no cross-process aggregation needed for this to be useful.
"""

import time
from collections import defaultdict
from contextlib import contextmanager

from config import PROFILING_ENABLED, LOG_EVERY_N_LOOPS

_profile_stats = defaultdict(lambda: {"count": 0, "total_ms": 0.0, "max_ms": 0.0})
_state_fps_stats = defaultdict(lambda: {"count": 0, "total_dt": 0.0})


@contextmanager
def profile_section(name):
    if not PROFILING_ENABLED:
        yield
        return
    t0 = time.perf_counter()
    try:
        yield
    finally:
        dt_ms = (time.perf_counter() - t0) * 1000.0
        s = _profile_stats[name]
        s["count"] += 1
        s["total_ms"] += dt_ms
        if dt_ms > s["max_ms"]:
            s["max_ms"] = dt_ms


def record_state_fps(state_name, loop_dt_sec):
    if not PROFILING_ENABLED or loop_dt_sec <= 0:
        return
    s = _state_fps_stats[state_name]
    s["count"] += 1
    s["total_dt"] += loop_dt_sec


def print_profile_summary(tag=""):
    if not PROFILING_ENABLED:
        return

    print(f"\n----- PROFILE SUMMARY [{tag}] (last {LOG_EVERY_N_LOOPS} loops) -----")

    if _profile_stats:
        print(f"{'section':32s} {'calls':>6s} {'avg_ms':>9s} {'max_ms':>9s} {'total_ms':>10s}")
        for name, s in sorted(_profile_stats.items(), key=lambda kv: -kv[1]["total_ms"]):
            avg_ms = s["total_ms"] / s["count"] if s["count"] else 0.0
            print(f"{name:32s} {s['count']:6d} {avg_ms:9.2f} {s['max_ms']:9.2f} {s['total_ms']:10.2f}")
    else:
        print("(no timed sections recorded yet)")

    if _state_fps_stats:
        print(f"\n{'state':32s} {'loops':>6s} {'avg_fps':>9s}")
        for name, s in sorted(_state_fps_stats.items(), key=lambda kv: -kv[1]["count"]):
            avg_fps = s["count"] / s["total_dt"] if s["total_dt"] > 0 else 0.0
            print(f"{name:32s} {s['count']:6d} {avg_fps:9.2f}")

    print("--------------------------------------------\n")

    _profile_stats.clear()
    _state_fps_stats.clear()
