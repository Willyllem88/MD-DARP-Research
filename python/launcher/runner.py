"""
runner.py -- Subprocess helpers for solver, instance generator, and result viewer.
"""

import os
import sys
import subprocess
import threading
from typing import Optional, Callable, List


def _ensure_tmp(tmp_dir: str) -> None:
    os.makedirs(tmp_dir, exist_ok=True)


# ── Instance Generator ─────────────────────────────────────────────────────────

def run_generator(
    generator_main: str,
    output_path: str,
    on_exit: Optional[Callable[[int], None]] = None,
) -> subprocess.Popen:
    """
    Launch the instance generator as a separate process.
    Passes -o <output_path> so the file is saved without a GUI dialog.
    Returns the Popen object immediately (non-blocking).
    """
    _ensure_tmp(os.path.dirname(output_path))
    cmd = [sys.executable, generator_main, "-o", output_path]
    proc = subprocess.Popen(cmd)

    if on_exit:
        def _watch():
            proc.wait()
            on_exit(proc.returncode)
        threading.Thread(target=_watch, daemon=True).start()

    return proc


# ── Solver ─────────────────────────────────────────────────────────────────────

def build_solver_cmd(
    solver_bin: str,
    instance_path: str,
    output_path: str,
    method: str,
    time_limit: Optional[float],
    seed: int,
    verbose: bool,
    enable_gice: bool,
    enable_nr: bool,
) -> List[str]:
    cmd = [
        solver_bin,
        "-i", instance_path,
        "-o", output_path,
        "-m", method,
        "-s", str(seed),
    ]
    if time_limit is not None:
        cmd += ["-t", str(time_limit)]
    if verbose:
        cmd.append("-v")
    if enable_gice:
        cmd.append("--GICE")
    if enable_nr:
        cmd.append("--NR")
    return cmd


def run_solver(
    solver_bin: str,
    instance_path: str,
    output_path: str,
    method: str,
    time_limit: Optional[float],
    seed: int,
    verbose: bool,
    enable_gice: bool,
    enable_nr: bool,
    on_stdout_line: Optional[Callable[[str], None]] = None,
    on_finish: Optional[Callable[[int, str], None]] = None,
) -> subprocess.Popen:
    """
    Launch the solver subprocess.  stdout/stderr are streamed line-by-line via
    on_stdout_line callback (called from a background thread).
    on_finish(returncode, output_path) is called when the process ends.
    Returns the Popen object immediately (non-blocking).
    """
    _ensure_tmp(os.path.dirname(output_path))
    cmd = build_solver_cmd(
        solver_bin, instance_path, output_path,
        method, time_limit, seed, verbose,
        enable_gice, enable_nr,
    )

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def _stream():
        for line in proc.stdout:
            if on_stdout_line:
                on_stdout_line(line.rstrip())
        proc.wait()
        if on_finish:
            on_finish(proc.returncode, output_path)

    threading.Thread(target=_stream, daemon=True).start()
    return proc


# ── Result Viewer ──────────────────────────────────────────────────────────────

def run_viewer(
    viewer_main: str,
    result_path: str,
    mode: str = "streets",
    on_exit: Optional[Callable[[int], None]] = None,
) -> subprocess.Popen:
    """
    Launch the result viewer for a given JSON result file.
    mode: 'direct' or 'streets'
    """
    cmd = [sys.executable, viewer_main, "-l", result_path, "-m", mode]
    proc = subprocess.Popen(cmd)

    if on_exit:
        def _watch():
            proc.wait()
            on_exit(proc.returncode)
        threading.Thread(target=_watch, daemon=True).start()

    return proc
