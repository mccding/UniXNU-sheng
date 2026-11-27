#!/usr/bin/env python3
"""Run Pad 8 Pro Keys, Backlight, PollPlan, and Pointer host tests against sheng's 21-byte WN8030 format."""
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'kits/sheng' if (ROOT / 'kits/sheng').is_dir() else ROOT / 'package'
with tempfile.TemporaryDirectory() as tmp:
    codec = Path(tmp) / 'Wn8030.o'
    exe = Path(tmp) / 'piano-wn8030-host'
    sanitize = ['-g', '-O1', '-fsanitize=address,undefined']
    subprocess.run(['cc', '-std=c11', *sanitize, '-c', str(PACKAGE / 'Wn8030.c'), '-o', str(codec)], check=True)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', *sanitize,
                    str(PACKAGE / 'tests/piano_wn8030_host.cpp'), str(codec), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
