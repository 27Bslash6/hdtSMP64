#!/usr/bin/env python3
"""Remove AVX build variant lines from hdtSMP64.sln."""

import re
import sys
import shutil
from pathlib import Path


def is_avx_config(line):
    # Same pattern as strip_avx_configs.py — remove _NoAVX, _AVX_DEBUG, _AVX2, _AVX512
    # Keep plain _AVX (the unified base configuration)
    return bool(re.search(r"_(NoAVX|AVX_|AVX2|AVX512)", line))


def strip_avx_sln(sln_path):
    path = Path(sln_path)
    lines = path.read_text(encoding="utf-8-sig").splitlines(keepends=True)

    kept = []
    removed = 0
    for line in lines:
        if is_avx_config(line):
            removed += 1
        else:
            kept.append(line)

    shutil.copy2(sln_path, path.with_suffix(".sln.bak"))
    path.write_text("".join(kept), encoding="utf-8")
    print(f"Removed {removed} lines from {sln_path}")


if __name__ == "__main__":
    strip_avx_sln(sys.argv[1] if len(sys.argv) > 1 else "hdtSMP64.sln")
