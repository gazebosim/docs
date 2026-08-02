#!/usr/bin/env python3
"""
Restructure Python API doc artifacts for deployment.

Input structure:
  python-api-docs-harmonic/math7/html/
  python-api-docs-ionic/math8/html/

Output structure:
  math/7/
  math/8/
"""

import argparse
import re
import sys
import shutil
from pathlib import Path


def restructure(input_dir: Path, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    for distro_dir in input_dir.iterdir():
        if not distro_dir.is_dir():
            continue
        for lib_dir in distro_dir.iterdir():
            if not lib_dir.is_dir():
                continue
            html_dir = lib_dir / "html"
            if not html_dir.exists():
                print(f"WARN: no html dir in {lib_dir}, skipping")
                continue
            # lib_dir.name is like "math7" or "math"
            m = re.match(r"([a-z_]+)(\d*)", lib_dir.name)
            if not m:
                continue
            lib_name, version = m.groups()
            if not version:
                version = "latest"
            dest = output_dir / lib_name / version
            dest.mkdir(parents=True, exist_ok=True)
            print(f"{html_dir} -> {dest}")
            shutil.copytree(html_dir, dest, dirs_exist_ok=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", dest="input_dir", required=True)
    parser.add_argument("output_dir")
    args = parser.parse_args()
    restructure(Path(args.input_dir), Path(args.output_dir))


if __name__ == "__main__":
    sys.exit(main())
