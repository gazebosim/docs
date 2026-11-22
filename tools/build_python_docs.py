#!/usr/bin/env python3
"""
Build Sphinx autodoc documentation for Gazebo Python bindings.
Usage: python3 build_python_docs.py --distro harmonic --output python-api-build
"""

import argparse
import subprocess
import sys
import shutil
from pathlib import Path

# Map distro -> list of (module_name, package_name, lib_short_name)
PYTHON_BINDINGS = {
    "harmonic": [
        ("gz.math7", "gz-math7", "math7"),
    ],
    "ionic": [
        ("gz.math8", "gz-math8", "math8"),
    ],
    "jetty": [
        ("gz.math", "gz-math",  "math"),
    ],
}

CONF_PY_TEMPLATE = """\
project = "{project}"
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]
html_theme = "furo"
autodoc_default_options = {{
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}}
"""

INDEX_RST_TEMPLATE = """\
{project}
{underline}

.. automodule:: {module}
   :members:
   :undoc-members:
   :show-inheritance:
"""


def build_docs(module_name, lib_short_name, output_dir: Path):
    build_dir = output_dir / lib_short_name
    src_dir = build_dir / "src"
    src_dir.mkdir(parents=True, exist_ok=True)

    project = f"Gazebo {lib_short_name} Python API"

    (src_dir / "conf.py").write_text(CONF_PY_TEMPLATE.format(project=project))
    (src_dir / "index.rst").write_text(
        INDEX_RST_TEMPLATE.format(
            project=project,
            underline="=" * len(project),
            module=module_name,
        )
    )

    html_out = build_dir / "html"
    result = subprocess.run(
        ["sphinx-build", "-b", "html", str(src_dir), str(html_out)],
        capture_output=True, text=True
    )
    print(result.stdout)
    if result.returncode != 0:
        print("WARN: sphinx-build failed for", module_name, file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distro", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    bindings = PYTHON_BINDINGS.get(args.distro, [])
    if not bindings:
        print(f"No Python bindings configured for distro: {args.distro}")
        sys.exit(0)

    for module_name, pkg_name, lib_short_name in bindings:
        print(f"Building docs for {module_name}...")
        build_docs(module_name, lib_short_name, output_dir)


if __name__ == "__main__":
    main()
