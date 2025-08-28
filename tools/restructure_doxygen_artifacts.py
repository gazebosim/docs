# Usage
#   /restructure_doxygen_artifacts.py <path_to_artifacts> <output_directory>
#
# Given a directory containing merged doxygen build artifacts, this script reorganizes them so they
# can be deployed to the website
# This assumes the following directory structure:
# api-docs-fortress
#   ignition-utils1
#     doxygen
#       html
#   ignition-math6
#     doxygen
#       html
# api-docs-harmonic
#   gz-utils2
#     doxygen
#       html
#   gz-math7
#     doxygen
#       html
# api-docs-jetty.
#   gz-utils
#     doxygen
#       html
#   gz-math
#     doxygen
#       html
#
# Note that starting Jetty, the directories will not have the version number.
# Therefore, we rely on the index.yaml file at the root of this repo to obtain
# the major version number of each library.
#
#   The result should look like
#   utils
#     1 (content of **/ignition-utils1/doxygen/html)
#     2 (content of **/gz-utils2/doxygen/html)
#   math
#     6 (content of **/ignition-math6/doxygen/html)
#     7 (content of **/gz-math7/doxygen/html)
#
#
# To test, run:
#
# python3 restructure_doxygen_artifacts.py --gen-test-files api-docs
# python3 restructure_doxygen_artifacts.py --input api-docs api-docs-merged

import argparse
import re
import sys
from pathlib import Path
import shutil
import yaml


re_expr = R"(?:ignition|gz)-([a-z_]*)(\d*)"
sdf_re_expr = R"(sdformat)(\d*)"
distro_re_expr = R"api-docs-([a-z]*)"


def copy_library(lib_html, lib_name, lib_version, output_dir: Path):
    output_lib_dir = output_dir / lib_name / lib_version
    output_lib_dir.mkdir(exist_ok=True, parents=True)
    print(f"{lib_html} -> {output_lib_dir}")
    shutil.copytree(lib_html, output_lib_dir, dirs_exist_ok=True)

def build_version_map_from_index(index):
    version_map = {}
    for release in index['releases']:
        name = release['name']
        version_map[name] = {}
        for lib in release['libraries']:
            version_map[name][lib['name']] = lib['version']

    return version_map

def restructure_docs(input_dir : Path, output_dir: Path, version_map):
    output_dir.mkdir(exist_ok=True)
    for api_docs in input_dir.iterdir():
        m = re.match(distro_re_expr, api_docs.name)
        if m:
            distro = m.group(1)
        else:
            raise RuntimeError("Could not determine Gazebo distribution")


        for lib in api_docs.iterdir():
            m = re.match(re_expr, lib.name)
            if not m:
                m = re.match(sdf_re_expr, lib.name)

            if m:
                lib_name, lib_version = m.groups()
                if not lib_version:
                    lib_version = str(version_map[distro][lib_name])
                    print("Could not determine version from library name. Using version from index", lib_version)

                lib_html = lib/"doxygen"/"html"
                copy_library(lib_html, lib_name, lib_version, output_dir)
                # Handle gazebo->sim rename by making a copy into the sim directory
                if lib_name == "gazebo":
                    copy_library(lib_html, "sim", lib_version, output_dir)

def _create_doxygen_dir(output_dir, distro, lib):
    path = (output_dir / f"api-docs-{distro}" /lib/"doxygen"/"html")
    print("Creating ", path)
    path.mkdir(parents=True)
    (path / "index.html").touch()

def gen_test_files(output_dir):
    output_dir.mkdir()
    distro_lib = {
        "fortress": ["ignition-utils1", "ignition-math6"],
        "harmonic": ["gz-utils2", "gz-math7"],
        "ionic": ["gz-cmake4", "gz-sim9"],
        "jetty": ["gz-cmake", "gz-sim"]
    }
    for distro, libs in distro_lib.items():
        for lib in libs:
            _create_doxygen_dir(output_dir, distro, lib)

def main(argv=None):
    # We will assume that this file is in the same directory as documentation
    # sources and conf.py files.
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-i", "--input", dest="input_dir", help="Input directory")
    group.add_argument("--gen-test-files", action="store_true", help="Generate test files")
    parser.add_argument("output_dir", help="Output directory",)
    parser.add_argument("--index", help="index.yaml file to use")

    args = parser.parse_args(argv)

    if args.gen_test_files:
        gen_test_files(Path(args.output_dir))
        return

    if args.index:
        index_path = Path(args.index)
    else:
        index_path = Path(__file__).parent.parent / "index.yaml"
    if not index_path.exists():
        raise RuntimeError("Could not find index.yaml")

    with open(index_path, 'r') as f:
        index = yaml.safe_load(f.read())
        version_map = build_version_map_from_index(index)
        restructure_docs(Path(args.input_dir), Path(args.output_dir), version_map)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
