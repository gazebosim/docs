# Usage
#   /restructure_doxygen_artifacts.py <path_to_artifacts> <output_directory>
#
# Given a directory containing merged doxygen build artifacts, this script reorganizes them so they
# can be deployed to the website
# This assumes the following directory structure:
#   ignition-utils1
#     doxygen
#       html
#   gz-utils2
#     doxygen
#       html
#   ignition-math6
#     doxygen
#       html
#   gz-math7
#     doxygen
#       html
#   ...
#
#   The result should look like
#   utils
#     1 (content of ignition-utils1/doxygen/html)
#     2 (content of gz-utils2/doxygen/html)
#   math
#     6 (content of ignition-math6/doxygen/html)
#     7 (content of gz-math7/doxygen/html)

import re
import sys
import pathlib
import shutil

input_dir = pathlib.Path(sys.argv[1])
output_dir = pathlib.Path(sys.argv[2])
output_dir.mkdir(exist_ok=True)

re_expr = R"(ignition|gz)-([a-z_]*)(\d*)"


def copy_library(lib_html, lib_name, lib_version):
    output_lib_dir = output_dir / lib_name / lib_version
    output_lib_dir.mkdir(exist_ok=True, parents=True)
    print(f"{lib_html} -> {output_lib_dir}")
    shutil.copytree(lib_html, output_lib_dir, dirs_exist_ok=True)


for lib in input_dir.iterdir():
    m = re.match(re_expr, lib.name)
    if m:
        _, lib_name, lib_version = m.groups()
        lib_html = lib/"doxygen"/"html"
        copy_library(lib_html, lib_name, lib_version)
        # Handle gazebo->sim rename by making a copy into the sim directory
        if lib_name == "gazebo":
            copy_library(lib_html, "sim", lib_version)
