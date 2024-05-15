# -*- coding: utf-8 -*-
# Copyright (C) 2024 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
from sphinx.cmd.build import main as sphinx_main
import argparse
import os
import copy
import sys
import yaml
import shutil
import json

additional_shared_directories = ["images", "releasing"]


def _combine_nav(common_nav, release_nav):
    combined = copy.deepcopy(common_nav)
    # Release are added after 'get_started'
    for i, item in enumerate(release_nav):
        combined.insert(i + 1, item)
    return combined


def copy_pages(pages, root_src_dir, dst):
    for page in pages:
        full_dst = Path(dst) / page["file"]
        if full_dst.parent != dst:
            full_dst.parent.mkdir(parents=True, exist_ok=True)

        shutil.copy2(root_src_dir / page["file"], full_dst)
        if "children" in page:
            copy_pages(page["children"], root_src_dir, dst)


def generate_sources(gz_nav_yaml, root_src_dir, tmp_dir, gz_release):

    if not gz_release:
        raise RuntimeError("gz_release not provided")
    # Copy release-specific directory
    version_src_dir = Path(root_src_dir) / gz_release

    matching_release = [
        release for release in gz_nav_yaml["releases"] if release["name"] == gz_release
    ]
    if not matching_release:
        raise RuntimeError(
            f"Provided gz_release '{gz_release}' not registered in `index.yaml`"
        )
    elif len(matching_release) > 1:
        raise RuntimeError(f"More than one releases named '{gz_release}' found.")

    release_info = matching_release[0]

    tmp_dir.mkdir(exist_ok=True)
    version_tmp_dir = tmp_dir / gz_release

    shutil.copytree(version_src_dir, version_tmp_dir, dirs_exist_ok=True)
    for dir in ["_static", "_templates"]:
        shutil.copytree(root_src_dir / dir, version_tmp_dir / dir, dirs_exist_ok=True)

    shutil.copy2(root_src_dir / "conf.py", version_tmp_dir)

    for dir in additional_shared_directories:
        shutil.copytree(root_src_dir / dir, version_tmp_dir / dir, dirs_exist_ok=True)

    copy_pages(gz_nav_yaml["pages"], root_src_dir, version_tmp_dir)

    # Write switcher.json file
    switcher = []
    for release in gz_nav_yaml["releases"]:
        name = release["name"].capitalize()
        if release["eol"]:
            name += " (EOL)"
        elif release["lts"]:
            name += " (LTS)"

        switcher.append(
            {
                "name": name,
                "version": release["name"],
                "url": f"/docs/{release['name']}",
            }
        )

    static_dir = version_tmp_dir / "_static"
    static_dir.mkdir(exist_ok=True)
    json.dump(switcher, open(static_dir / "switcher.json", "w"))

    def handle_file_url_rename(file_path, file_url):
        computed_url, ext = os.path.splitext(file_path)
        print("renames:", file_path, file_url)
        if file_url != computed_url:
            new_path = file_url + ext
            # If the file url is inside a directory, we want the new path to end up in the same directory
            print("Moving", version_tmp_dir / file_path, version_tmp_dir / new_path)
            shutil.move(version_tmp_dir / file_path, version_tmp_dir / new_path)
            return new_path
        return file_path

    toc_directives = ["{toctree}", ":hidden:", ":maxdepth: 1", ":titlesonly:"]

    with open(version_tmp_dir / "index.yaml") as f:
        version_nav_yaml = yaml.safe_load(f)
        combined_nav = _combine_nav(gz_nav_yaml["pages"], version_nav_yaml["pages"])

        nav_md = []
        # TODO(azeey) Make this recursive so multiple levels of
        # 'children' can be supported.
        for page in combined_nav:
            file_url = page["name"]
            file_path = page["file"]

            children = page.get("children")
            nav_md.append(f"{page['title']} <{page['name']}>")
            new_file_path = handle_file_url_rename(file_path, file_url)

            if children:
                child_md = []
                for child in children:
                    file_url = child["name"]
                    file_path = child["file"]
                    handle_file_url_rename(file_path, file_url)
                    child_md.append(f"{child['title']} <{file_url}>")

                with open(version_tmp_dir / new_file_path, "a") as ind_f:
                    ind_f.write("```")
                    ind_f.write("\n".join(toc_directives) + "\n")
                    ind_f.writelines("\n".join(child_md) + "\n")
                    ind_f.write("```\n")

        library_reference_nav = "library_reference_nav"
        libraries = release_info["libraries"]
        if libraries:
            nav_md.append(library_reference_nav)
            # Add Library Reference
            with open(version_tmp_dir / f"{library_reference_nav}.md", "w") as ind_f:
                ind_f.write("# Library Reference\n\n")
                ind_f.write("```")
                ind_f.write("{toctree}\n")
                for library in libraries:
                    ind_f.write(
                        f"{library['name']} <https://gazebosim.org/api/{library['name']}/{library['version']}>\n"
                    )
                ind_f.write("```\n\n")

        with open(version_tmp_dir / "index.md", "w") as ind_f:
            ind_f.write(
                """---
myst:
    html_meta:
      "http-equiv=refresh": "0; url=getstarted"
---
"""
            )
            ind_f.write("# Index\n\n")
            ind_f.write("```")
            ind_f.write("\n".join(toc_directives) + "\n")
            ind_f.writelines("\n".join(nav_md) + "\n")
            ind_f.write("```\n\n")


def main(argv=None):
    # We will assume that this file is in the same directory as documentation sources and conf.py files.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--releases",
        metavar="GZ_RELEASES",
        nargs="*",
        help="Names of releases to build. Builds all known releases if empty.",
    )
    args, unknown_args = parser.parse_known_args(argv)

    src_dir = Path(__file__).parent
    index_yaml = src_dir / "index.yaml"
    assert index_yaml.exists()

    with open(index_yaml) as top_index_file:
        gz_nav_yaml = yaml.safe_load(top_index_file)

    if not args.releases:
        args.releases = [release["name"] for release in gz_nav_yaml["releases"]]

    tmp_dir = src_dir / ".tmp"
    for release in args.releases:
        generate_sources(gz_nav_yaml, src_dir, tmp_dir, release)
        build_dir = src_dir / ".build" / "docs" / release
        sphinx_args = [
            "-b",
            "dirhtml",
            f"{tmp_dir/release}",
            f"{build_dir}",
            "-D",
            f"gz_release={release}",
            "-D",
            f"gz_root_index_file={index_yaml}",
            *unknown_args,
        ]
        print(f"sphinx_args: {sphinx_args}")

        sphinx_main(sphinx_args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
