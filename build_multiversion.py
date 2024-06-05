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
from string import Template
import argparse
import copy
import json
import os
import requests
import shutil
import sys
import yaml

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

    shutil.copy2(root_src_dir / "base_conf.py", version_tmp_dir)
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
        elif release.get("dev", False):
            name += " (dev)"

        switcher.append(
            {
                "name": name,
                "version": release["name"],
                "url": f"/docs/{release['name']}/",
                "preferred": release.get("preferred", False)
            }
        )

    static_dir = version_tmp_dir / "_static"
    static_dir.mkdir(exist_ok=True)
    json.dump(switcher, open(static_dir / "switcher.json", "w"))

    def handle_file_url_rename(file_path, file_url):
        computed_url, ext = os.path.splitext(file_path)
        # print("renames:", file_path, file_url)
        if file_url != computed_url:
            new_path = file_url + ext
            # If the file url is inside a directory, we want the new path to end up in the same directory
            # print("Moving", version_tmp_dir / file_path, version_tmp_dir / new_path)
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


def get_preferred_release(releases: dict):
    preferred = [rel for rel in releases if rel.get("preferred", False)]
    assert len(preferred) == 1
    return preferred[0]


def github_repo_name(lib_name):
    prefix = "gz-" if lib_name != "sdformat" else ""
    return f"{prefix}{lib_name.replace('_','-')}"


def github_branch(repo_name, version):
    return f"{repo_name}{version}" if repo_name != "sdformat" else f"sdf{version}"


def github_url(lib_name):
    return f"https://github.com/gazebosim/{github_repo_name(lib_name)}"


def api_url(lib_name, version):
    if lib_name == "sdformat":
        return "http://sdformat.org/api"
    else:
        return f"https://gazebosim.org/api/{lib_name}/{version}"


def get_github_content(lib_name, version, file_path):
    repo_name = github_repo_name(lib_name)
    branch = github_branch(repo_name, version)
    url = f"https://raw.githubusercontent.com/gazebosim/{repo_name}/{branch}/{file_path}"
    if os.environ.get("SKIP_FETCH_CONTENT", False):
        return f"Skipped fetching context from {url}"

    print(f"fetching {url}")
    result = requests.get(url, allow_redirects=True)
    return result.text


def generate_individual_lib(library, libs_dir):
    lib_name = library["name"]
    version = library["version"]
    cur_lib_dir = libs_dir / lib_name
    cur_lib_dir.mkdir(exist_ok=True)

    template = Template("""\
# $name

{.gz-libs-lists}
- [{material-regular}`code;2em` Source Code]($github_url)
- [{material-regular}`description;2em` API & Tutorials]($api_url)

::::{tab-set}

:::{tab-item} Readme
$readme
:::

:::{tab-item} Changelog
$changelog
:::

::::
    """)

    mapping = {
        "name": lib_name,
        "readme": get_github_content(lib_name, version, "README.md"),
        "changelog": get_github_content(lib_name, version, "Changelog.md"),
        "github_url": github_url(lib_name),
        "api_url": api_url(lib_name, version),
    }
    with open(cur_lib_dir / "index.md", "w") as f:
        f.write(template.substitute(mapping))


def generate_libs(gz_nav_yaml, libs_dir):
    libraries = get_preferred_release(gz_nav_yaml["releases"])["libraries"]
    library_directives = "\n".join([
        f"{library['name'].capitalize()} <{library['name']}/index>"
        for library in libraries
    ])

    index_md_header_template = Template("""\
# Libraries

```{toctree}
:maxdepth: 1
:hidden:
:titlesonly:
$library_directives
```

""")

    library_card_template = Template("""\
:::{card} [$name_cap]($name/index)
:class-card: gz-libs-cards
   - [{material-regular}`fullscreen;2em` Details]($name/index)
   - [{material-regular}`code;2em` Source Code]($github_url)
   - [{material-regular}`description;2em` API & Tutorials]($api_url)
+++,

$description
:::


""")
    with open(libs_dir / "index.md", "w") as f:
        f.write(
            index_md_header_template.substitute(library_directives=library_directives)
        )

        for library in sorted(libraries, key=lambda lib: lib["name"]):
            name = library["name"]
            try:
                description = gz_nav_yaml["library_info"][name]["description"]
            except KeyError as e:
                print(
                    f"Description for library {name} not found."
                    "Make sure there is an entry for it in index.yaml"
                )
                print(e)
                description = ""
            mapping = {
                "name": name,
                "name_cap": name.capitalize(),
                "github_url": github_url(name),
                "api_url": api_url(name, library["version"]),
                "description": description,
            }
            f.write(library_card_template.substitute(mapping))

            generate_individual_lib(library, libs_dir)


def build_libs(gz_nav_yaml, src_dir, tmp_dir, build_dir):
    libs_dir = tmp_dir / "libs"
    libs_dir.mkdir(exist_ok=True)
    shutil.copy2(src_dir / "base_conf.py", libs_dir/"base_conf.py")
    shutil.copy2(src_dir / "libs_conf.py", libs_dir/"conf.py")
    if len(gz_nav_yaml["releases"]) == 0:
        print("No releases found in 'index.yaml'.")
        return

    for dir in ["_static", "_templates"]:
        shutil.copytree(src_dir / dir, libs_dir / dir, dirs_exist_ok=True)

    build_dir = build_dir / "libs"

    generate_libs(gz_nav_yaml, libs_dir)

    sphinx_args = [
        "-b",
        "dirhtml",
        f"{libs_dir}",
        f"{build_dir}",
    ]
    sphinx_main(sphinx_args)


def main(argv=None):
    src_dir = Path(__file__).parent

    # We will assume that this file is in the same directory as documentation
    # sources and conf.py files.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--releases",
        metavar="GZ_RELEASES",
        nargs="*",
        help="Names of releases to build. Builds all known releases if empty.",
    )
    parser.add_argument(
        "--output_dir", default=src_dir / ".build", help="Path to output directory"
    )
    parser.add_argument(
        "--libs", action="store_true", default=False, help="Build /libs page"
    )
    parser.add_argument(
        "--libs_only", action="store_true", default=False, help="Build only /libs page"
    )
    parser.add_argument(
        "--pointers",
        action="store_true",
        default=False,
        help="Build 'latest' and 'all'",
    )

    args, unknown_args = parser.parse_known_args(argv)

    index_yaml = src_dir / "index.yaml"
    assert index_yaml.exists()

    with open(index_yaml) as top_index_file:
        gz_nav_yaml = yaml.safe_load(top_index_file)

    if not args.releases:
        args.releases = [release["name"] for release in gz_nav_yaml["releases"]]

    preferred_release = get_preferred_release(gz_nav_yaml["releases"])
    tmp_dir = src_dir / ".tmp"
    tmp_dir.mkdir(exist_ok=True)
    build_dir = Path(args.output_dir)
    build_dir.mkdir(exist_ok=True)
    if args.libs or args.libs_only:
        build_libs(gz_nav_yaml, src_dir, tmp_dir, build_dir)

    if args.libs_only:
        return

    build_docs_dir = build_dir / "docs"
    for release in args.releases:
        generate_sources(gz_nav_yaml, src_dir, tmp_dir, release)
        release_build_dir = build_docs_dir / release
        sphinx_args = [
            "-b",
            "dirhtml",
            f"{tmp_dir/release}",
            f"{release_build_dir }",
            "-D",
            f"gz_release={release}",
            "-D",
            f"gz_root_index_file={index_yaml}",
            *unknown_args,
        ]
        sphinx_main(sphinx_args)

    # Handle "latest" and "all"
    release = preferred_release["name"]
    if args.pointers and (release in args.releases):
        for pointer in ["latest", "all"]:
            release_build_dir = build_docs_dir / pointer
            pointer_tmp_dir = tmp_dir/pointer
            try:
                pointer_tmp_dir.symlink_to(tmp_dir/release)
            except FileExistsError:
                # It's okay for it to exist, but make sure it's a symlink
                if not pointer_tmp_dir.is_symlink:
                    raise RuntimeError(
                        f"{pointer_tmp_dir} already exists and is not a symlink"
                    )

            sphinx_args = [
                "-b",
                "dirhtml",
                f"{pointer_tmp_dir}",
                f"{release_build_dir}",
                "-D",
                f"gz_release={release}",
                "-D",
                f"gz_root_index_file={index_yaml}",
                *unknown_args,
            ]
            sphinx_main(sphinx_args)

        # Create a redirect to "/latest"
        redirect_page = build_docs_dir / "index.html"
        redirect_page.write_text("""\
<!DOCTYPE html>
<html>
  <head>
    <meta content="0; url=latest" http-equiv="refresh" />
  </head>
</html>
""")


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
