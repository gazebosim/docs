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

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import sys
from pathlib import Path
import yaml


from sphinx.application import Sphinx
from sphinx.config import Config


sys.path.append(str(Path(__file__).parent))
from base_conf import *  # noqa

html_baseurl = f"{html_context['deploy_url']}/docs/latest/"  # noqa

html_context.update({
    "github_user": "gazebosim",
    "github_repo": "docs",
    "github_version": "master",
    "edit_page_url_template": "{{ github_url }}/{{ github_user }}/{{ github_repo }}"
    "/edit/{{ github_version }}/{{ get_file_from_map(file_name) }}",
    "edit_page_provider_name": "GitHub",
})


def setup_file_map(app: Sphinx, pagename: str, templatename: str, context, doctree):
    def get_file_from_map(file_name: str):
        result = context["file_name_map"].get(Path(file_name).stem)
        if result:
            return result
        return file_name

    context["get_file_from_map"] = get_file_from_map


def load_releases(index_file):
    with open(index_file) as top_index_file:
        gz_nav_yaml = yaml.safe_load(top_index_file)

    return dict([(release["name"], release) for release in gz_nav_yaml["releases"]])


def get_preferred_release(releases: dict):
    preferred = [rel for rel in releases.values() if rel.get("preferred", False)]
    assert len(preferred) == 1
    return preferred[0]


def create_file_rename_map(nav_yaml_pages, release):
    file_name_map = {}

    prefix = f"{release}/" if release is not None else ""

    for page in nav_yaml_pages:
        file_name_map[page["name"]] = f"{prefix}{page['file']}"

        children = page.get("children")
        if children:
            file_name_map.update(create_file_rename_map(children, release))

    return file_name_map


def config_init(app: Sphinx, config: Config):
    if not config.gz_release:
        raise RuntimeError("gz_release not provided")
    config.release = config.gz_release  # type: ignore
    config.version = config.gz_release  # type: ignore

    file_name_map = {}

    with open(app.config.gz_root_index_file) as f:
        file_name_map.update(create_file_rename_map(yaml.safe_load(f)["pages"], None))

    with open(Path(app.srcdir) / "index.yaml") as f:
        file_name_map.update(
            create_file_rename_map(yaml.safe_load(f)["pages"], config.release)
        )

    config.html_context["file_name_map"] = file_name_map

    # We've disabled "check_switcher" since it doesn't play well with our directory structure.
    # So we check for the existence of switcher.json here
    #
    assert Path(f"{app.srcdir}/_static/switcher.json").exists()
    config.html_theme_options["switcher"] = {
        "json_url": f"{html_context['deploy_url']}/docs/{config.gz_release}/_static/switcher.json",
        "version_match": config.gz_release,
    }

    try:
        releases = load_releases(config.gz_root_index_file)
        release_info = releases[config.gz_release]
        app.config.html_context["release_info"] = release_info
        app.config.html_context["preferred_release"] = get_preferred_release(releases)
        if release_info.get('eol', False):
            print(f"Disable nitpicky for {release_info['name']}")
            app.config.nitpick_ignore_regex.append(("myst", r".*"))

    except KeyError as e:
        print(e)
        raise RuntimeError(
            f"Provided gz_release '{config.gz_release}' not registered in `index.yaml`"
        )


def setup(app: Sphinx):
    app.add_config_value("gz_release", "", rebuild="env", types=[str])
    app.add_config_value("gz_root_index_file", "", rebuild="env", types=[str])
    app.connect("html-page-context", setup_file_map)
    app.connect("config-inited", config_init)
