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
import os
from pathlib import Path
import yaml


from sphinx.application import Sphinx
from sphinx.config import Config

project = "Gazebo"
copyright = "2024, Open Robotics"
author = "Gazebo Team"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "sphinx_copybutton",
    # 'sphinx_sitemap',
]

templates_path = ["_templates"]

source_suffix = [
    ".md",
]

myst_heading_anchors = 4

myst_enable_extensions = [
    "amsmath",
    "attrs_inline",
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "linkify",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

# TODO(azeey) Setting this to true hides a lot of broken links. Consider using
# the `attrs_inline` myst extension and handle external links case by case
# instead of globally.
# myst_all_links_external = True

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_style = "css/gazebo.css"
html_theme_options = {
    "header_links_before_dropdown": 4,
    # "use_edit_page_button": True,
    "show_toc_level": 1,
    "navigation_with_keys": False,
    "show_prev_next": False,
    "footer_start": [],
    "footer_end": [],
    "secondary_sidebar_items": ["page-toc", "edit-this-page"],
    "navbar_align": "left",
    "navbar_center": ["gz-navbar-nav"],
    "navbar_end": ["navbar-icon-links", "theme-switcher", "fuel_app_link"],
    "pygment_light_style": "sphinx",
    "pygment_dark_style": "monokai",
    "logo": {
        "image_light": "_static/images/logos/gazebo_horz_pos.svg",
        "image_dark": "_static/images/logos/gazebo_horz_neg.svg",
    },
    "check_switcher": False,
}

html_sidebars = {"**": ["gz-sidebar-nav"]}
html_baseurl = os.environ.get(
    "SPHINX_HTML_BASE_URL", "http://localhost:8000/docs/latest/"
)


def load_releases(index_file):
    with open(index_file) as top_index_file:
        gz_nav_yaml = yaml.safe_load(top_index_file)

    return dict([(release["name"], release) for release in gz_nav_yaml["releases"]])


def config_init(app: Sphinx, config: Config):
    if not config.gz_release:
        raise RuntimeError("gz_release not provided")
    config.release = config.gz_release  # type: ignore
    config.version = config.gz_release  # type: ignore

    # We've disabled "check_switcher" since it doesn't play well with our directory structure.
    # So we check for the existence of switcher.json here
    #
    assert Path(f"{app.srcdir}/_static/switcher.json").exists()
    config.html_theme_options["switcher"] = {
        "json_url": f"docs/{config.gz_release}/_static/switcher.json",
        "version_match": config.gz_release,
    }

    try:
        releases = load_releases(config.gz_root_index_file)
        app.config.html_context["release_info"] = releases[config.gz_release]
    except KeyError as e:
        print(e)
        raise RuntimeError(
            f"Provided gz_release '{config.gz_release}' not registered in `index.yaml`"
        )

def setup(app: Sphinx):
    app.add_config_value("gz_release", "", rebuild="env", types=[str])
    app.add_config_value("gz_root_index_file", "", rebuild="env", types=[str])
    app.connect("config-inited", config_init)
