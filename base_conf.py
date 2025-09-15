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

project = "Gazebo"
copyright = "2024, Open Robotics"
author = "Gazebo Team"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "sphinx_copybutton",
    "sphinx_design",
    "sphinxcontrib.googleanalytics",
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
    "attrs_block",
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

nitpicky = True
# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_css_files = ["css/gazebo.css"]
html_favicon = "_static/icon/favicon.ico"

html_theme_options = {
    "header_links_before_dropdown": 4,
    "use_edit_page_button": True,
    "show_toc_level": 1,
    "navigation_with_keys": False,
    "show_prev_next": False,
    "footer_center": ["gz-footer"],
    "footer_start": ["sphinx-version"],
    "secondary_sidebar_items": ["page-toc", "edit-this-page"],
    "navbar_align": "left",
    "navbar_center": ["gz-navbar-nav"],
    "navbar_end": ["navbar-icon-links", "theme-switcher", "fuel_app_link"],
    "pygments_light_style": "tango",
    "pygments_dark_style": "monokai",
    "logo": {
        "image_light": "_static/images/logos/gazebo_horz_pos.svg",
        "image_dark": "_static/images/logos/gazebo_horz_neg.svg",
    },
    "check_switcher": False,
    # We have our own version, so we disable the one from the theme.
    "show_version_warning_banner": False,
}
html_sidebars = {"**": ["gz-sidebar-nav"]}

html_context = {
    "deploy_url":  os.environ.get("GZ_DEPLOY_URL", "")
}

googleanalytics_id = "G-JKS50SX85K"
