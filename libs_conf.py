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

sys.path.append(str(Path(__file__).parent))

# Import the base_conf.py and override settings for /libs
from base_conf import * # noqa

html_baseurl = f"{html_context['deploy_url']}/libs"  # noqa

html_theme_options["use_edit_page_button"] = False
html_theme_options["secondary_sidebar_items"] = []
