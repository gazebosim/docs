# -*- coding: utf-8 -*-
# Copyright (C) 2026 Open Source Robotics Foundation
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

import yaml
from pathlib import Path

_base_dir = Path(__file__).parent

def set_base_dir(base_dir):
    global _base_dir
    _base_dir = Path(base_dir)

def yaml_include(loader, node):
    """Custom constructor for !include tag in YAML."""
    file_path = loader.construct_scalar(node)
    full_path = _base_dir / file_path
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)

def register_yaml_include(base_dir):
    """Register the !include custom constructor with a custom base directory."""
    set_base_dir(base_dir)
    yaml.SafeLoader.add_constructor('!include', yaml_include)

def flatten_navigation(nav_items):
    """Recursively flatten lists in navigation structure caused by !include.

    This only flattens lists of lists created when !include is used as a list item.
    It preserves the intentional tree structure defined by 'children' keys.
    """
    flat_list = []
    for item in nav_items:
        if isinstance(item, list):
            flat_list.extend(flatten_navigation(item))
        else:
            if 'children' in item and item['children']:
                item['children'] = flatten_navigation(item['children'])
            flat_list.append(item)
    return flat_list
