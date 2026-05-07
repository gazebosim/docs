#!/bin/sh
# Copyright (C) 2025 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This script uses the commands rdfind and symlinks to detect duplicates and
# replace them with symlinks leaving only one copy from the list of duplicates.

if [[ $# -lt 1 ]]; then
    echo "deduplicate_files <directory>"
    exit 1
fi

(
  cd ${1}
  rdfind -makeresultsfile false -makesymlinks true .
  symlinks -cr .
)

