#!/bin/bash
# Command line parameters:
# 1 - GitHub organization name. For example ignitionrobotics or osrf.
# 2 - the name of the ignition repository. For example ign-math.
# 3 - the name of the branch. For example ign-math6
# 4 - 'y' or 'n' without the quotes that indicate whether or not to upload docs
# 5 - Release date in the ISO 8601 format. See the command `date -Iseconds`.
# 6 - Password to https://api.ignitionrobotics.org/1.0/versions.

set -o verbose

export DEBIAN_FRONTEND=noninteractive

echo -e "\e[46m\e[30mProcessing [$1/$2] branch [$3]...\e[0m\e[39m"

echo ::group::Clone and make
git clone https://github.com/$1/$2 -b $3
cd $2

sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'$SYSTEM_VERSION'.apt' -o -iname 'packages.apt') | tr '\n' ' ')

mkdir build
cd build
cmake ../ -DBUILD_TESTING=false
make doc
echo ::endgroup::

echo ::group::Upload documentation
if [[ ! -z "$4" && "$4" != "n" ]]; then
  # Upload documentation
  echo -e "\e[46m\e[30mUploading documentation for $3...\e[0m\e[39m"
  sh upload_doc.sh $4
  echo -e "\e[46m\e[30mUploaded documentation for $3\e[0m\e[39m"
  echo ::endgroup::

  # Get the project version from cmake
  echo ::group::Add version
  version=`grep "project(.* VERSION" ../CMakeLists.txt  | grep -oP "(?<=VERSION )[0-9]*.[0-9]*.[0-9]*"`

  # Get the libName from the second parameter
  libName=`echo "$2" | grep -oP "(?<=ign-).*"`
  libName="${libName//-/_}"

  echo -e "\e[46m\e[30mAdding version [$version] for library [$libName], release date [$5]...\e[0m\e[39m"
  curl -k -X POST -d '{"libName":"'"$libName"'", "version":"'"$version"'", "releaseDate":"'"$5"'","password":"'"$6"'"}' https://api.ignitionrobotics.org/1.0/versions
  echo -e "\e[46m\e[30mAdded version [$version] for library [$libName], release date [$5]\e[0m\e[39m"
fi
echo ::endgroup::
