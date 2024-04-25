#!/bin/bash
# Command line parameters:
# 1 - GitHub organization name. For example gazebosim or osrf.
# 2 - the name of the Gazebo repository. For example gz-math.
# 3 - the name of the branch. For example gz-math7
# 4 - 'y' or 'n' without the quotes that indicate whether or not to upload docs
# 5 - Release date in the ISO 8601 format. See the command `date -Iseconds`.
# 6 - Password to https://api.gazebosim.org/1.0/versions.

set -o verbose

export DEBIAN_FRONTEND=noninteractive

echo -e "\e[46m\e[30mProcessing [$1/$2] branch [$3]...\e[0m\e[39m"

echo ::group::Clone and make
git clone https://github.com/$1/$2 -b $3
cd $2

sudo DEBIAN_FRONTEND=noninteractive apt -y install \
  $(sort -u $(find . -iname 'packages-'$SYSTEM_VERSION'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | tr '\n' ' ')

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
  libName=`echo "$2" | grep -oP "(?<=gz-).*"`
  libName="${libName//-/_}"

  majorVersion="${version/\.*/}"
  # Make sure the majorVersion is a valid number
  numberCheckRegex='^[0-9]+$'
  if [[ $majorVersion =~ $numberCheckRegex ]]; then
    # If this is ign-gazebo (gz-sim <= 6), the upload_doc.sh will upload to api/gazebo so we'll need to
    # sync to api/sim manually
    if [[ "$libName" == "sim" && "$majorVersion" -le 6 ]]; then
      aws s3 sync s3://gazebosim.org/api/gazebo/${majorVersion}/ s3://gazebosim.org/api/sim/${majorVersion}/
    fi
  else
    echo "Invalid major version ${majorVersion}"
  fi

  echo -e "\e[46m\e[30mAdding version [$version] for library [$libName], release date [$5]...\e[0m\e[39m"
  curl -k -X POST -d '{"libName":"'"$libName"'", "version":"'"$version"'", "releaseDate":"'"$5"'","password":"'"$6"'"}' https://api.gazebosim.org/1.0/versions
  echo -e "\e[46m\e[30mAdded version [$version] for library [$libName], release date [$5]\e[0m\e[39m"
fi
echo ::endgroup::
