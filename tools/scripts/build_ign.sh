#!/bin/bash
# Command line parameters:
# 1 - bitbucket organization name. For example ignitionrobotics or osrf.
# 2 - the name of the ignition repository. For example ign-math.
# 3 - the name of the branch. For example ign-math6
# 4 - 'y' or 'n' without the quotes that indicate whether or not to upload docs
# 5 - Release date in the ISO 8601 format. See the command `date -Iseconds`.
# 6 - Password to https://api.ignitionrobotics.org/1.0/versions.

set -o errexit
set -o verbose

hg clone https://bitbucket.org/$1/$2 -b $3
cd $2 
mkdir build
cd build
cmake ../
sudo make install

if [[ ! -z "$4" && "$4" != "n" ]]; then
  # Upload documentation
  sh upload_doc.sh $4

  # Get the project version from cmake
  version=`grep "project(.* VERSION" ../CMakeLists.txt  | grep -oP "(?<=VERSION )[0-9].[0-9].[0-9]"`

  # Get the libName from the second parameter
  libName=`echo "$2" | grep -oP "(?<=ign-).*"`

  curl -k -X POST -d '{"libName":"'"$libName"'", "version":"'"$version"'", "releaseDate":"'"$5"'","password":"'"$6"'"}' https://api.ignitionrobotics.org/1.0/versions
fi
