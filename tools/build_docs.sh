#!/bin/bash
#
# Usage
#   1. Export the AWS keys using:
#           export AWS_ACCESS_KEY_ID=aws_access_key_value
#           export AWS_SECRET_ACCESS_KEY=aws_secret_key_value
#   2. Documentation upload requires a password to
#      https://api.gazebosim.org. The password is listed on the
#      internal Open Robotics wiki. Set the GZ_VERSION_PASSWORD environment
#      variable to the correct password before running this script.
#   3. Run this command when you want to upload ALL of the Gazebo library
#      documentation. This will not upload the documentation contained in this
#      repository. To do that, you'll need to make a release of the
#      api.gazebosim.org server (see the
#      github.com/gazebo-web/gazebosim-web-backend repository).
#
#           sh ./build_docs.sh <release_name | all>
#
#   4. Once complete you'll need to invalidate the Cloudfront distribution using
#
#           aws cloudfront create-invalidation --distribution-id \$CLOUDFRONT_DISTRIBUTION_ID --paths '/*'
#
#      The distribution ID can be found on the internal wiki right near the
#      GZ_VERSION_PASSWORD information

# Build the docker container, which also uploads all documentation.
# We are using docker because a library's documentation links to other
# library documentation, and we want to guarantee a clean system.

if [[ $1 == 'all' || $1 == 'citadel' || $1 == 'Citadel' ]]; then
  echo -e "\e[46m\e[30mUploading documentation for Citadel\e[0m\e[39m"
  docker build -t gz-citadel-docs -f Dockerfile.citadel --build-arg GZ_VERSION_PASSWORD --build-arg GZ_VERSION_DATE=`date -Iseconds` --no-cache --build-arg AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS_KEY .
fi

if [[ $1 == 'all' || $1 == 'fortress' || $1 == 'Fortress' ]]; then
  echo -e "\e[46m\e[30mUploading documentation for Fortress\e[0m\e[39m"
  docker build -t gz-fortress-docs -f Dockerfile.fortress --build-arg GZ_VERSION_PASSWORD --build-arg GZ_VERSION_DATE=`date -Iseconds` --no-cache --build-arg AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS_KEY .
fi

if [[ $1 == 'all' || $1 == 'garden' || $1 == 'Garden' ]]; then
  echo -e "\e[46m\e[30mUploading documentation for Garden\e[0m\e[39m"
  docker build -t gz-garden-docs -f Dockerfile.garden --build-arg GZ_VERSION_PASSWORD --build-arg GZ_VERSION_DATE=`date -Iseconds` --no-cache --build-arg AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS_KEY .
fi

if [[ $1 == 'all' || $1 == 'harmonic' || $1 == 'Harmonic' ]]; then
  echo -e "\e[46m\e[30mUploading documentation for Harmonic\e[0m\e[39m"
  docker build -t gz-harmonic-docs -f Dockerfile.harmonic --build-arg GZ_VERSION_PASSWORD --build-arg GZ_VERSION_DATE=`date -Iseconds` --no-cache --build-arg AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS_KEY .
fi


# Reminder to tic over cloudfront.
echo "WARNING"
echo "  A CloudFront invalidation is required. Run the following command with the appropriate \$CLOUDFRONT_DISTRIBUTION_ID:\n"
echo "  aws cloudfront create-invalidation --distribution-id \$CLOUDFRONT_DISTRIBUTION_ID --paths '/*'"
