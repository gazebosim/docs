#!/bin/bash
#
# Usage
#   1. Make sure you have the s3cmd tool configured. This script will use your 
#      ~/.s3cfg file to upload documentation.
#   2. Documentation upload requires a password to
#      https://api.ignitionrobotics.org. The password is listed on the
#      internal Open Robotics wiki. Set the IGN_VERSION_PASSWORD environment
#      variable to the correct password before running this script.
#   3. Run this command when you want to upload ALL of the Ignition library 
#      documentation. This will not upload the documentation contained in this
#      repository. To do that, you'll need to make a release of the
#      api.ignitionrobotics.org server (see the
#      bitbucket.org/ignitionrobotics/ign-webserver repository).
#         
#           sh ./build_docs.sh
#
#   4. Once complete you'll need to invalidate the Cloudfront distribution using
#
#           aws cloudfront create-invalidation --distribution-id \$CLOUDFRONT_DISTRIBUTION_ID --paths '/*'
#
#      The distribution ID can be found on the internal wiki right near the
#      IGN_VERSION_PASSWORD information

# Copy your s3 configuration to the local path so that docker can copy it.
s3cmd --dump-config > s3.cfg

# Build the docker container, which also uploads all documentation.
# We are using docker because a library's documentation links to other
# library documentation, and we want to guarantee a clean system.
docker build -t ign-acropolis-docs -f Dockerfile.acropolis --build-arg IGN_VERSION_PASSWORD --build-arg IGN_VERSION_DATE=`date -Iseconds` --no-cache .

docker build -t ign-blueprint-docs -f Dockerfile.blueprint --build-arg IGN_VERSION_PASSWORD --build-arg IGN_VERSION_DATE=`date -Iseconds` --no-cache .

# Reminder to tic over cloudfront.
echo "WARNING"
echo "  A CloudFront invalidation is required. Run the following command with the appropriate \$CLOUDFRONT_DISTRIBUTION_ID:\n"
echo "  aws cloudfront create-invalidation --distribution-id \$CLOUDFRONT_DISTRIBUTION_ID --paths '/*'"

# Remove your s3 configuration
rm s3.cfg
