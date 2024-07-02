---
orphan: true
---
# Release repositories

> TODO: the document needs to be completed with real information. The points
> below are only for reference. Check with the infra-team if you need
> any information related to -release repositories.

## Design

> TODO

## Ignore architectures using '.releasepy_NO_ARCH_'
To disable builds for a specific architecture, add a file to the `-release`
repository that starts with `.releasepy_NO_ARCH_` and append the name of
the architecture to be excluded. The file can be added to the root of the
`-release` repository (like [.releasepy_NO_ARCH_ARM64 in ign-gazebo2-release](https://github.com/gazebo-release/ign-gazebo2-release/blob/master/.releasepy_NO_ARCH_ARM64))
or in a distro-specific sub-folder (like [bionic/.releasepy_NO_ARCH_armhf in ign-launch2-release](https://github.com/gazebo-release/ign-launch2-release/blob/master/bionic/.releasepy_NO_ARCH_armhf)).
The architecture suffix is not case-sensitive.

## New distributions in Debian/Ubuntu

> TODO: script

## New repository

> TODO: point to major version bump doc)
