# Ignition Documentation

This repository contains documentation about [Ignition
Robotics](https://ignitionrobotics.org) that does not pertain to a specific
[Ignition library](https://ignitionrobotics.org/libs). An example would be
installation instructions for an Ignition Robotics release. The documentation
contained in this repository can be view at
[https://ignitionrobotics.org/docs](https://ignitionrobotics.org/docs).

Each [Ignition library](https://ignitionrobotics.org/libs) maintains
documentation and tutorials that are scoped to the features and
capabilities of the library itself. The documentation for a library can be
found under the `API Reference` section of [https://ignitionrobotics.org/docs](https://ignitionrobotics.org/docs).

## Updating ignitionrobotics.org

## Main docs

The documentation in this repository is updated whenever the ign-webserver,
a private repository to Open Robotics, is deployed. The ign-webserver maintains a clone of this repository, and
serves the markdown pages to https://ignitionrobotics.org.

## Library docs

Instructions on how to update all of the library docs is contained in the
[tools/build_docs.sh](https://github.com/ignitionrobotics/docs/blob/master/tools/build_docs.sh) script.
