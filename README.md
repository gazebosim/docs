# Gazebo Documentation

This repository contains documentation about [Gazebo](https://gazebosim.org) that does not pertain to a specific
[Gazebo library](https://gazebosim.org/libs). An example would be
installation instructions for an Gazebo release. The documentation
contained in this repository can be view at
[https://gazebosim.org/docs](https://gazebosim.org/docs).

Each [Gazebo library](https://gazebosim.org/libs) maintains
documentation and tutorials that are scoped to the features and
capabilities of the library itself. The documentation for a library can be
found under the `API Reference` section of [https://gazebosim.org/docs](https://gazebosim.org/docs).

## Updating gazebosim.org

## Main docs

The documentation in this repository is updated whenever the
[gazebosim-web-backend](https://github.com/gazebo-web/gazebosim-web-backend),
is deployed. The gazebosim-web-backend webserver maintains a clone of this repository, and serves the markdown pages to https://gazebosim.org/docs.

## Library docs

Instructions on how to update all of the library docs is contained in the
[tools/build_docs.sh](https://github.com/gazebosim/docs/blob/master/tools/build_docs.sh) script.
