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

The documentation in this repository is built using [Sphinx](https://www.sphinx-doc.org/).
To build, you need to install the following:

* python virtualenv

Create the virtual env and activate it:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Then install the necessary dependencies:

```bash
pip install -r requirements.txt
```

```bash
python3 build_multiversion.py
```

This will build all the documentation for all versions of Gazebo.
You can preview the result locally by running an HTTP server on
the output directory `.build`. For example:

```bash
python3 -m http.server 8000 -d .build

```

This will serve the website on <http://localhost:8000>

For quicker iteration, you can build the documentation for a subset
of Gazebo versions. To build `garden` and `harmonic`:

```bash
python3 build_multiversion.py --release garden harmonic
```

## Library docs

Instructions on how to update all of the library docs is contained in the
[tools/build_docs.sh](https://github.com/gazebosim/docs/blob/master/tools/build_docs.sh) script.
