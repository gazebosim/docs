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

## Documentation Structure

The documentation is organized by Gazebo release version, with shared files located in a root-level `common` directory.

### Repository Layout

- **`common/`**: Contains markdown files and folders that are shared across all Gazebo releases (e.g., installation overviews, architecture guides, governance documents, etc.).
- **`<release_name>/`** (e.g., `jetty/`): Contains markdown files specific to a particular release and its specific `index.yaml`.

### Release Navigation Menu (`index.yaml`)

Navigation menus are defined per-release in the release directory's `index.yaml` (e.g., `jetty/index.yaml`). The top-level `index.yaml` only contains global release metadata and library information, not navigation definitions.

#### Organizing Pages with Sections

The `pages` key in the release `index.yaml` defines the menu hierarchy using the `section` key to group related items.

Example:

```yaml
pages:
  - section: Get Started
    children:
      - name: getstarted
        title: Get Started
        file: common:get_started.md
      - name: release_notes
        title: Release Notes
        file: release_notes.md
```

#### Shared Files Prefix (`common:`)

To reference a file from the root-level `common` directory in a release-specific `index.yaml`, use the `common:` prefix in the `file` attribute.

```yaml
- name: getstarted
  title: Get Started
  file: common:get_started.md
```

Files without the prefix are assumed to be relative to the release directory.

#### Organizational Elements

If an entry in `index.yaml` has children but no `file` entry, it is considered an organizational element. The build script will automatically create a placeholder page for Sphinx to group the children under.

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
