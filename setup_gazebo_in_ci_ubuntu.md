# Using `setup-gazebo` with Ubuntu Runners

This section details the usage of `setup-gazebo` GitHub Action on Ubuntu runners. The action can be directly run using GitHub hosted default Ubuntu runners, a full list of available runners can be found [here](https://docs.github.com/en/actions/using-github-hosted-runners/using-github-hosted-runners/about-github-hosted-runners#standard-github-hosted-runners-for-public-repositories). Alternatively the action can also be run inside Ubuntu docker containers.

## Overview

The `setup-gazebo` GitHub Action on Ubuntu runners takes in the following parameters as input:

* `required-gazebo-distributions`: A **required** parameter that specifies the Gazebo distribution to be installed. The allowed keywords are,
  * `citadel`
  * `fortress`
  * `garden`
  * `harmonic`
  * `ionic`
* `use-gazebo-prerelease`: An **optional** parameter to install pre-release binaries from OSRF repository.
* `use-gazebo-nightly`: An **optional** parameter to install nightly binaries from OSRF repository.
* `install-ros-gz`: An **optional** parameter to install the ROS 2 Gazebo bridge (`ros_gz`). This will require a previous ROS installation which can be done using the [`setup-ros`](https://github.com/ros-tooling/setup-ros) GitHub action. Installation of the `ros_gz` bridge supports the ROS official and ROS non-official (from packages.osrfoundation.org) variants following the [Installing Gazebo with ROS](https://gazebosim.org/docs/latest/ros_installation/#summary-of-compatible-ros-and-gazebo-combinations) documentation.


## Installing a compatible Gazebo on an Ubuntu runner

This workflow shows how to spawn a job to install Gazebo on an Ubuntu distribution. The action needs an input in the `required-gazebo-distributions` field.

### Default: Using GitHub-hosted runners

The following code snippet shows the installation of Gazebo Ionic on Ubuntu Noble runner.

```yaml
  jobs:
    test_gazebo:
      runs-on: ubuntu-24.04
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.2
          with:
            node-version: '20.x'
        - name: 'Setup Gazebo'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: ionic
        - name: 'Test Gazebo installation'
          run: 'gz sim --versions'
```

### Alternative: Using Ubuntu docker containers

The following code snippet shows the installation of Gazebo Ionic on an Ubuntu Noble docker container.

```yaml
  jobs:
    test_gazebo:
      runs-on: ubuntu-latest
      container:
        image: ubuntu:noble
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.2
          with:
            node-version: '20.x'
        - name: 'Setup Gazebo'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: ionic
        - name: 'Test Gazebo installation'
          run: 'gz sim --versions'
```

## Iterating on all Gazebo and Ubuntu combinations using a `matrix`

This workflow shows how to spawn one job per Gazebo release and iterates over all specified Gazebo and Ubuntu combinations. It is done by defining a `matrix` to iterate over jobs.

### Default: Using GitHub-hosted runners

```yaml
  jobs:
    test_gazebo:
      runs-on: ${{ matrix.ubuntu_distribution }}
      strategy:
        fail-fast: false
        matrix:
          gazebo_distribution:
            - citadel
            - fortress
            - garden
            - harmonic
          include:
            # Gazebo Citadel (Dec 2019 - Dec 2024)
            - ubuntu_distribution: ubuntu-20.04
              gazebo_distribution: citadel

            # Gazebo Fortress (Sep 2021 - Sep 2026)
            - ubuntu_distribution: ubuntu-20.04
              gazebo_distribution: fortress

            # Gazebo Garden (Sep 2022 - Nov 2024)
            - ubuntu_distribution: ubuntu-20.04
              gazebo_distribution: garden

            # Gazebo Harmonic (Sep 2023 - Sep 2028)
            - ubuntu_distribution: ubuntu-22.04
              gazebo_distribution: harmonic
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.2
          with:
            node-version: '20.x'
        - name: 'Check Gazebo installation on Ubuntu runner'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: ${{ matrix.gazebo_distribution }}
        - name: 'Test Gazebo installation'
          run: |
            if command -v ign > /dev/null; then
              ign gazebo --versions
            elif command -v gz > /dev/null; then
              gz sim --versions
            else
              echo "Neither ign nor gz command found"
              exit 1
            fi
```

### Alternative: Using Ubuntu docker containers

```yaml
  jobs:
    test_gazebo:
    runs-on: ubuntu-latest
    container:
      image: ${{ matrix.docker_image }}
    strategy:
      fail-fast: false
      matrix:
        gazebo_distribution:
          - citadel
          - fortress
          - garden
          - harmonic
        include:
          # Gazebo Citadel (Dec 2019 - Dec 2024)
          - docker_image: ubuntu:focal
            gazebo_distribution: citadel

          # Gazebo Fortress (Sep 2021 - Sep 2026)
          - docker_image: ubuntu:focal
            gazebo_distribution: fortress

          # Gazebo Garden (Sep 2022 - Nov 2024)
          - docker_image: ubuntu:focal
            gazebo_distribution: garden

          # Gazebo Harmonic (Sep 2023 - Sep 2028)
          - docker_image: ubuntu:jammy
            gazebo_distribution: harmonic
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4.0.3
        with:
          node-version: '20.x'
      - name: 'Check Gazebo installation on Ubuntu runner'
        uses: gazebo-tooling/setup-gazebo@v0.3.0
        with:
          required-gazebo-distributions: ${{ matrix.gazebo_distribution }}
      - name: 'Test Gazebo installation'
        run: |
          if command -v ign > /dev/null; then
            ign gazebo --versions
          elif command -v gz > /dev/null; then
            gz sim --versions
          else
            echo "Neither ign nor gz command found"
            exit 1
          fi
```

## Using pre-release and/or nightly Gazebo binaries

This workflow shows how to use binaries from [pre-release](https://packages.osrfoundation.org/gazebo/ubuntu-prerelease/) or [nightly] (https://packages.osrfoundation.org/gazebo/ubuntu-nightly/) Gazebo repositories instead of the stable repository by setting the `use-gazebo-prerelease` or `use-gazebo-nightly` to `true`. The following example shows the installation of Gazebo Ionic from pre-release and nightly repositories.

### Default: Using GitHub-hosted runners

```yaml
  jobs:
    test_gazebo:
        runs-on: ubuntu-24.04
        steps:
          - uses: actions/checkout@v4
          - uses: actions/setup-node@v4.0.2
            with:
              node-version: '20.x'
          - name: 'Check Gazebo installation on Ubuntu runner'
            uses: gazebo-tooling/setup-gazebo@v0.3.0
            with:
              required-gazebo-distributions: 'ionic'
              use-gazebo-prerelease: 'true'
              use-gazebo-nightly: 'true'
          - name: 'Test Gazebo installation'
            run: 'gz sim --versions'
```

### Alternative: Using Ubuntu docker containers

```yaml
  jobs:
    test_gazebo:
        runs-on: ubuntu-latest
        container:
          image: ubuntu:noble
        steps:
          - uses: actions/checkout@v4
          - uses: actions/setup-node@v4.0.2
            with:
              node-version: '20.x'
          - name: 'Check Gazebo installation on Ubuntu runner'
            uses: gazebo-tooling/setup-gazebo@v0.3.0
            with:
              required-gazebo-distributions: 'ionic'
              use-gazebo-prerelease: 'true'
              use-gazebo-nightly: 'true'
          - name: 'Test Gazebo installation'
            run: 'gz sim --versions'
```

## Installing ROS 2 and Gazebo side-by-side along with `ros_gz`

The following workflows show how to install ROS 2 using the GitHub action `ros-tooling/setup-ros` along with Gazebo installed using `setup-gazebo`.
The `ros-gz` package will be installed by setting the input parameter `install-ros-gz` to the required ROS 2 distributions.

The action takes care of installing the correct version of `ros_gz` based on the type of recommended or supported ROS-Gazebo combination specified. Check [this section](https://gazebosim.org/docs/latest/ros_installation/#summary-of-compatible-ros-and-gazebo-combinations) for more information on recommended and supported ROS-Gazebo combinations.

Starting with ROS 2 Jazzy, Gazebo is also available to be installed from ROS packages via [vendor packages](https://gazebosim.org/docs/all/ros2_gz_vendor_pkgs/#gazebo-vendor-packages).
When using `install-ros-gz` this action will check for availability of these Gazebo vendor packages and install them if available for the specified ROS 2 distribution.
Only the default (recommended) Gazebo release is currently available for the ROS 2 releases using the vendor packages (i.e if ROS 2 Jazzy is used, only Gazebo Harmonic is the valid option).
More information on vendor packages can be found in the [official documentation](https://gazebosim.org/docs/all/ros2_gz_vendor_pkgs/).

### Installing a recommended ROS-Gazebo combination

This example shows the installation of ROS 2 Humble and Gazebo Fortress which is a recommended ROS-Gazebo combination.


```yaml
  jobs:
    test_gazebo:
      env:
        ROS_DISTROS: 'humble'
      runs-on: ubuntu-latest
      container:
        image: ubuntu:jammy
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.4
          with:
            node-version: '20.x'
        - name: 'Install ROS 2 Humble'
          uses: ros-tooling/setup-ros@v0.7
          with:
            required-ros-distributions: ${{ env.ROS_DISTROS }}
        - name: 'Install Gazebo with ros_gz'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: 'fortress'
            install-ros-gz: ${{ env.ROS_DISTROS }}
        - name: Test Humble ros_gz installation
          run: |
            source /opt/ros/humble/setup.bash
            ros2 pkg list | grep ros_gz
            ign gazebo --version | grep 'version 6.*'
```

This example shows the installation of ROS 2 Jazzy and Gazebo Harmonic which is a recommended ROS-Gazebo combination. In this case, Gazebo libraries are will be installed as ROS packages.

```yaml
  jobs:
    test_gazebo:
      env:
        ROS_DISTROS: 'jazzy'
      runs-on: ubuntu-latest
      container:
        image: ubuntu:noble
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.3
          with:
            node-version: '20.x'
        - name: 'Install ROS 2 Jazzy'
          uses: ros-tooling/setup-ros@v0.7
          with:
            required-ros-distributions: ${{ env.ROS_DISTROS }}
        - name: 'Install Gazebo with ros_gz'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: 'harmonic'
            install-ros-gz: ${{ env.ROS_DISTROS }}
        - name: Test Jazzy ros_gz installation
          run: |
            source /opt/ros/jazzy/setup.bash
            ! [ $(apt list --installed gz-harmonic) ]
            ros2 pkg list | grep ros_gz
            gz sim --version | grep 'version 8.[0-9*].[0-9*]'
```

### Installing a supported ROS-Gazebo combination

This example shows the installation of ROS 2 Iron and Gazebo Harmonic which is a supported ROS-Gazebo combination.

```yaml
  jobs:
    test_gazebo:
    env:
      ROS_DISTROS: 'iron'
    runs-on: ubuntu-latest
    container:
      image: ubuntu:jammy
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4.0.3
        with:
          node-version: '20.x'
      - name: 'Install ROS 2 Iron'
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: ${{ env.ROS_DISTROS }}
      - name: 'Install Gazebo Harmonic with ros_gz'
        uses: gazebo-tooling/setup-gazebo@v0.3.0
        with:
          required-gazebo-distributions: 'harmonic'
          install-ros-gz: ${{ env.ROS_DISTROS }}
      - name: Test Iron ros_gz installation
        run: |
          source /opt/ros/iron/setup.bash
          ros2 pkg list | grep ros_gz
          gz sim --version | grep 'version 8.[0-9*].[0-9*]'
```
