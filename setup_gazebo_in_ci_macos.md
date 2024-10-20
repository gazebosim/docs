# Using `setup-gazebo` with macOS Runners

This section details the usage of `setup-gazebo` GitHub Action on macOS runners. The action can be directly run using GitHub hosted macOS runners, a full list of available runners can be found [here](https://docs.github.com/en/actions/using-github-hosted-runners/using-github-hosted-runners/about-github-hosted-runners#standard-github-hosted-runners-for-public-repositories).

## Overview

The `setup-gazebo` GitHub Action on macOS runners takes in the following parameters as input:

* `required-gazebo-distributions`: A **required** parameter that specifies the Gazebo distribution to be installed. The allowed keywords are,
  * `citadel`
  * `fortress`
  * `garden`
  * `harmonic`
  * `ionic`

## Installing Gazebo on a macOS runner

This workflow shows how to install Gazebo Ionic on a macOS worker using the Homebrew package manager which is installed by the action. To run, this action needs an input for `required-gazebo-distributions` parameter.

```yaml
  jobs:
    test_gazebo:
      runs-on: macos-13
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.2
          with:
            node-version: '20.x'
        - name: 'Check Gazebo installation on MacOS runner'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: 'ionic'
        - name: 'Test Gazebo installation'
          run: 'gz sim --versions'
```