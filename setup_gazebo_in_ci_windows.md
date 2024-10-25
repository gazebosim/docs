# Using `setup-gazebo` with Windows Runners

This section details the usage of `setup-gazebo` GitHub Action on Windows runners. The action can be directly run using GitHub hosted Windows runners, a full list of available runners can be found [here](https://docs.github.com/en/actions/using-github-hosted-runners/using-github-hosted-runners/about-github-hosted-runners#standard-github-hosted-runners-for-public-repositories).

## Overview

The `setup-gazebo` GitHub Action on Windows runners takes in the following parameters as input:

* `required-gazebo-distributions`: A **required** parameter that specifies the Gazebo distribution to be installed. The allowed keywords are,
  * `citadel`
  * `fortress`
  * `harmonic`
  * `ionic`

## Install Gazebo on a Windows runner

This workflow shows how to install Gazebo Ionic on a Windows runner. The action requires a Conda package management system such as miniconda as all Gazebo packages are available on conda-forge. The action is run by specifying the distribution of choice in `required-gazebo-distributions` field.

```yaml
  jobs:
    test_gazebo:
      runs-on: windows-latest
      steps:
        - uses: actions/checkout@v4
        - uses: actions/setup-node@v4.0.2
          with:
            node-version: '20.x'
        - uses: conda-incubator/setup-miniconda@v3
        - name: 'Check Gazebo installation on Windows runner'
          uses: gazebo-tooling/setup-gazebo@v0.3.0
          with:
            required-gazebo-distributions: 'ionic'
        - name: 'Test Gazebo installation'
          shell: pwsh
          run: |
            conda activate
            gz sim --versions
```
