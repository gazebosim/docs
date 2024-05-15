# Gazebo Release Instructions

## Initial setup

A small number of configurations and credentials need to be made on the
developer's system before triggering the release.  If a permanent operating
system is used for releasing, these installation steps only need to be
executed once.

### 1. Software and configurations

> Triggering the releasing process is only supported on Linux at this moment.

The tool designed to facilitate the process of releasing software for all
platforms is called [`release.py`](https://github.com/gazebo-tooling/release-tools/blob/master/release.py), and is hosted at https://github.com/gazebo-tooling/release-tools.
Cloning the `release-tools` repository is required to perform a new release:

```
cd ~/
git clone https://github.com/gazebo-tooling/release-tools.git
```

Some Debian tools require the following variables to be set:
```bash
export DEBEMAIL="<username>@openrobotics.org"
export DEBFULLNAME="<Your full name>"
```

> **Note:** the two above exported variables can be added to `.bashrc` to have
> them configured automatically on every run.

### 2. Access and Credentials

Before starting the release process, make sure to ask for write access to:
1.  The Gz library intended to be released and
2.  `gazebo-release` repository.

There are some credentials needed to interact with the release
process:

 * Release token: magic sequence of characters needed while running `release.py`
   to interact with `build.osrfoundation.org`. This should be given to Gz developers that run releases by the Infra team.

## For Each Release

### 1. Team and development checks

When creating a new release, there are some guidelines to follow before starting
the process:

 * Ask the team if there are any concerns about making the release.
 * Check if there are changes to previous library versions that need to be forward-ported.
 * See if there are open PRs against the release branch (release branch is the
   one with the name `ign/gz-fooX` where foo is the name of the Gz library and
   X the major version of the version bump planned) that could go into the new
   release.

### 2. Update code version and changelogs

The first step to get a new release ready is to update the current code (upstream)
version (view the [versioning](release#versioning) section for more information). This
bump could be in the major number (non-compatible changes), minor number (new
features), patch number (patches and bugfixes).

**Bumping major number** of the version implies some work to have the
[metadata](#metadata-for-releasing) updated correctly. There is a [dedicated
document](releasing/bump_major.md) that you should go through before continuing to work through the steps in this
document.

   1. To update the upstream version a local checkout of the Gz library is
   needed. A new branch is required to submit changes:

      ```bash
      # version X.Y.Z
      git checkout -b bump_to_X_Y_Z
      ```

   2. The current upstream version can be found in `CMakeLists.txt` file
   following the CMake `project declaration`:

       ```cmake
       # Gz library named foo and version X.Y.Z
       project(gz-fooX VERSION X.Y.Z)
       ```

      Stable releases can modify the X, Y or Z directly while prereleases will need to include the preX (X number starts with 1) suffix in the `gz_configure_project`:

      ```cmake
      # first prerelease of a serie, number 1
      gz_configure_project(VERSION_SUFFIX pre1)
      ```

  3. Together with bumping the version number, **updating the Changelog and Migration
  documents** is required. For updating the `Changelog.md` file, the script
  [source_changelog.bash](https://github.com/gazebo-tooling/release-tools/blob/master/source-repo-scripts/source_changelog.bash)
  can be useful:

      ```bash
      cd <path to source code>
      bash <path to release-tools>/source-repo-scripts/source_changelog.bash <previous tag>
      # e.g. bash ../release-tools/source-repo-scripts/source_changelog.bash 3.5.0
      ```
  The `Migration.md` document needs to be updated if some breaking changes are in the Changelog.

  4. Open a pull request for reviewing ([example PR](https://github.com/gazebosim/gz-physics/pull/447)).
  When opening a PR look for the "Release" section in the template and follow the instructions there.

### 3. Update packages version

Once the PR for bumping the code version is merged, the binary packages version
needs to be updated for the Debian/Ubuntu packages. Brew metadata will be
updated by the building server when creating the binary packages
(known as `bottles`).

The version in the packages (binary version) should be similar to the one updated
in the code in the step above but usually has some other components that reflect the
package metadata version (i.e: new code version 2.1.0, new version in Debian packages
is usually 2.1.0-1).

There should be a repository matching the name and major version of Gz
library that you want to bump in the
[gz-release](https://github.com/gazebo-release/) GitHub organization.
(see [release repositories document](releasing/release_repositories.md) for more
information about how they are used).

1. It is required to clone the corresponding release repository to update the
  binary version:

    ```bash
    # Gz library named foo and major version X
    git clone https://github.com/gazebo-release/gz-fooX-release
    ```

2. To bump the package versions that will appear in Debian/Ubuntu binary packages there is a helper script in `release-tools` (see [initial setup](#initial-setup)). The script is called `changelog_spawn.sh` and require to be executed while the active directory is a `release repository`:

    ```bash
    # Gz library named foo and major version X
    cd gz-fooX-release
    ~/release-tools/release-repo-scripts/changelog_spawn.sh X.Y.Z-R

    # Example gz-cmake3 bumped from 3.0.0 to 3.0.1
    cd gz-cmake3-release
    ~/release-tools/release-repo-scripts/changelog_spawn.sh 3.0.1-1
    ```

`changelog_spawn.sh` will display information about the Ubuntu/Debian versions being updated as well as a `git diff` before uploading information to the GitHub release repository.

## Triggering the release

After updating the code and releasing metadata everything is ready to launch the
build in the server. Now, the following needs to happen:

 1. Tag the corresponding code in the repository and upload that tag to GitHub.
 1. Request `build.osrfoundation.org` server to start a chain of job by calling `gz-fooX-source`.
    1. `gz-fooX-source` will generate the source tarball and call `repository_uploader_packages` to upload it to
    1. `gz-fooX-source` will call `_releasepy` that will generate the builders jobs:
        1. Debian/Ubuntu: use `ign/gz-fooX-debbuilder` job names
        1. Brew: entry job is `generic-release-homebrew_pull_request_updater`

The `release.py` script  and Jenkins will perform all these actions. For more information of all the processes
triggered by the `release.py` script please check [the release process](release#using-the-gzdev-repository-command).

### 4. Executing release.py

Make sure you are in the source code repository before running `release.py`.
You should be on the branch to be released, after the pull request bumping
the version has been merged (run `git status` to check the branch, and `git
    log` to check that the version bump pull request has been included).
Running `release.py` from the source code repository will generate and
upload some Git tags ("release tags") to the source code repository.

You will also need the token described in the [credentials
section](#access-and-credentials).

**dry-run simulation mode**

The `release.py` tool supports a `--dry-run` flag that allows users to
simulate releases (nothing is modified) in order to ensure that the correct
arguments are being used to trigger a particular release.  Gz releasers
should **always** call `release.py` with `--dry-run` first in order to
ensure that proper commands are being used to trigger releases.

The script needs to be run from the repository with the source code (i.e., the repository where the Gz library version bump pull request took place):

```bash
# Example of dry-run for gz-cmake3 bumped to 3.0.1
cd gz-cmake3
git checkout gz-cmake3
~/release-tools/release.py gz-cmake3 3.0.1 dry-run-fake-token --dry-run
```

**release.py for stable releases**

```bash
# Gz library named foo and major version X
cd ign/gz-fooX
git checkout ign/gz-fooX

# Example gz-cmake3 bumped to 3.0.1 with jenkins_token credential
cd gz-cmake3
git checkout gz-cmake3
# please replace <jenkins_token> with real release token (check crendentials section)
~/release-tools/release.py gz-cmake3 3.0.1 <jenkins_token>
```

**release.py for prereleases or nightlies**

When releasing prereleases or nightly releases, there are some special flags
to be set. The `--upload-to-repo` argument is mandatory when running
`release.py`, and should be set to `prerelease` or `nightly`.

```bash
# Example gz-cmake3 bumped to prerelease 3.0.0~pre1 with jenkins_token credential
cd gz-cmake3
git checkout gz-cmake3
# please replace <jenkins_token> with real release token (check crendentials section)
~/release-tools/release.py gz-cmake3 3.0.0~pre1 <jenkins_token> --upload-to-repo prerelease
```

Nightly invocation is generally coded in the server. The version will be
taken from the last changelog entry and modified during building. No source
code will be uploaded, but taken directly in the binary build from the
branch pointed by `--nightly-src-branch`.

```bash
# Example gz-cmake3 nightly from main branch with jenkins_token credential
# please replace <jenkins_token> with real release token (check crendentials section)
~/release-tools/release.py gz-cmake3 3.0.0~pre1 <jenkins_token> --upload-to-repo nightly --nightly-src-branch main

```

**Binary version schema for prereleases and nightlies**

Prerelease and nightlies binary versions use particular naming schema to define right
ordering for package managers. [This information about versioning](https://classic.gazebosim.org/tutorials?tut=install_unstable&cat=install#Versioninginnightlyandprerelease) in Gazebo Classic applies to the [Gazebo] too.

**release.py for revision bumps**

Bumping the [revision number for binary packages](release#versioning) is a special
case of releasing since the original tarball with the source code will
remain the same. Once the release repository is ready with the new release
version, `release.py` needs the `--only-bump-revision-linux` flag.

Note that the `--source-tarball-uri` parameter is needed with the original tarball
of the software version. All the tarballs can be found at https://classic.gazebosim.org/distributions
or the information should appear in the parameters of the Jenkins -debbuilder build that created
the first version of the sofware.

```bash
# Example gz-cmake3 bumped from 3.0.1-1 to 3.0.1-2 with jenkins_token credential
# please replace <jenkins_token> with real release token (check crendentials section)
~/release-tools/release.py gz-cmake3 3.0.1 <jenkins_token> --source-tarball-uri https://osrf-distributions.s3.amazonaws.com/gz-cmake/releases/gz-cmake-3.0.1.tar.bz2 --only-bump-revision-linux -release-version 2
```

## Checking the Building Process

For checking that the build process is ongoing as expected:

1. Checking the source generation and upload:
   1. The `gz-fooX-source` job (change fooX by the software name being released, i.e: math7) is triggered directly
      by `release.py` (script output will point to it) should have finished successfully.
   1. The `gz-fooX-source` job launches a one new build in the `repository_uploader_packages` job with the name of the software.
      1. If there is a failure, contact with the Infra team.
   1. There should be one new build in the `_releasepy` job with the name of the software.
      1. If there is a failure, you can check the output if you can access to the job workspace in the Jenkins build,
         since it is disabled by default. Contact with the Infra team for help.
1. Checking package generation after the `_releasepy` build:
    1. Several `-debbuilder` jobs should be in https://build.osrfoundation.org/.
       For watching the jobs to see if any of them fail or are marked unstable,
       open the page for a specific debbuild, such as
       https://build.osrfoundation.org/job/gz-math7-debbuilder/.

       1. If there is a failure, check the list of supported
          architectures in the corresponding [Gazebo Platform Support issue](https://github.com/gazebo-tooling/release-tools/issues?q=label%3A%22Tier+Platform+Support%22+)

       1. To check if a debbuild has previously succeeded for a given architecture,
          check packages.osrfoundation.org to see the most recent successful builds.
          (i.e most recent [Ubuntu builds of ignition-gazebo6](https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/i/ignition-gazebo6/)
          or the [Debian builds of ignition-gazebo6](https://packages.osrfoundation.org/gazebo/debian-stable/pool/main/i/ignition-gazebo6/).

       1. If the failure is on a supported architecture, check the source repository for an existing report of this failure and if none
          exists, report the failure (see [gazebosim/gz-math#161](https://github.com/gazebosim/gz-math/issues/161)
          for an example).

       1. If a build is unstable, check if it was unstable before this release and if it has already been reported.
          A common cause of unstable debbuilds is newly installed files that are not captured by the patterns in the `.install`
          files in the `-release` repository. This can be checked by searching for `dh_missing` in the console log of the
          unstable build and looking for a list of uninstalled files. Example pull requests that fix problems with `dh_missing`
          are [gazebo-release/gz-transport11-release#4](https://github.com/gazebo-release/gz-transport11-release/pull/4)
          and [gazebo-release/gz-tools-release#4](https://github.com/gazebo-release/gz-tools-release/pull/4).

    1. A pull request was opened to https://github.com/osrf/homebrew-simulation
       1. This pull request may take a minute or two to open.
       1. Once it is open, make a comment containing the text "build bottle".
          For further details, see the [README at osrf/homebrew-simulation](https://github.com/osrf/homebrew-simulation#to-build-bottles).
