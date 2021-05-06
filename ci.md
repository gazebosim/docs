# Ignition Continuous Integration

When developers push code changes into Ignition, a series of automated tests
run for various platforms. This process is called Continuous Integration, or CI.
This document covers all the CI jobs that are available for Ignition and how to
interpret their results.

## Types of checks

The main type of check that is performed by CI is compiling the code and running
all automated tests. Some jobs may also run linters and other static checkers.
Finally, we have checks that add labels, check DCO and coverage.

We use two platforms to run CI, each of them running different builds.

All checks are visible at the bottom of a pull request.

![PR checks](images/PR_checks.png)

### Jenkins server

Our Jenkins server is located at https://build.osrfoundation.org/. It runs builds
for all supported operating systems. It also runs an ABI checker for all stable
branches.

Each pull request to a stable branch or `main` branch will trigger:

* `<library>-ci-pr_any-ubuntu-auto-amd64`: Build and test on Ubuntu Linux
    * Builds are run inside Docker containers for Bionic or Focal.
    * By default, dependencies are installed from stable debian binaries for
      stable branches, and nightlies for `main` branches.
    * Library being tested is compiled using CMake and Make.
* `<library>-ci-pr_any-homebrew-amd64`: Build and test on macOS
    * Dependencies are installed from homebrew.
    * Library being tested is compiled using CMake and Make.
* `<library>-ci-pr_any-windows*-amd64`: Build and test on Windows
    * External dependencies are installed from vcpkg
    * Library being tested and its Ignition dependencies are compiled using colcon.

Pull requests to stable branches also trigger:

* `<library>-abichecker-any_to_any-ubuntu_auto-amd64`: Run ABI checker

### GitHub Actions

GitHub actions run on GitHub's servers. Currently, it builds Linux for all
libraries, and macOS for some of them.

Every commit to any branch will trigger:

* `Ubuntu CI / Ubuntu Bionic CI (push)`: Build, test and run linters on Ubuntu Bionic.
    * Builds are run inside Docker containers.
    * By default, dependencies are installed from stable debian binaries for
      stable branches, and nightlies for `main` branches.
    * Library being tested is compiled using CMake and Make.
    * Test coverage is computed and uploaded to Codecov.
* `Ubuntu CI / Ubuntu Focal CI (push)`: Build, test and run linters on Ubuntu Focal.
    * Builds are run inside Docker containers.
    * By default, dependencies are installed from stable debian binaries for
      stable branches, and nightlies for `main` branches.
    * Library being tested is compiled using CMake and Make.

Every pull request will also trigger the exact same builds, but with the
`pull_request` suffix. Note that pull requests from forks won't have the
`push` checks, while pull requests from the official repositories will have
both `pull_request` and `push`.

Other checks run on GitHub Actions:

* `DCO`: Checks that all commits are signed correctly for the
    [Developer Certificate of Origin](https://developercertificate.org/).
* `PR Collection Labeler`: Adds collection labels (i.e. Blueprint, Citadel...)
    to the pull request automatically according to the target branch.
* `Ticket opened`: Adds the pull request to the [Core development board](https://github.com/orgs/ignitionrobotics/projects/3).
* `codecov/*`: Checks that the test coverage hasn't been reduced.

## Required checks

Some checks are marked as `Required`. This means the pull request can't
be merged unless that check is green.

Ideally, all checks would be required. But known warnings, test failures and
flaky tests often prevent us from marking them as required. Until these are
addressed, some checks will need to be checked by hand when they fail, to see
if those failures are unexpected. See
[this issue](https://github.com/ignition-tooling/release-tools/issues/398) for
what checks are required for each library.

## Interpreting results

On the GitHub UI, checks can be:

* ✅: Passed, all good!
* 🟡: Pending, results haven't been received yet.
* ❌: Failed, something is wrong.

Depending on the library and on the build queue, checks can be in a pending
🟡 state from a couple of minutes up to an entire day. If a build is in this
state for too long, there may be some issue with infrastructure and the
build didn't run or didn't report back.

Failing builds ❌ usually require more investigation:

* If the build is `Required`, something is definitely wrong and must be fixed
  before merging the pull request.
* If the build is not required, investigation is needed to find out if the
  failure is pre-existing or if it is being introduced by the pull request.

Clicking on the `Details` button of a failing build takes you to the build so
that you can inspect it.

Builds can fail for a variety of reasons, for example:

* The code failed to compile. This should never be accepted, even in non-required
  checks, and must be fixed.
    * Jenkins: When there's a compilation failure, the build is marked red 🔴.
               On the build's page, click on `Console Output` then scroll until
               you find the compilation errors under the `compiling` section.
    * Actions: When there's a compilation failure, the build is marked red ❌.
               On the build logs, the compilation failure should be under the
               `make` collapsible.
* There are test failures. Test failures that aren't pre-existing must be fixed.
    * Jenkins: When there are test failures, the build is marked yellow 🟡.
               On the build's page, the failing tests are listed under
               `Test Result`. You can also check the `Console Output` for the
               full test logs.
    * Actions: When there are test failures, the build is marked red ❌.
               On the build logs, the test failures should be under the
               `make test` collapsible.
    * Check which tests are known failures by searching for the test name on
      issues.
* There are warnings. Warnings that aren't pre-existing must be fixed.
    * Jenkins: When there are warnings, the build is marked yellow 🟡.
               On the build's page, the warnings are listed under
               `GNU C Compiler` and `CMake`. You can also check the
               `Console Output` for the full warning logs.
    * Actions: Does not detect compiler or CMake warnings.
* There are static checker failures. All code checker failures must be fixed.
    * Jenkins: Does not run static checkers.
    * Actions: When there are code cherk failures, the build is marked red ❌.
               On the build logs, the checker failures should be under the
               `Code check` collapsible.
* There are infrastructure failures. These should be reported to the build farmer.
    * Jenkins: When there's an infra failure, the build is marked red 🔴.
               These can manifest in various ways. In general, if the
               build logs don't fall in the categories above, there's a high
               chance it's an infrastructure failure. Common patterns usually
               have several `java` or `hudson` messages.
    * Actions: Infrastructure failures haven't been identified.

## Triggering CI

CI is usually triggered automatically when code is pushed or a pull request is
opened. But often it's necessary to re-trigger CI without making changes to the
code if:

* There were infrastructure failures in a previous build.
* There are fixes upstream, on infra or dependencies, that may make a new build pass.

The following methods can be used to re-trigger builds:

* Jenkins
    * Make a comment starting with `@osrf-build run tests` on the pull request
      and all Jenkins builds will be re-triggered.
    * To restart just one specific build and avoid re-running builds that aren't
      necessary, go to the failing build and click `Retry`. This is only available
      to maintainers.
* Actions
    * On the top-right of a build, click `Re-run jobs`. This button is only
      available to maintainers and sometimes mysteriously disappears.

### Custom branches

Maintainers can manually trigger builds for custom branches without opening pull
requests, optionally building dependencies from source and changing other
configurations for testing.

* Actions: Builds are run for every commit. For building dependencies from source,
           add a `.github/ci/dependencies.yaml` file as described
           [here](https://github.com/ignition-tooling/action-ignition-ci#source-dependencies).
* Jenkins: The `pr_any` jobs can be triggered manually for any branch by
           maintainers. Go to the job's page, for example
           [ignition_launch-ci-pr_any-ubuntu_auto-amd64](https://build.osrfoundation.org/job/ignition_launch-ci-pr_any-ubuntu_auto-amd64/),
           click on `Build with Parameters` and put the desired
           `origin/$branch_name` under `sha1`. For various configurations, a
           branch must be created in
           [release-tools](https://github.com/ignition-tooling/release-tools)
           and added under `RTOOLS_BRANCH`. For example, see
           [this change](https://github.com/ignition-tooling/release-tools/commit/28fcb9d8ad66ad0a29ad4d2675c4c451d41fba19)
           for building a custom `ign-rendering` branch on `ign-sensors` CI.

## Development

All the infrastructure for our CI is in the
[ignition-tooling](https://github.com/ignition-tooling) organization.

* Most of the stuff running in Jenkins is in
  [release-tools](https://github.com/ignition-tooling/release-tools/).
* The GitHub action is in
  [action-ignition-ci](https://github.com/ignition-tooling/action-ignition-ci/).

