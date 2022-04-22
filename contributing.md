# Contributing to Ignition

Thank you for your interest in contributing to Ignition!

The following is a set of guidelines for contributing to Ignition
and its components, which are hosted in the [Ignition
Organization](https://github.com/ignitionrobotics) on GitHub. These
are mostly guidelines, not rules. Use your best judgment, and feel free to
propose changes to this document in a pull request.

#### Table of Contents

[Code of Conduct](https://ignitionrobotics.org/docs/all/contributing#code-of-conduct)

[Project Design](https://ignitionrobotics.org/docs/all/contributing#project-design)

  * [Repository List](https://ignitionrobotics.org/docs/all/contributing#repository-list)

[How to Contribute](https://ignitionrobotics.org/docs/all/contributing#how-to-contribute)

  * [Reporting Bugs](https://ignitionrobotics.org/docs/all/contributing#reporting-bugs)
  * [Suggesting Enhancements](https://ignitionrobotics.org/docs/all/contributing#suggesting-enhancements)
  * [Contributing Code](https://ignitionrobotics.org/docs/all/contributing#contributing-code)
  * [Tracking Progress](https://ignitionrobotics.org/docs/all/contributing#tracking-progress)

[Writing Tests](https://ignitionrobotics.org/docs/all/contributing#writing-tests)

  * [Test Coverage](https://ignitionrobotics.org/docs/all/contributing#test-coverage)

[Styleguides](https://ignitionrobotics.org/docs/all/contributing#style-guides)

[Appendix](https://ignitionrobotics.org/docs/all/contributing#appendix)

## Code of Conduct

This project and everyone participating in it is governed by the [Ignition
Code of Conduct](https://github.com/ignitionrobotics/ign-gazebo/blob/main/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this
code. Please report unacceptable behavior at [https://ignitionrobotics.org/support](https://ignitionrobotics.org/support).

## Project Design

### Repository List

The following is a list of the major Ignition repositories.
A complete list can be found at
[https://github.com/ignitionrobotics/](https://github.com/ignitionrobotics/).

* [ign-cmake](https://github.com/ignitionrobotics/ign-cmake): CMake
modules used to configure and build Ignition libraries.
* [ign-common](https://github.com/ignitionrobotics/ign-common): Set of
general purpose components and utilities, such as 3D mesh processing, console
logging, and signal handling.
* [ign-fuel-tools](https://github.com/ignitionrobotics/ign-fuel-tools): Interact
with Ignition Fuel servers.
* [ign-gazebo](https://github.com/ignitionrobotics/ign-gazebo):
A high-fidelity 3D rigid-body dynamic simulator.
* [ign-gui](https://github.com/ignitionrobotics/ign-gui): QT-based
library to configure and manage graphical applications.
* [ign-launch](https://github.com/ignitionrobotics/ign-launch): Launch
executables and plugins.
* [ign-math](https://github.com/ignitionrobotics/ign-math): A math
library targeted toward robotic applications.
* [ign-msgs](https://github.com/ignitionrobotics/ign-msgs): Protobuf
messages and utilities for simulation and robotics.
* [ign-physics](https://github.com/ignitionrobotics/ign-physics):
Plugin based library for physics simulation.
* [ign-plugin](https://github.com/ignitionrobotics/ign-plugin): Library
for registering, loading, and managing plugin libraries.
* [ign-rendering](https://github.com/ignitionrobotics/ign-rendering):
Library that supports rendering through different engines, such as
[OGRE](https://www.ogre3d.org/) and [Optix](https://developer.nvidia.com/optix).
* [ign-sensors](https://github.com/ignitionrobotics/ign-sensors): A set
of models that generate realistic sensor data.
* [ign-tools](https://github.com/ignitionrobotics/ign-tools): Provides
the `ign` command line interface that can be configured and used by multiple
libraries.
* [ign-transport](https://github.com/ignitionrobotics/ign-transport):
High performance inter- and intra-process communication based on
[ZeroMQ](http://zeromq.org/) and [Protobuf](https://developers.google.com/protocol-buffers/).
* [ign-utils](https://github.com/ignitionrobotics/ign-utils): General purpose
classes and functions with minimal dependencies.
* [sdformat](https://github.com/osrf/sdformat): World description format.

### Ignition Architecture

Documents describing the architecture of Ignition are listed here:

* [Ignition Gazebo Architecture](/docs/all/architecture)

## How to Contribute

### Reporting Bugs

**Before Submitting a Bug Report**

1. Check the [questions and answers forum](http://answers.gazebosim.org). Your issue may have already been resolved.
2. Determine [the repository](https://ignitionrobotics.org/docs/all/contributing#repository-list) which should receive the problem.
3. Search the repository's issues to see if the same or similar problem has
   been opened. If it has and the issue is still open, then add a comment to
   the existing issue. Otherwise, create a new issue.

**How to Submit a Good Bug Report**

Create an issue on the repository that is related to your bug, explain the
problem, and include additional details to help maintainers reproduce the
problem. Refer to the [Short, Self Contained, Correct (Compilable), Example
Guide](http://sscce.org/) as well as the following tips:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the exact steps which reproduce the problem** in as many details as possible. When listing steps, don't just say what you did, but explain how you did it.
* **Provide specific examples to demonstrate the steps.** Include links to files or projects, or copy/pasteable snippets, which you use in those examples.
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why**.
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem. See [Creating GIFs](https://ignitionrobotics.org/docs/all/contributing#creating-gifs) for GIF creation utilities.
* **If the problem wasn't triggered by a specific action**, describe what you were doing before the problem happened and share more information using the guidelines below.

Provide more context by answering these questions:

* **Did the problem start happening recently** (e.g. after updating to a new version) or was this always a problem?
* If the problem started happening recently, **can you reproduce the problem in an older version?** What's the most recent version in which the problem doesn't happen?
* **Can you reliably reproduce the issue?** If not, provide details about how often the problem happens and under which conditions it normally happens.
* If the problem is related to working with files, **does the problem happen for all files and projects or only some?** Does the problem happen only when working with local or remote files, with files of a specific type (e.g. only Collada or SDF), or with large files or files with very long lines? Is there anything else special about the files you are using?

Include details about your configuration and environment:

* **Which version of Ignition are you using?**?
* **What's the name and version of the OS you're using**?
* **Are you running Ignition in a virtual machine?** If so, which VM software are you using and which operating systems and versions are used for the host and the guest?

### Suggesting Enhancements

This section guides you through submitting an enhancement suggestion,
including completely new features and minor improvements to existing
functionality. Following these guidelines helps maintainers and the
community understand your suggestion and find related suggestions.

Before creating enhancement suggestions, please check [this
list](https://ignitionrobotics.org/docs/all/contributing#before-submitting-an-enhancement-suggestion) as you
might find out that you don't need to create one. When you are creating an
enhancement suggestion, please [include as many details as
possible](https://ignitionrobotics.org/docs/all/contributing#how-do-i-submit-a-good-enhancement-suggestion-).
When filling in the issue form for an enhancement suggestion, include the
steps that you imagine you would take if the feature you're requesting
existed.

#### Before Submitting An Enhancement Suggestion

* **Check if you're using the latest software version**. A more recent version may contain your desired feature.
* **Check if there's already [a library](https://ignitionrobotics.org/libs) which provides that enhancement.**
* **Determine [which repository the enhancement should be suggested in](https://ignitionrobotics.org/docs/all/contributing#repository-list).**
* **Perform a [cursory search](https://github.com/search?q=org%3Aignitionrobotics&type=Issues)** to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.
* **Ask on the [community forum](https://community.gazebosim.org) about your
feature.** Someone else might already have started, and you might be able to
help.

#### How Do I Submit A (Good) Enhancement Suggestion?

Enhancement suggestions are tracked as [GitHub
issues](https://help.github.com/en/github/managing-your-work-on-github/about-issues).
After you've determined [which repository](https://ignitionrobotics.org/docs/all/contributing#repository-list)
your enhancement suggestion is related to, create an issue on that
repository and provide the following information:

* **Use a clear and descriptive title** for the issue to identify the suggestion.
* **Provide a step-by-step description of the suggested enhancement** in as many details as possible.
* **Provide specific examples to demonstrate the steps**. Include copy/pasteable snippets which you use in those examples, as [Markdown code blocks](https://help.github.com/en/github/writing-on-github/creating-and-highlighting-code-blocks).
* **Describe the current behavior** and **explain which behavior you expected to see instead** and why.
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem. See [Creating GIFs](https://ignitionrobotics.org/docs/all/contributing#creating-gifs) for GIF creation utilities.
* **Explain why this enhancement would be useful** to most users and isn't something that can or should be implemented as a separate application.
* **Specify which version of Ignition you're using.**
* **Specify the name and version of the OS you're using.**

### Contributing Code

We follow a development process designed to reduce errors, encourage
collaboration, and make high quality code. Review the following to
get acquainted with this development process.

1. **Read the [Reporting Bugs](https://ignitionrobotics.org/docs/all/contributing#reporting-bugs) and [Suggesting Enhancements](https://ignitionrobotics.org/docs/all/contributing#suggesting-enhancements)** sections first.

1. **Fork the Ignition library** you want to contribute to. This will create
   your own personal copy of the library. All of your development should
   take place in your fork.
   - An important thing to do is create a remote pointing to the [upstream remote repository](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/configuring-a-remote-for-a-fork). This way, you can always check for modifications on the original repository and **always** keep your fork repository up to date.

1. **Choose a base branch.** If your changes will break API or ABI, then
   base your new branch off of `main`. If your changes don't break
   API/ABI and you would like them to be released to an existing release
   with major version `N`, then use branch `ign-<library>N` as the base.

1. **Work out of a branch** Always work out of a new branch, one that is not
   a release / main branch. This is a good habit to get in, and will make
   your life easier.

1. **Write your code.** This is the fun part, but is good to remember:
   - Always [sign your commits](https://docs.github.com/en/github/authenticating-to-github/signing-commits) (See the bullet about Developer Certificate of Origin in the [Process](https://ignitionrobotics.org/docs/all/contributing#process) section below)
   - Look at the existing code and try to maintain the existing style and pattern as much as possible
   - **Always** keep your branch updated with the original repository

1. **Write tests.** In most cases, a pull request will only be accepted if
   it has tests. See the [Writing Tests](https://ignitionrobotics.org/docs/all/contributing#writing-tests)
  section below for more information.

1. **Resolve compiler warnings.** Code must have zero compile warnings, or at least make sure your pull request is not adding new warnings.

1. **Follow the [style guide](https://ignitionrobotics.org/docs/all/contributing#style-guides).**

    Static code checking analyzes your code for bugs, such as potential memory
    leaks, and style. Most Ignition libraries use the `cppcheck` static code
    checker, and a modified version `cpplint`. Ubuntu
    users can install via:

        sudo apt-get install cppcheck

    To check your code, run the following script from the `build` folder of the project that you're working on.
    If you're working on *ignition-math*, for instance, the path for the folder should be something similar to `~/citadel_ws/build/ignition-math6`.
    The path example is assuming you followed [our installation instructions](/docs/citadel/install) using colcon.

    Then, run the script inside this folder:

        make codecheck

    This may take a few minutes to run. Fix all errors and warnings until
    the output looks like:

        Built target codecheck

    The tool does not catch all style errors. See the [code style](https://ignitionrobotics.org/docs/all/contributing#style-guides) section below for more information.

1. **(optional) Use clang-tidy for additional checks.**

    clang-tidy should return no errors when run against the code base.

    Ubuntu users can install via:

        sudo apt-get install clang-tidy-6.0 libclang-6.0-dev python-yaml

    clang-tidy can then be executed by running from the source dir:

        ./tools/clang_tidy.sh

    Address issues that are found.

    If you are feeling adventurous, you can experiment with adding additional checks to the `.clang-tidy` file by referencing the full list of options in the [clang-tidy documentation](http://clang.llvm.org/extra/clang-tidy/checks/list.html)

1. **Tests must pass.** You can check by running `make test` in
    your build directory. Running tests may take a bit of time, be patient.

1. **Write documentation.** Document all your code. Every class, function, member variable must have doxygen comments. All code in source files must have documentation that describes the functionality. This will help reviewers and future developers.

1. **Review your code.** Before submitting your code through a pull request,
   take some time to review everything line-by-line. The review process will
   go much faster if you make sure everything is perfect before other people
   look at your code.  There is a bit of the human-condition involved here.
   Folks are less likely to spend time reviewing your code if it's sloppy.

1. **Make small pull requests**

    A large pull request is hard to review, and will take a long time. It is
    worth your time to split a large pull request into multiple smaller pull
    requests. For reference, here are a few examples:

    * [Small, very nice](https://github.com/osrf/gazebo/pull/2789)

    * [Medium, still okay](https://github.com/osrf/gazebo/pull/2784)

    * [Too large](https://github.com/osrf/gazebo/pull/2776)

1. **Submit a pull request** to the Ignition library through GitHub when you're ready.

1. **Check Continuous integration**

    The moment you make a pull request, a few test jobs
    will be started. These jobs will build your branch on Linux, Mac and
    Windows, run all tests and check for warnings.

    Check the [Continuous Integration guide](https://ignitionrobotics.org/docs/all/ci)
    for information on how to interpret the results.

1. **Respond to reviewers.** At least two other people have to approve your pull request before it can be merged. Please be responsive to any questions and comments.

1. **Done, phew.** Once you have met all the requirements, your code will be merged. Thanks for improving Ignition!

### Tracking Progress

Ignition development progress is tracked publicly using a GitHub project board.
Using project boards ensures the community has visibility to what’s coming up,
external contributors can understand where they can help, and the reasoning behind
development decisions are visible to everyone.

Contributors should look at the ["Core development" board](https://github.com/orgs/ignitionrobotics/projects/3), though it's possible we may have other boards in our organization at various times.

#### Repositories

The following repositories from [Ignition](https://github.com/ignitionrobotics/) are tracked:

* [ign-cmake](https://github.com/ignitionrobotics/ign-cmake)
* [ign-common](https://github.com/ignitionrobotics/ign-common)
* [ign-fuel-tools](https://github.com/ignitionrobotics/ign-fuel-tools)
* [ign-gazebo](https://github.com/ignitionrobotics/ign-gazebo)
* [ign-gui](https://github.com/ignitionrobotics/ign-gui)
* [ign-launch](https://github.com/ignitionrobotics/ign-launch)
* [ign-math](https://github.com/ignitionrobotics/ign-math)
* [ign-msgs](https://github.com/ignitionrobotics/ign-msgs)
* [ign-physics](https://github.com/ignitionrobotics/ign-physics)
* [ign-plugin](https://github.com/ignitionrobotics/ign-plugin)
* [ign-rendering](https://github.com/ignitionrobotics/ign-rendering)
* [ign-sensors](https://github.com/ignitionrobotics/ign-sensors)
* [ign-tools](https://github.com/ignitionrobotics/ign-tools)
* [ign-transport](https://github.com/ignitionrobotics/ign-transport)
* [ign-utils](https://github.com/ignitionrobotics/ign-utils)
* [docs](https://github.com/ignitionrobotics/docs/)
* [design](https://github.com/ignitionrobotics/design)
* [ros_ign](https://github.com/ignitionrobotics/ros_ign)


New issues and pull requests, and issue and pull request statuses, from across the tracked repositories are all automatically synced with the same board.

Unfortunately, GitHub boards' cross-org support is poor.
The following repos can't be handled automatically and have to be manually tracked on the board along with the ones listed above:

* [sdformat](https://github.com/osrf/sdformat)
* [gazebo](https://github.com/osrf/gazebo)
* [homebrew-simulation](https://github.com/osrf/homebrew-simulation)
* [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/)
* [release-tools](https://github.com/ignition-tooling/release-tools)
* [gazebodistro](https://github.com/ignition-tooling/gazebodistro)

#### Board Columns

The project board columns are defined below:

* **Inbox**: New issues and pull requests
* **To do**: Issues ready to be worked on.
* **In progress**: Issues being worked on / pull requests being iterated on.
* **In review**: Pull requests under review / issues requesting feedback.
* **Done**: Closed issues / pull requests (cleared from time to time).

If a ticket is not on the board, it means that the core team is not planning
to work on it, but the community is welcome to do so.

#### Process

All Ignition team members actively:

* **Watch** all Ignition repositories to receive email notifications of new issues / pull requests
* Provide **feedback** to issues as soon as possible
* **Review** pull requests as soon as possible

  * Team members can review pull requests already under review or approved
  * Team members can provide some feedback without doing a full review

During weekly simulation meetings, the team:

* Discusses the **Done** column and archives all tickets
* Discusses the **In Review** column and makes sure all pull requests have reviewers
* Discusses the **In Progress** column
* Quickly **triages** the inbox, if there’s anything left in it

  * If we won’t work on an issue in the near future, **remove** it from the board.
    * If we think the issue should be tackled, label it **[help wanted](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22help+wanted%22&state=open&type=Issues)**.
  * If we’re going to work on an issue, move it to **To Do**, no assignment necessary initially.
  * If we won’t review a pull request in the near future, **close** it.

When opening a pull-request:

* **Labels** according to the targeted collection(s) (blueprint, citadel, etc) will be added automatically
* Add yourself as the **assignee**

  * Maintainers of each repo will be also automatically assigned

* It will be automatically triaged to the “Core development” **project board**
* Add **reviewers** as appropriate

  * If reviewers won’t be able to review soon, they can remove themselves and also let the original author know

Pull requests can be merged when:

* They have at least 1 approval from a member of the core team
* There are no unresolved comments
* CI is passing
* [Developer Certificate of Origin](https://developercertificate.org/) (DCO) check is passing

  * DCO is a declaration of ownership, basically saying that you created the contribution and that it is suitable to be covered under an open source license (not proprietary).
  * All you have to do is end your commit message with `Signed-off-by: Your Full Name <your.name@email.com>`
  * If your `user.name` and `user.email` configurations are set up in git, then you can simply run `git commit -s` to have your signature automatically appended.


Merging strategy:

* For internal contributions, give the original author some time to hit the merge button themselves / check directly with them if it’s ok to merge.
* Default to “squash and merge”
  * Review the pull request title and reword if necessary since this will be part of the commit message.
  * Make sure the commit message concisely captures the core ideas of the pull request and contains all authors' signatures.
* “Rebase and merge” when moving files (do a `git mv` as a separate commit).
* “Create a merge commit” when porting changes forward. "Rebase and merge" when porting backwards.
* Refrain from force-pushing while the PR is under review (which includes rebasing and squashing).

Porting changes across branches:

* Pull requests should target the lowest possible
  [supported version](https://ignitionrobotics.org/docs/all/releases) where the
  changes can be added in a backwards-compatible way (no API / ABI / behavior
  break in released branches).
* Periodically, a maintainer will **forward-port** changes to newer release
  branches all the way up to `main`.
* See [this list](https://github.com/ignitionrobotics/docs/blob/master/tools/branch_comparisons.md) to check if a branch needs porting.
* The merge forward can be done with `git merge` in order to keep the commit history
  and so it's easier to compare branches. For example:

        git checkout ign-<library>M
        git pull
        git checkout ign-<library>N
        git pull
        git checkout -b username/M_to_N_<date> # It's important to do this before `git merge`
        git merge ign-<library>M
        # Fix conflicts
        git commit -sam"Merge M into N"
        # Open pull request
        # Do not squash or rebase, create a merge commit

* In the rare event that a pull request needs to be backported (i.e. from a
  higher version to a lower version), use `git cherry-pick` instead of `git merge`,
  for example:

        git checkout ign-<library>N
        git pull
        git checkout ign-<library>M
        git pull
        git checkout -b N_to_M_<date>
        git cherry-pick <commits from verrsion N>
        # Fix conflicts
        git commit -sam"Backport from N to M"
        # Open pull request
        # Do not squash, rebase instead

## Writing Tests

Most Ignition libraries use [GTest](http://code.google.com/p/googletest) for
general testing and [QTest](http://doc.qt.io/qt-5/qtest.html) for GUI tests.
There are a few kinds of tests:

1. **Unit tests**: all classes should have corresponding unit tests. These live
in the same directory as the source code and are suffixed by `_TEST`.

1. **Integration tests**: tests which verify how many classes are working together go under the `tests/integration` directory.

1. **Regression tests**: tests which fix broken features go under `tests/regression` and are prefixed by the issue number on library's issue tracker.

1. **Performance tests**: tests that are designed to check performance
   characteristics, such as CPU or memory usage, go under `tests/performance`.

Before creating a new integration or performance test file, check the current
test files. If one closely matches the topic of your new code, simply add a new
test function to the file. Otherwise, create a new test file, and write your
test.

### Test Coverage

The goal is to achieve 100% line and branch coverage. However, this is not
always possible due to complexity, analysis tools misreporting
coverage, and time constraints. Try to write as complete of a test suite as
possible, and use the coverage analysis tools as guide. If you have trouble
writing a test please ask for help in your pull request.

Ignition CMake provides build target called `make coverage` that produces a code
coverage report. You'll need to have [lcov](http://ltp.sourceforge.net/coverage/lcov.php) and [gcov](https://gcc.gnu.org/onlinedocs/gcc/Gcov.html) installed.

1. In your `build` folder, compile with `-DCMAKE_BUILD_TYPE=Coverage`

    If using plain cmake:

        cd <path to build directory>
        cmake -DCMAKE_BUILD_TYPE=Coverage ..
        make

    If using a colcon workspace:

        cd <path to colcon workspace>
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Coverage --merge-install

1. Run a single test, or all the tests

    If using plain cmake:

       ./bin/UNIT_<TestName>_Test   # single test
        make test                   # all tests

    If using a colcon workspace:

      * Run single/specific test:

      ```
      colcon test --ctest-args -R <part of test name> --merge-install
      # or
      ./build/<package_name>/bin/UNIT_<TestName>_TEST
      ```

      * Run all tests for a single package:

      ```
      colcon test --packages-select <package_name> --merge-install
      ```

      * Run all tests for all packages in the workspace:

      ```
      colcon test --merge-install
      ```

    When using `colcon test`, all test results end up in the `log` directory.
    You can check `log/latest_test` to see the latest results.

1. Make the coverage report

        make coverage

1. View the coverage report

        firefox coverage/index.html

### Sanitizers

Sanitizers capture very detailed information during runtime about code quality issues and print them to stderr.

#### Install dependencies

Install and enable Colcon mixin package:

```bash
pip3 install colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

Install colcon-sanitizer-reports (Recommended):

```bash
git clone https://github.com/colcon/colcon-sanitizer-reports.git
cd colcon-sanitizer-reports
sudo python3 setup.py install
```

You can find more details [here](https://github.com/colcon/colcon-sanitizer-reports/blob/master/README.rst) about the instalation process.

#### Compiling the code and running the tests

Create a Ignition workspace for ASAN/TSAN related tasks. Refer to the [offical documentation to get
all the Ignition Robotics sources](https://ignitionrobotics.org/docs/latest/install_ubuntu_src#getting-the-sources).

First, compile all packages:

 - ASan
```bash
cd ~/ignition  # you will need to be exactly in this directory
colcon build --build-base=build-asan --install-base=install-asan \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    --mixin asan-gcc
```

 - TSan
```bash
# Equivalent command for tsan:
# You can either use different workspaces or the same one.
colcon build --build-base=build-tsan --install-base=install-tsan \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    --mixin tsan
```

**IMPORTANT:** Do not forget to pass ``-DCMAKE_BUILD_TYPE=Debug`` to
ensure that debugging symbols are generated. This allows ASAN/TSAN to
report files and line numbers in backtraces.

Once the compilation is finished, colcon will report that all packages have been compiled
but some had “stderr output”. This is fine.

 - To run the tests for ASan:
```bash
cd ~/ignition  # you will need to be exactly in this directory
colcon test --build-base=build-asan --install-base=install-asan \
    --event-handlers sanitizer_report+
```

 - To run tests for TSan
```bash
 cd ~/ignition  # you will need to be exactly in this directory
colcon test --build-base=build-tsan --install-base=install-tsan \
  --event-handlers sanitizer_report+
```

Omit the `--event-handlers` flag if you did not install colcon-sanitizer-reports.

Some tests may fail, this is OK. Once done, you can look at the test logs or
`sanitizer_report.csv`. Examples from tests logs:

```bash
cd ~/ignition  # you will need to be exactly in this directory
cd log/latest_test
# Displays three lines after the beginning of a ASAN reported issue.
grep -R '==.*==ERROR: .*Sanitizer' -A 3
```

Review [Appendix - ASAN/TSAN Issues Zoology](https://github.com/colcon/colcon-sanitizer-reports/blob/master/README.rst#appendix---asantsan-issues-zoology)
to review which kind of failures you can find in this kind of reports.

## Style Guides

You can check code for compliance by running the following command from
a build folder:

        make codecheck

In general, we follow [Google's style guide](https://google.github.io/styleguide/cppguide.html) and rules set forth by `cppcheck`. However, we have added some extras listed below.

1. **This pointer**
> All class attributes and member functions must be accessed using the `this->` pointer. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/src/Server.cc#L89).

1. **Underscore function parameters**
> All function parameters must start with an underscore. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/src/Server.cc#L173-L174).

1. **Do not cuddle braces**
> All braces must be on their own line. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/src/Server.cc#L245-L253).

1. **Multi-line code blocks**
> If a block of code spans multiple lines and is part of a flow control statement, such as an `if`, then it must be wrapped in braces. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/src/Server.cc#L295-L299).

1. **++ operator**
> This occurs mostly in `for` loops. Prefix the `++` operator, which is [slightly more efficient than postfix in some cases](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/src/EntityComponentManager_TEST.cc#L108).

1. **PIMPL/Opaque pointer**
> If you are writing a new class, it must use a private data pointer. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/include/ignition/gazebo/EntityComponentManager.hh#L682), and you can read more [here](https://en.wikipedia.org/wiki/Opaque_pointer).

1. **const functions**
> Any class function that does not change a member variable should be marked as `const`. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/include/ignition/gazebo/EntityComponentManager.hh#L453).

1. **const parameters**
> All parameters that are not modified by a function should be marked as `const`, except for "Plain Old Data" (`int`, `bool`, etc). This applies to parameters that are passed by reference, and pointer. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/include/ignition/gazebo/Model.hh#L111-L112).

1. **Pointer and reference variables**
> Place the `*` and `&` next to the variable name, not next to the type. For example: `int &variable` is good, but `int& variable` is not. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/include/ignition/gazebo/Model.hh#L96).

1. **Camel case**
> In general, everything should use camel case. Exceptions include SDF element names, and protobuf variable names. Here is an [example](https://github.com/ignitionrobotics/ign-gazebo/blob/de5b025968c9cf0cfbfd8a852458482e87c70c6c/include/ignition/gazebo/SdfEntityCreator.hh#L64-L65).

1. **Class function names**
> Class functions must start with a capital letter, and capitalize every word.
>
> `void MyFunction();` : Good
>
> `void myFunction();` : Bad
>
> `void my_function();` : Bad

1. **Variable names**
> Variables must start with a lower case letter, and capitalize every word thereafter.
>
> `int myVariable;` : Good
>
> `int myvariable;` : Bad
>
> `int my_variable;` : Bad

1. **No inline comments**
> `//` style comments may not be placed on the same line as code.
>
> `speed *= 0.44704;  // miles per hour to meters per second` : Bad

1. **Accessors must not start with `Get`**
> Member functions granting read access to protected or private data must look like a noun.
>
> `public: ::ServerConfig ServerConfig() const;` : Allowed
>
> `public: ServerConfig GetServerConfig() const;` : Not Allowed
>
> **Corner Cases**
>
> * A class name may conflict with an accessor function. For example, `Model(int)` would conflict with a `Model` class. In these cases, try to follow the `Noun` - `By` pattern. For example:
>
>   `ModelByName(const std::string &_name)` : Allowed
>
>   `ModelById(const int _id)` : Allowed
> * However, if the function does not have a parameter, it may be difficult to use the `Noun` - `By` pattern. In this case, qualifying the name of the class with its namespace should avoid the conflict.
> For example, if there is class called `Entity` in the `gazebo` namespace and we want to create an accessor called `Entity()`, we can do the following:
>
>   `public: gazebo::Entity Entity() const;` : Allowed
>
> * A template function that returns a data type may use a stand-alone `Get`. For example:
>
>   `public: template<typename T> T Get();` : Allowed

1. **Mutators must start with `Set`**
> Member functions granting write access to protected data must begin with `Set`.
>
> `public: void SetServerConfig(ServerConfig &_config);` : Allowed
>
> `public: void ServerConfig(::ServerConfig &_config);` : Not Allowed

## Appendix


### Releasing information for members of development team

[Releasing documentation](/docs/all/release) is available for the development team
members. Includes a general overview as well as detailed information about how to
run a new release.

### Creating GIFs

You can use [LICEcap](https://www.cockos.com/licecap/) to record GIFs on
macOS and Windows, and [Silent
Cast](https://github.com/colinkeenan/silentcast) or
[Byzanz](https://github.com/GNOME/byzanz) or
[Peek](https://github.com/phw/peek) on Linux.
