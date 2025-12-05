# Gazebo Roadmap

This page describes planned work for Gazebo. The set of planned
features and development efforts should provide insight into the overall
direction of Gazebo. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Gazebo Kura Roadmap

Starting in Gazebo Kura, we are introducing the idea of a secondary roadmap
which is a list of items the Gazebo PMC is looking to implement with the help
of the community. Committers on the Gazebo project will focus on the primary
roadmap, but will provide guidance and review to anyone in the community
willing to work on any of the items. Each item will have one or more point of
contacts (POCs) who are Gazebo committers and who will be responsible for
coordinating with a community member to get roadmap item implemented. The
POC(s) will provide a description of the roadmap item on a Github issue as well
some suggestions on how the feature should be implemented.

:::{note}
Note that the roadmap may be adjusted during the development cycle and some
items might move from the secondary to the primary roadmap depending on the
progress of primary tasks and availability of committers.
:::

### Primary Roadmap

:::{card}

#### Migrate Gazebo-classic tutorials

* Copy/adapt tutorials from <https://classic.gazebosim.org/tutorials>
* {bdg-primary}`POC:` https://github.com/azeey

#### Zenoh Refinement

* Continue refining the recent Zenoh integration work in gz-transport to make
it production ready
* [Add support for loading Zenoh configuration from
file](https://github.com/gazebosim/gz-transport/pull/733)
* Consider supporting shared memory transport over Zenoh
* {bdg-primary}`POC:` https://github.com/caguero

#### Deep dive into Gazebo performance issues

* Investigate and improve performance issues in core parts of Gazebo such as
the Entity Component Manager(ECM), libsdformat (world loading as well as
synchronization of models via the ECM), Physics system and associated plugins,
and performance of rendering for moderately complex worlds.
* {bdg-primary}`POC:` https://github.com/azeey

#### Improve rendering quality

* Explore how Gazebo can leverage modern rendering engines that provide higher
rendering fidelity.
* Potentially create a new way of integrating external rendering engines
running in a separate process.
* Curate models and worlds with high rendering fidelity for reference
* {bdg-primary}`POC:` https://github.com/iche033

#### Third-party plugin repository

* This is part of larger plan to support CI and binary releases for third-party
plugins, but initial task here will focus on creating a list of third-party
plugins similar to an awesome list.
* {bdg-primary}`POC:` https://github.com/azeey

#### Add MuJoCo as a new physics engine

* MuJoCo is known for its speed and superior handling of contact dynamics. In
addition it is actively maintained and has a thriving community of users.
Integrating MuJoCo as Gazebo's fourth physics engine will ensure that Gazebo
can offer a more efficient physics engine while also ensuring the long term
health of the project.
* {bdg-primary}`POC:` https://github.com/azeey
:::

### Secondary Roadmap

:::{card}
#### Expose an API similar to SimulationRunner

* Allow getting the ECM from either sim::Server or the new interface.
* Improve C++ API provided by convenience classes (e.g. `gz::sim::Model`,
`gz::sim::Link`) so that they are more complete for basic use cases. Users will
still need to access the ECM directly for more advanced user cases.
* {bdg-primary}`POCs:` https://github.com/arjo129, https://github.com/caguero

#### Low fidelity, faster-than-realtime simulation

* Full support for kinematic objects in dartsim and bullet
* Support a mode where we sim runs kinematics mode without having to change the
SDF.
* {bdg-primary}`POC:` https://github.com/scpeters

#### Dynamically create ros_gz bridges

* Improve the quality of life for ros_gz_bridge users by automatically creating
bridges of all known Gazebo topics with no or minimal configuration.
* {bdg-primary}`POC:` https://github.com/azeey

#### A list of polished examples using Gazebo

* Robot arms properly set up with ros2_control/moveit!
* Working examples of one or two robots of each robot category: arm, mobile,
humanoid, etc.
* {bdg-primary}`POC:` https://github.com/azeey

#### Improve multi-robot simulation with ROS

* Gazebo-classic ROS plugins (from gazebo_ros_pkgs) had a way to set namespaces
for each plugin, which could be used to simulate multiple robots easily. ros_gz
doesn’t have this kind of capability. Current solution is to use xacro to add
namespaces to Gazebo topic names.
* {bdg-primary}`POC:` https://github.com/azeey

#### Refactor initial start up sequence code

* Reduce chance of "gazebo is not working" issues stemming from
misconfiguration:
* https://github.com/gazebosim/gz-sim/issues/2962
* https://github.com/gazebosim/gz-sim/issues/2967
* {bdg-primary}`POC:` https://github.com/arjo129

#### Replace homebrew with Pixi in our macOS CI

* Due to the rolling nature of homebrew, the maintenance burden of building
bottles and the associated CI disruptions have become too much. Pixi on the
other hand can provide version locked binaries.
* {bdg-primary}`POC:` https://github.com/j-rivero

#### Build python docs and ensure that they are complete

* Host a sphinx autogenerated Python documentation for all of Gazebo's python
bindings
* {bdg-primary}`POC:` https://github.com/j-rivero

#### Provide an integration with ros2’s resource_retriever

* This would allow Gazebo can find assets from ROS packages and ROS tools such
as RViz can find assets known to Gazebo

* {bdg-primary}`POC:` https://github.com/mjcarroll
:::

:::{note}
Note that some of these roadmap items may be backported to older versions of
Gazebo on a best-effort basis if they do not break API and ABI.
:::

### Gazebo Kura Release Timeline (2026)

This is a tentative release timeline for the next Gazebo release.

| Event / Period | Date | Duration | Primary Focus |
| :--- | :--- | :--- | :--- |
| **Feature Freeze** | Friday, June 26 | ~1 month | Stabilize new features, merge pre-existing PRs. |
| **Code Freeze** | Monday, July 27 | ~1 month | Bug fixes only. |
| **Tutorial Party** | July 29 - August 13 | 2.5 Weeks | Community testing, tutorial verification. |
| **Internal QA/Demo Prep** | August 17 - 21 | 1 Week | Final package releases and feature demo preparation. |
| **Community Meeting** | Wednesday, August 26 | N/A | Feature showcase. |
| **Official Announcement** | Monday, August 31 | N/A | Release day! |

### Detailed Release Phase Guidelines

#### Feature Freeze

During the feature freeze period, we will not accept new feature pull requests to Gazebo.
This includes new features to the upcoming release as well as to currently stable versions. The focus will be reviewing open pull requests that are likely to be merged before the code freeze starts.
If you have a new feature you want to contribute, please open a PR before we go into feature freeze noting that changes can be made to open pull requests during the feature freeze period.

* **Initial Two-Week Review Period:** For the first two weeks following the feature freeze date, all branches remain frozen to *new-feature pull requests opened after the deadline*.
During this time, our focus is to review, merge, and forward-port eligible pull requests that were submitted *before* the freeze began.
* **Unfreezing Stable Branches:** After this initial two-week period, branches for older, stable Gazebo releases will be "unfrozen." New-feature pull requests targeting these branches can once again be opened and merged.
* **Protecting the `main` Branch:** To ensure the stability of the upcoming release, features merged into stable branches during the freeze **will not** be forward-ported to `main` until after the release is complete.

**Exception**: Branches shared between the upcoming release and older releases (which can happen to slowly changing packages such as `gz-cmake`) will remain fully frozen for the entire duration of the freeze.

#### Code Freeze

After the code freeze date, only critical bug fixes and documentation updates may be merged.

#### Tutorial Party

To make sure we have the best release possible the team goes over all the tutorials using pre-releases to check that everything is working as it should and make final tweaks.
We invite the community to join the party! If you are new to Gazebo, this would be a great opportunity to get your feet wet.
The tutorial party is also a great way for prospective interns and GSoC students to get familiar with Gazebo, meet the community, and close their first Github issue.
The tutorials are hosted on https://github.com/gazebosim/gazebo_test_cases and instructions will be posted on [Discourse](https://discourse.openrobotics.org/) a few weeks before the party starts.

#### Internal QA/Demo Preparation

Every year, the Gazebo team prepares a demo to showcase all the new features in the upcoming release. During this period, the team works on finalizing the demo as well as making final package releases.

#### Community Meeting

The demo mentioned above is presented to the community.

#### Official Announcement

The official release announcement is made on [Discourse](https://discourse.openrobotics.org/).

## Planned releases

Please see the [Releases](https://github.com/gazebosim/docs/blob/master/releases.md) for the timeline of and information about future distributions.

## Contributing to Gazebo

Looking for something to work on, or just want to help out? Here are a few
resources to get you going.

1. [How to contribute](contributing) guide.
1. [Feature comparision](/docs/citadel/comparison){.external} list. This page lists the
   feature gaps between Gazebo classic and Gazebo Sim.
1. Take a look at the various [libraries](/libs){.external}, and the issue tracker
   associated with each.
