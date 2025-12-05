# Gazebo Roadmap

This page describes planned work for Gazebo. The set of planned
features and development efforts should provide insight into the overall
direction of Gazebo. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Gazebo Kura Roadmap

Coming soon...

<!-- Note that some of these roadmap items may be backported to older versions of Gazebo -->
<!-- on a best-effort basis if they do not break API and ABI. -->

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

During the feature freeze period, we will not accept new features to Gazebo.
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
