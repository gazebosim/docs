# Debian/Ubuntu versioning in nightly and prerelease binaries

Binary packages produced for prerelease and nightly builds have some
particularities to establish the priority among them nicely.

**Prereleases** for version `X.Y.Z` are created as candidates of the stable
release of `X.Y.Z` so the stable needs to have precedence over prereleases when
both are available through different repositories.

**Nightlies** for version `X.Y.Z` are snapshots of development code merged on
top of a released version `X.Y.Z`. This includes features and/or patches that will be shipped
into the next release. Therefore nightlies need to have precedence over stable
releases. Another use case for nightlies is to serve as continuous unstable
releases when preparing a new version `X.Y.Z`. In this scenario the stable
release should have precedence over the nightlies generated before it. To get
this precedence, the nighlty version uses the trick of setting the version to
`{X-1.99.99}` (i.e: if the version to release is `9.0.0`, the nightlies used before
the version will use `8.99.99`).

## Version schemes

**Prerelease** versioning scheme: `{upcoming_version}~pre{prerelease_version}`

 * `upcoming_version`: upstream version target for current prerelease series (e.g., `X.Y.Z`)
 * `prerelease_version`: prerelease version number in the series

**Nightly** uses the following versioning scheme: `{current_released_version}+git{date}+{nightly_revision}r{hash}-{nightly_revision}`

 * `current_released_version`: will be the latest version released available in
   the changelog file of the corresponding `*-release` repo. If the nightly is
   used for an upcoming release (for example, gazebo10 where X is 10) then {X-1}.99.99-1
   (gazebo10_9.99.99-1) form will be used until prereleases or final release.

 * `date`: timestamp YYYY-MM-DD

 * `hash`: git hash corresponding to code HEAD used in the nightly build.
    Used for information proposes.

 * `nightly_revision`:  revision number to apply to the nightly. It is also
   used to generate a new nightly using the same date timestamp.

## Versions when mixing stable, prerelease and nightly

Which version has priority when using prerelease and stable repositories?

 * packageA version: `1.0.0-1` (stable)
 * packageA prerelease: `1.0.1~pre1-1` (prerelease)
 * packageA prerelease: `1.0.1~pre2-1` (prerelease)
 * packageA version: `1.0.1-1` (stable)

 * Order: `1.0.1-1` > `1.0.1~pre2-1` > `1.0.1~pre1-1` > `1.0.0`

Which version has priority when using nightly and stable repositories?

 * packageA version: `0.99.99+git201501011r2212b5136299-1` (nightly)
 * packageA version: `1.0.0-1` (stable)
 * packageA version: `1.0.0-1+git20150303r6912b5136236-1` (nightly)
 * packageA version: `1.0.1-1` (stable)

 * Order: `1.0.1-1` > `1.0.0-1+git20150303r6912b5136236-1` > `1.0.0-1` > `0.99.99+git20150101r2212b5136299-1`

Which version has priority when using nightly, prerelease and stable repositories?

 * packageA version: `0.99.99+git20150101r2212b5136299-1` (nightly)
 * packageA prerelease: `1.0.0~pre1` (prerelease)
 * packageA version: `1.0.0~pre1+git20150101r2212b5136299-1` (nightly)
 * packageA prerelease: `1.0.0~pre2-1` (prerelease)
 * packageA version: `1.0.0-1` (stable)

 * Order: `1.0.0-1` > `1.0.0~pre2-1` > `1.0.0~pre1+git20150101r2212b5136299-1` > `1.0.0~pre1-1` > `0.99.99+git20150101r2212b5136299-1`

## MacOS versioning in unstable packages

Some Brew packages have unstable versions, snapshots of software not corresponding to an official release.
The version scheme is typically: `{current_released_version}~${osrf_revision}~{date}~{hash}`
 * `osrf_revision`: correspond to the revision number that identifies the
    packaging metadata being used in the Brew formulae. 
