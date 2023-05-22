# Gazebo Releases

This page details the set of past and planned releases.

## What is a Release?

A release of Gazebo consists of a set of versioned [Gazebo Libraries](/libs). Each library follows [semantic versioning](https://semver.org), and the major version of a library is guaranteed not to change with an Gazebo release. This essentially means you can develop code against a release of Gazebo without the worry of breaking changes being made upstream.

### Naming Scheme

A Gazebo release follows the form "Gazebo Codename", for example Gazebo Acropolis. The codename is alphabetically increasing, and chosen to fall loosely within the architectural domain.

## Release List

| Name                                                     | Date      | EOL date  | Notes |
|----------------------------------------------------------|-----------|-----------|-------|
| Gazebo-J                                                 | Sep, 2025 | Sep, 2030 | LTS   |
| Gazebo-I                                                 | Sep, 2024 | Sep, 2026 |       |
| [Harmonic](https://gazebosim.org/docs/harmonic)          | Sep, 2023 | Sep, 2028 | LTS   |
| [Garden](https://gazebosim.org/docs/garden)              | Sep, 2022 | Nov, 2024 |       |
| [Fortress](https://gazebosim.org/docs/fortress)          | Sep, 2021 | Sep, 2026 | LTS   |
| [Edifice](https://gazebosim.org/docs/edifice)            | Mar, 2021 | Mar, 2022 | EOL   |
| [Dome](https://gazebosim.org/docs/dome)                  | Sep, 2020 | Dec, 2021 | EOL   |
| [Citadel](https://gazebosim.org/docs/citadel)            | Dec, 2019 | Dec, 2024 | LTS   |
| [Blueprint](https://gazebosim.org/docs/blueprint)        | May, 2019 | Dec, 2020 | EOL   |
| [Acropolis](https://gazebosim.org/docs/acropolis)        | Feb, 2019 | Sep, 2019 | EOL   |

## Library Versions

Gazebo adheres to [semantic versioning](https://semver.org), with the
addition that ABI is considered part of the public interface. In summary:

* **Major** version increased when incompatible ABI/API changes are made.
* **Minor** version increased when functionality has been added in a
  backwards-compatible manner, but it may not be forward-compatible. That is,
  code compiled against minor `N` will work with minor `N+1`, but code compiled
  against minor `N+1` may not work with minor `N`.
* **Patch** version increased when backwards-compatible bug fixes are released.
* **Pre-release** version used before making stable releases, using the `~`
  separator.

### Deprecation strategy

Where possible, a tick-tock deprecation strategy is used during major version
increments. Deprecated code produces compile-time or runtime warnings (tick).
These warnings serve as notification to users that their code should be upgraded.
The next major release removes the deprecated functionality (tock).

Example of function `foo` deprecated and replaced by function `bar`:

Version     | API
----------- | ---
Ign-(N)     | void foo();
Ign-(N+1)   | void foo() IGN_DEPRECATED(N+1); <br> void bar();
Ign-(N+2)   | void bar();

### Support lifecycle

Check out [this table](https://github.com/gazebosim/docs/blob/master/tools/versions.md)
for a list of release and EOL dates for all versions of all libraries.

### Supported platforms

Platforms are defined as a combination of operating system and architecture.
For example, "Ubuntu Focal on amd64".

Each release is targeted at a specific set of platforms. A support level applies
to an entire Gazebo release, including all library versions within it. The
supported platforms for each release are listed [below](/docs/all/releases#platforms-by-release).

In general, there are three categories of support for a platform:

* **Official support**: Officially supported platforms are regularly tested on
  continuous integration and released as binary packages. Errors or bugs
  discovered in these platforms are prioritized for correction by the
  development team. Significant errors discovered in these platforms can impact
  release dates and we strive to resolve all known high priority errors in
  officially supported platforms prior to new version releases.
* **Best-effort support**: Platforms supported at best-effort may be regularly
  tested on continuous integration and / or released as binary packages. Errors
  may be present in released versions for these platforms. Known errors in
  best-effort platforms will be addressed subject to resource availability on a
  best-effort basis and may or may not be corrected prior to new releases.
* **No effort to support**: All platforms not included under official or
  best-effort support are considered not supported. If a release is functional
  in a platform that isn't supported at a point in time, there are no guarantees
  that it will remain so in the future. Pull requests addressing issues on these
  platforms may be considered if they don't break any supported platforms and
  reviewed as time allows.

# Platforms by Release

## Acropolis (End of Life)

Acropolis is supported on the platforms below.

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this moment. The `ign-gazebo`
package is still not available for Windows, the installation should be done from
source code.

## Blueprint (End of Life)

Blueprint is supported on the platforms below:

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave
     * Gazebo currently only works in headless mode

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this moment. The `ign-gazebo`
package is still not available for Windows, the installation should be done from
source code.

## Citadel

Citadel is supported on the platforms below.

These are the **officially** supported platforms:

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/ignition-tooling/release-tools/issues/595)
for the full status.

## Dome (End of Life)

Dome is supported on the platforms below.

These are the **officially** supported platforms:

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/ignition-tooling/release-tools/issues/297)
for the full status.

## Edifice (End of Life)

Edifice is supported on the platforms below.

These are the **officially** supported platforms:

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/ignition-tooling/release-tools/issues/421)
for the full status.

## Fortress

Fortress is supported on the platforms below.

These are the **officially** supported platforms:

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64
* Ubuntu Jammy on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/ignition-tooling/release-tools/issues/596)
for the full status.

## Garden

Garden is supported on the platforms below.

These are the **officially** supported platforms:

* Ubuntu Focal on amd64
* Ubuntu Jammy on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/597)
for the full status.

## Harmonic (Planned)

Harmonic is planned to be supported on the platforms below.
This list may change up to the release date.

These are the **officially** supported platforms:

* Ubuntu Jammy on amd64
* Ubuntu 24.04 on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/928)
for the full status.

---

If you or your company are interested in directly supporting, or
sponsoring the support of other platforms, please contact
`info@openrobotics.org`.
