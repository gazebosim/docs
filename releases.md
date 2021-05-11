# Ignition Releases

This page details the set of past and planned releases.

## What is a Release?

A release of Ignition consists of a set of versioned [Ignition Libraries](/libs). Each library follows [semantic versioning](https://semver.org), and the major version of a library is guaranteed not to change with an Ignition release. This essentially means you can develop code against a release of Ignition without the worry of breaking changes being made upstream.

### Naming Scheme

An Ignition release follows the form "Ignition Codename", for example Ignition Acropolis. The codename is alphabetically increasing, and chosen to fall loosely within the architectural domain.

## Release List

| Name       | Date      | EOL date  | Notes |
|------------|-----------|-----------|-------|
| Acropolis  | Feb, 2019 | Sep, 2019 | EOL   |
| Blueprint  | May, 2019 | Dec, 2020 | EOL   |
| Citadel    | Dec, 2019 | Dec, 2024 | LTS   |
| Dome       | Sep, 2020 | Dec, 2021 |       |
| Edifice    | Mar, 2021 | Mar, 2022 |       |
| Fortress   | Sep, 2021 | Sep, 2026 | LTS   |
| Ignition-G | Sep, 2022 | Sep, 2024 |       |
| Ignition-H | Sep, 2023 | Sep, 2028 | LTS   |

## Library Versions

Ignition adheres to [semantic versioning](https://semver.org), with the
addition that ABI is considered part of the public interface. In summary:

* **Major** version increased when incompatible ABI/API changes are made.
* **Minor** version increased when functionality has been added in a
  backwards-compatible manner, but it may not be forward-compatible. That is,
  code compiled against minor X will work with minor Y, but code compiled
  against minor Y may not work with minor X.
* **Patch** version increased when backwards-compatible bug fixes are released.
* **Pre-release** version used before making stable releases, using the `~`
  separator.

### Deprecation strategy

Where possible, a tick-tock deprecation strategy is used during major version
increments. Deprecated code produces compile-time or runtime warnings (tick).
These warnings serve as notification to users that their code should be upgraded.
The next major release removes the deprecated functionality (tock).

Example of function `foo` deprecated and replaced by function `bar`:

Version | API
------- | ---
Ign-X   | void foo();
Ign-Y   | void foo() IGN_DEPRECATED(Y); <br> void bar();
Ign-Z   | void bar();

### Support lifecycle

Check out [this table](https://github.com/ignitionrobotics/docs/blob/master/tools/versions.md)
for a list of release and EOL dates for all versions of all libraries.
