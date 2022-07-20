# Migration Guide

Hello Gazebo community!!

As you know, a couple months ago it was announced that [we’d be retiring the “Ignition” name in favor of “Gazebo”.](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356) This migration guide will serve you help you execute the necessary changes in your own packages, and luckily it won’t be as troublesome as the move from Gazebo Classic!

# Overview

## Changes

So what’s happening in practice? In summary:

- Whenever the name `Ignition` or `ign` is used, the Gazebo counterpart (`Gazebo` or `gz`) will be used instead, preserving case
- `ign-gazebo` / `Ignition Gazebo` will become `gz-sim` / `Gazebo Sim`
- The Ignition logo will also be replaced by the Gazebo logo.

These changes will be made in:

- Websites
- GitHub organizations and repositories
- Documentation
- UIs
- Namespaces, command line tools, shared libraries, directories, APIs, files

This means that a bulk of the migration effort will involve intelligent find-and-replaces of filenames, directories, and source code.

You may also look at the [tracking GitHub issue](https://github.com/gazebo-tooling/release-tools/issues/698) if you need to trace any changes made to support the migration in the core libraries.

## Tick-tocks and Hard-tocks

This section provides just an overview of the different changes made, for a more detailed listing of tick-tocks, see the `[MIGRATION.md](http://MIGRATION.md)` file in each of the individual core libraries’ repositories. Additionally the migration pointers in a later section of this migration guide should help you get your packages ready and working with Gazebo.

Generally speaking, you should still be able to use either the Ignition counterpart or Gazebo counterpart for **most things** if you are using Garden, due to explicit tick-tocking logic written in the stack. Just note that using the Ignition counterpart will generally cause deprecation warnings to emit.

### Tick-tocks

Tick-tocks for the following are implemented, though not all of them will emit deprecation warnings. These tick-tocks are implemented either as aliases, or otherwise have some sort of redirection mechanism (e.g. symlinks, directory retargets, string replacements in source) to target the Gazebo counterpart instead.

Also, in the source code, most of these tick-tocks will have an associated comment calling out that they are deprecated, or have a `GZ_DEPRECATED()` macro call.

<details>
<summary>Tick-tocks</summary>

**Namespaces**

- Python namespaces
    - e.g. `ignition.math.XXX` → `gz.math.XXX`
- C++ namespaces
    - e.g. `ignition::gazebo::XXX` → `gz::sim::XXX`
- Message namespaces and packages
    - e.g. `ignition.msgs.XXX` → `gz.msgs.XXX`, `ignition/msgs/header.proto` → `gz/msgs/header.proto`

**Source**

- Class names, members, functions, and variables in public headers
    - e.g. `IgnitionFormatter` → `GzFormatter`
- Public headers
    - e.g. `include/ignition` → `include/gz`
- Plugins
    - e.g. `ignition::gazebo::systems::LiftDrag` → `gz::sim::systems::LiftDrag`
- Shared libraries
    - e.g. `libignition-gazebo-buoyancy-engine-system.so` → `gz-sim-buoyancy-engine-system`
- C++ Macros in public headers
    - e.g. `IGN_PARTITION` → `GZ_PARTITION`

**CMake and Packaging**

- CMake macros/functions
    - e.g. `ign_find_package()` → `gz_find_package()`
- CMake macro/function arguments
    - e.g. `NO_IGNITION_PREFIX` → `NO_PROJECT_PREFIX`
- CMake variables*
    - e.g. `IgnOGRE2_FOUND` → `GzOGRE2_FOUND`
    - Not every CMake variable is tick-tocked, but most of the ones that are used in downstream libraries are
- CMake packages found by `gz_find_package()`
    - e.g. `gz_find_package(IgnCURL)` → `gz_find_package(GzCURL)`
- Debian packages
    - e.g. `libignition-cmake3-dev` → `libgz-cmake3-dev`

**Misc.**

- Environment variables (names and values)
    - e.g. `IGN_GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH`
- Command line
    - e.g. `ign` → `gz`, `ign gazebo` → `gz sim`
- GUI QML
    - e.g. `IgnSpinBox` → `GzSpinBox`
- Topics* (typically in tests)
    - e.g. `/ignition/XXX` → `/gz/XXX`
    - **Note:** `/gazebo` is NOT migrated to `/sim`
- GitHub organizations and repositories
    - e.g. `ignitionrobotics` → `gazebosim` , `ign-cmake` → `gz-cmake`
- GitHub actions and workflows
    - e.g. `ignition-tooling` → `gazebo-tooling`
- Websites
    - e.g. `[ignitionrobotics.org](http://ignitionrobotics.org)` → `gazebosim.org`
- SDF and launch tags
    - e.g. `<ignition-gui>` → `<gz-gui>`

</details>

### Hard-tocks

There are some exceptions that have been hard-tocked instead, meaning that you **MUST** use the Gazebo counterpart. Using the Ignition counterpart will likely cause compilation or something else to break (unless it’s just a documentation change.)

<details>
<summary>Hard-tocks</summary>

**Namespaces**

- Ruby namespaces
    - e.g. `ignition/math` → `gz/math`

**Source**

- C++ Macros in private headers (typically something in a `detail/` directory)
    - e.g. `DETAIL_IGNITION_UTILS_WARN_RESUME` → `DETAIL_GZ_UTILS_WARN_RESUME`
- Class names, members, functions, and variables in private headers, or function-local scope
    - e.g. `kIgnAuthDomain` → `kGzAuthDomain`
- Install space
    - e.g. `install/share/ignition` → `install/share/gz`
- Header guards (where appropriate)
    - e.g. `IGNITION_COMMON_MATERIAL_HH_` → `GZ_COMMON_MATERIAL_HH_`

**CMake and Packaging**

- Most includeable CMake files in `gz-cmake`
    - e.g. `IgnUtils.cmake` → `GzUtils.cmake`
- Gazebo library CMake project names
    - e.g. `ignition-utils2` → `gz-utils2`
- Internally used CMake variables
    - e.g. `IGN_TRANSPORT_VER` → `GZ_TRANSPORT_VER`

**Misc.**

- Gazebo launchfiles and tags
    - e.g. `sim.ign` → `sim.gzlaunch`, `<ign`  → `<gz`
- Config and log paths (PENDING)
    - e.g. `~/.ignition/gui/log` → `~/.gz/gui/log`
- Fuel URL (PENDING)
    - e.g. https://fuel.ignitionrobotics.org
- Fuel cache paths (ALSO PENDING)
    - e.g. `~/.ignition/fuel` → `~/.gz/fuel`
- Library names in documentation and comments
    - e.g. Ignition Gazebo → Gazebo Sim
- `gz-launch` Websocket server
    - e.g. `ign.js` → `gz.js`

**Additionally**

- `gz-common` plugins are deprecated, use `gz-plugin` instead
- `gz-common` `SuppressWarning` is deprecated, use `gz-utils` instead

Also, anything that is internal to the core Gazebo libraries and not used in downstream libraries (e.g. header guards, private headers or source, tests, documentation) is hard-tocked.

</details>

## Untocks

A very small selection of things have not been migrated, mostly for backwards compatibility reasons (e.g. supporting Fortress.)

- Branch names for versions of Gazebo libraries targeting releases before Garden
    - e.g. `ign-cmake1`
- Some links
    - e.g. [https://osrf-migration.github.io/ignition-gh-pages](https://osrf-migration.github.io/ignition-gh-pages)
- Fuel user agent related
    - e.g. `X-Ign-Resource-Version`, `IgnitionFuelTools`

# Migration

## Overview

The following migration guidelines are just that—guidelines and suggestions for how to get your package migrated, that hopefully should cover most general cases.

Just keep in mind the overarching goal of replacing every Ignition counterpart (`IGN`, `Ign`, `Ignition`, `ign`, `ignition`) to the Gazebo counterpart (`GZ`, `Gz`, `gz`).

### Recommendations

What might help greatly is to:

- Make ample use of regex and `sed`
    - The migration guide will be giving suggestions for what regex expressions to use, but **always review before executing the replace**
- Pay attention to case!!
- Use an editor to review changes, and source control to allow for easy rollbacks
- Pay attention to compilation warnings/errors, which usually (but not always) give suggestions for what to replace!
- If in doubt, trace the change to changes listed in the [tracking issue](https://github.com/gazebo-tooling/release-tools/issues/698)

Also, if you are building the Gazebo stack from source, you should do a clean, fresh rebuild and install. Delete your `build` and `install` directories, and run the build with `--merge-install`.

### Gotchas

Most of the migration effort you should do is mainly finding and replacing instances of Ignition-related terms with the Gazebo-related one.

However, here are a lot of edge cases that make it difficult to create a script to handle the migration (leading to errors or bugs), so before the migration steps are suggested, it would be good to know some of these cases.

- Case-sensitivity not respected
    - e.g. `IGNITION_ADD_PLUGIN` → `gz_ADD_PLUGIN`
- Greedy matches
    - e.g. `Align` → `Algz`, `unsigned` → `unsgzed`, `signal` → `sgzal`
- Not migrating “gazebo” to “sim”
    - e.g. `ign-gazebo` → `gz-gazebo` (it should be `gz-sim`)
- Matching `ign` before `ignition`
    - e.g. `ignition-cmake3` → `gzition-cmake3`
- Migrating a library that should not be migrated
    - e.g. `ignition-cmake2` → `gz-cmake2`
- Grammar
    - e.g. “an Ignition library” → “**an** Gazebo library”

Additionally, take note of the following behaviors:

- Config files are not overwritten, so you might need to delete or manually migrate Ignition-counterpart references in your config files to get them to point to the appropriate Gazebo-counterpart locations
- Because the install spaces have changed, if you have hard-coded locations involving `ignition`, you might need to migrate them to `gz`

## Migration Steps

It’s important to execute the steps within each of the individual sections in-order! The steps will typically go from specific to general.

### Migrate Files and File References

1. In your package root, look for files and directories matching the (case-insensitive) pattern of `ign(ition)?[_|-]gazebo`, and migrate `ign` / `ignition` appropriately to `gz`, and `gazebo` to `sim`
2. In your package root, look for files and directories with (case-insensitive) `ign` / `ignition`, and move them to `gz`, matching case
3. Update all internal references to the migrated files and directories in (1) and (2) in your package

### Migrate CMake

In `CMakeLists.txt` files (and their references in your source files!):

**Variables and macro/function calls**

```cpp
Find: IGN(ITION)?_GAZEBO
Replace: GZ_SIM

Find: ign(ition)?_gazebo
Replace: gz_sim

Find: IGN(ITION)?_
Replace: GZ_

Find: ign(ition)?_
Replace: gz_
```

**Includes**

```cpp
Find: include\(Ign
Replace: include(Gz

Find: include\(ign
Replace: include(gz

Find: gz_find_package\(ign-
Replace: gz_find_package(gz-

Find: gz_find_package\(Ign(ition)?
Replace: gz_find_package(Gz-
```

**Project Names**

```cpp
Find: ignition-gazebo
Replace: gz-sim

Find: ignition-
Replace: gz-
```

**Note:** Be wary that sometimes CMake arguments trickle down to the source files, so ensure you also migrate them appropriately

### Migrate Macros and Environment Variables

- [Helpful list of env var migrations](https://github.com/gazebo-tooling/release-tools/issues/734)
- [Helpful list of macro migrations](https://github.com/gazebo-tooling/release-tools/issues/737) (see the toggled drop-down block, some are skipped!)

Migrate source macros and environment variables

```cpp
Find: IGN(ITION)?_GAZEBO
Replace: GZ_SIM

Find: ign(ition)?_gazebo
Replace: gz_sim

Find: IGN(ITION)?_
Replace: GZ_

Find: ign(ition)?_
Replace: gz_
```

For environment variables, you can use the same approach as with macros, but pay attention to the values stored in the environment variables! (e.g. paths.)

Additionally, the logging macros have also been migrated! Migrate any uses!

- `ignerr` -> `gzerr`
- `ignwarn` -> `gzwarn`
- `ignmsg` -> `gzmsg`
- `igndbg` -> `gzdbg`
- `ignlog` -> `gzlog`
- `ignLogInit` -> `gzLogInit`
- `ignLogClose` -> `gzLogClose`
- `ignLogDirectory` -> `gzLogDirectory`

### Migrate SDF

In `.sdf` files:

```cpp
Find: <ignition
Replace: <gz

Find: </ignition
Replace: </gz

Find: ignition:type
Replace: gz:type
```

Some examples:

- `<gz:odometer`
- `<gz-gui`

### Migrate Plugins and Shared Libraries

The plugin finder is able to find plugins even if their filenames are stripped of `lib` and `.so`.

In `.sdf` files and source files (e.g. `.cc`):

```cpp
Find: ign(ition)?-gazebo([^. ]*)\.so
Replace: gz-sim\2

Find: ign(ition)?([^. ]*)\.so
Replace: gz\2

Find: ignition::gazebo
Replace: gz::sim

Find: ignition::
Replace: gz::
```

### Migrate Bindings

In Python files (e.g. `.py`)

```cpp
Find: ignition.gazebo
Replace: gz.sim

Find: ignition.
Replace: gz.
```

In Ruby files (e.g. `.i`, `.rb`)

```cpp
Find: ign(ition)?/
Replace: gz/
```

### Migrate Messages

In your message definitions

```cpp
Find: ign(ition)?\.gazebo
Replace: gz.sim

Find: ign(ition)?/gazebo
Replace: gz/sim

Find: ign(ition)?\.
Replace: gz.

Find: ign(ition)?/
Replace: gz/
```

### Migrate Headers and Sources

Sweeping checks everywhere (pay special attention to reviewing these!)

**Headers**

```cpp
Find: #include\s*([<"])ign(ition)?/gazebo
Replace: #include \1gz/sim

Find: #include\s*([<"])ign(ition)?/
Replace: #include \1gz/

// Note: You should be wary of the IGNITION GAZEBO case for the following
// and adjust accordingly
Find: #([^\s]*)\s+(.*)IGN(?:ITION)?_(.*)_(H+)_(.*)$
Replace: #$1 $2GZ_$3_$4_$5

Find: #endif\s*// GZ(.*)_H
Replace: #endif  // GZ$1_H
```

**Namespaces**

```cpp
Find: namespace\s*ignition
Replace: namespace gz

Find: namespace\s*gazebo
Replace: namespace sim

Find: ignition::gazebo
Replace: gz::sim

Find: Ignition::Gazebo
Replace: Gz::Sim

Find: ignition::
Replace: gz::

Find: Ignition::
Replace: Gz::
```

**Everything Else**

You probably want to manually inspect:

- `Ign`
- `ignition`

### Double-Check

These might be useful to double-check if you have any lingering stuff, or erroneously migrated instances

You should match these **case-insensitively**.

**Errors**

- `gz-gazebo`
- `gzition`
- `an gz`

**Remaining Ign**

- `\.ign(ition)?`
- `ign(ition)?[-_]`

### Migrate Your CLI Usage

Where you used to use:

```cpp
ign gazebo shapes.sdf
```

Now you should use:

```cpp
gz sim shapes.sdf
```

Notice that the `gazebo` verb is **deprecated**.