# Installing Gazebo11 side by side with new Gazebo

## Supported Gazebo versions and Ubuntu platforms for this tutorial

This tutorial affects the Gazebo Classic users on Ubuntu `Jammy` (latest
Ubuntu release were Gazebo Classic packages supported by the Gazebo team)
that plan on migrating code to new Gazebo `Fortress`, `Garden` or `Harmonic.`

There is no guarantee that the Gazebo Classic version detailed in this document
will be updated in any case. Please consider the migration to the new Gazebo
as the recommended alternative.

## General context and migrations to new Gazebo

New versions of Gazebo can not be installed together with Gazebo Classic 
(aka `gazebo11`) since both use the `gz` command line tool. Trying to 
install `gz-garden` or `gz-harmonic` from `packages.osrfoundation.org` 
on a system that already has gazebo-classic installed from binaries 
will cause gazebo-classic and its dependencies to be uninstalled.

Having a side by side installation of both can make easier the recommended
migration to newer versions of Gazebo.

## Alternative approach for Gazebo Classic without the gz tool

A possible workaround could be to use Gazebo Classic without the `gz`
command or with a renamed `gz` command. This can be particularly
interesting since the ROS (Robot Operative System) wrapper known as
`gazebo_ros_pkgs` do not use the `gz` command but the `gzserver` and
`gzclient` commands.

Not shipping the `gz` command by default in a Gazebo Classic installation
would be a disruptive change for existing users so it should not be hosted in
`packages.osrfoundation.org`.

## Gazebo11 with a separate gz tool package

This alterntive approach has been implemented and packages for `jammy` are
hosted in the Open Robotics `gazebo11-gz-cli` PPA:
https://launchpad.net/~openrobotics/+archive/ubuntu/gazebo11-gz-cli

The PPA contains a `gazebo11` version that ships the `gz` executable in
an independent packages called `gazebo11-gz-cli` while the main `gazebo11`
package has a `gz11` command. Both are really symlinks to `gz-11.x.y`.

### How the packaging works for the gz command

The `gazebo11-gz-cli` is a soft dependency on `gazebo11`. It is installed
by default but it will be uninstalled when installing the new Gazebo 
packages and the effect is that the `gz` command for Gazebo Classic replaced 
by the `gz` tool from new Gazebo. The `gz11` command can be used instead of
the `gz` command to access to the same functionality.

## Installing the new packaging from the PPA

From an existing `gazebo11` installation (on `jammy` the Ubuntu official
repositories will install `11.10.2+dfsg-1` version) or a non existing
`gazebo11` installation the steps are the following:

```bash
sudo add-apt-repository ppa:openrobotics/gazebo11-gz-cli
sudo apt update
sudo apt-get install gazebo11
```

If `gazebo11` was installed before, it will be upgraded to the version in the
PPA. From this point, a new Gazebo installation for `fortress`, `garden` or
`harmonic` can be executed.

If a new Gazebo installation was installed before, the `gazebo11-gz-cli` package
won't be installed.
