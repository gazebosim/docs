# Source Installation on MacOS

This tutorial will work for MacOS Mojave 10.14 and MacOS Catalina 10.15.

## Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

The easiest way to get the sources of all libraries is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and ign-gazebo in the right order
[colcon](https://colcon.readthedocs.io/en/released/) is recommended.
The colcon tool is available on all platforms using pip (or pip3, if pip fails).

## Python3 from homebrew

Tools and dependencies for Edifice can be installed using the [homebrew package manager](https://brew.sh/).
The homebrew tool can be installed by entering the following in a terminal:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

Edifice is compatible with Python3; it can be installed by running the following in a terminal:

```bash
brew install python3
```

## vcstool and colcon from pip

PIP is available on all platforms:

```bash
python3 -m pip install vcstool
```

```bash
python3 -m pip install -U colcon-common-extensions
```

## Getting the sources

The first step is to create a developer workspace in which `vcstool` and
`colcon` can work:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

All the sources of ignition-edifice are declared in a yaml file. Download
it to the workspace:

```bash
wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-edifice.yaml
```

Use `vcstool` to automatically retrieve all the Ignition libraries sources from
their repositories:

```bash
vcs import < collection-edifice.yaml
```

The src subdirectory should contain all the sources ready to be built.


## Install dependencies

Add `osrf/simulation` to Homebrew formulae

```bash
brew update
brew tap osrf/simulation
```

Install all dependencies:

Dependency for Ogre:

```bash
brew cask install xquartz
```

General dependencies:

```bash
brew install assimp boost bullet cmake cppzmq dartsim@6.10.0 doxygen eigen fcl ffmpeg flann freeimage freetype gflags google-benchmark gts ipopt irrlicht jsoncpp libccd libyaml libzzip libzip nlopt ode open-scene-graph ossp-uuid ogre1.9 ogre2.1 pkg-config protobuf qt qwt rapidjson ruby tbb tinyxml tinyxml2 urdfdom zeromq
```

`dartsim@6.10.0` and `qt5` are not sym-linked. To use those dependencies when building
`ignition-physics2` and `ignition-gui3`, run the following after installation to add them to `/use/local`:

```bash
# dartsim@6.10.0
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/dartsim@6.10.0
export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/usr/local/opt/dartsim@6.10.0/lib:/usr/local/opt/octomap/local
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/dartsim@6.10.0/lib/pkgconfig
# qt5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/qt
```

### Install compiler requirements

The Ignition Libraries require the Xcode 10 compiler on MacOS Mojave.

On Mac machines, gcc is acquired by installing Xcode command line tools.
The required version of Xcode for Edifice is Xcode 10.3, which can be downloaded from
[Apple Developer Site](https://developer.apple.com/download/more/).
You will need to sign in to your Apple account and download the Mojave version of
Xcode command line tools. Command line tools can also be obtained by downloading
Xcode from the Apple App Store (installing the full app may take over an hour).

## Building the Ignition Libraries in MacOS Catalina (10.15)

If you want to compile Ignition Libraries in MacOS Catalina (10.15) you will need to apply some patches in your filesystem:

 - `/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.15.sdk/System/Library/Frameworks/Ruby.framework/Headers/ruby/ruby/intern.h`

Create a file called `intern.patch` with the following content:

```patch
--- intern.h    2019-12-16 18:17:08.000000000 +0100
+++ /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.15.sdk/System/Library/Frameworks/Ruby.framework/Headers/ruby/ruby/intern.h
@@ -14,6 +14,10 @@
 #ifndef RUBY_INTERN_H
 #define RUBY_INTERN_H 1

+#if __cplusplus > 199711L
+#define register      // Deprecated in C++11.
+#endif  // #if __cplusplus > 199711L
+
 #if defined(__cplusplus)
 extern "C" {
 #if 0
```

Now we can apply the patch:

```{.sh}
sudo patch -p0 < intern.patch
```

 - `/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.15.sdk/System/Library/Frameworks/Ruby.framework/Headers/ruby/ruby/config.h`

Create a file called `config.patch` with the following content:

```
--- config.h    2019-12-16 18:19:13.000000000 +0100
+++ /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.15.sdk/System/Library/Frameworks/Ruby.framework/Headers/ruby/ruby/config.h
@@ -410,6 +410,6 @@
 #define RUBY_PLATFORM_CPU "x86_64"
 #endif /* defined __x86_64__ &&! defined RUBY_PLATFORM_CPU */
 #define RUBY_PLATFORM_OS "darwin19"
-#define RUBY_ARCH"universal-"RUBY_PLATFORM_OS
-#define RUBY_PLATFORM"universal."RUBY_PLATFORM_CPU"-"RUBY_PLATFORM_OS
+#define RUBY_ARCH "universal-" RUBY_PLATFORM_OS
+#define RUBY_PLATFORM "universal." RUBY_PLATFORM_CPU "-" RUBY_PLATFORM_OS
 #endif /* INCLUDE_RUBY_CONFIG_H */
```

Now we can appply the patch:

```{.sh}
sudo patch -p0 < config.patch
```

## Building the Ignition Libraries

Once the compiler and all the sources are in place it is time to compile them.
Start the procedure by changing into the workspace and listing the packages
recognized by `colcon`:

```bash
cd ~/workspace/
colcon graph
```

`colcon graph` should list the Ignition libraries with an
[interdependency diagram](https://colcon.readthedocs.io/en/released/reference/verb/graph.html#example-output).
If that is the case, then you are ready
to build the whole set of libraries:

```bash
colcon build --merge-install
```

To speed up the build process, you could also disable tests by using

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

To build a specific package with all its dependent packages:

```bash
colcon build --merge-install --packages-up-to PACKAGE_NAME
```

To build a single package:

```bash
colcon build --merge-install --packages-select PACKAGE_NAME
```

Visit [colcon documentation](https://colcon.readthedocs.io/en/released/#) to view more `colcon` build and test options.

If there are no errors, all the binaries should be ready to use.

## Using the workspace

The workspace needs to be sourced every time a new terminal is used.

Run the following command to source the workspace in bash:

```bash
. ~/workspace/install/setup.bash
```

Or in zsh:

```zsh
. ~/workspace/install/setup.zsh
```

This is the end of the source install instructions; head back to the [Getting started](getting_started.html)
page to start using Ignition!

## Uninstalling source-based install

If you need to uninstall Ignition or switch to a binary-based install once you
have already installed the library from source, navigate to your source code
directory's build folders and run `make uninstall`:

```bash
cd /workspace
sudo make uninstall
```

## Troubleshooting

### No rule to make target `/usr/lib/libm.dylib', needed by `lib/libignition-physics3-dartsim-plugin.3.1.0.dylib'.  Stop.

Try to run `brew outdated` followed by a `brew upgrade` may fix some of it.

### Unable to find `urdf_model.h` error

After installing all the dependencies and starting the build process, you may encounter an error that looks like this:

```bash
/Users/user/edifice_ws/src/sdformat/src/parser_urdf.cc:30:10: fatal error: 'urdf_model/model.h' file not found
#include <urdf_model/model.h>
         ^~~~~~~~~~~~~~~~~~~~
1 error generated.
make[2]: *** [src/CMakeFiles/sdformat9.dir/parser_urdf.cc.o] Error 1
make[1]: *** [src/CMakeFiles/sdformat9.dir/all] Error 2
make: *** [all] Error 2
Failed   <<< sdformat9	[ Exited with code 2 ]
```

First check if `urdfdom` and `urdfdom_headers` are installed by running:

```bash
brew install urdfdom urdfdom_headers
```

Then if the error persists, compile with the internal version of `urdfdom` by running:

```bash
colcon build --cmake-args -DUSE_INTERNAL_URDF=ON --merge-install
```

This command will ignore the system installation of `urdfdom` and use the internal version instead.

### Unable to load .dylib file

When running the `ign gazebo -s` command, an error like the one below may show up:

```bash
Error while loading the library [/Users/edifice/edifice_ws/install/lib//libignition-physics2-dartsim-plugin.2.dylib]: dlopen(/Users/edifice/edifice_ws/install/lib//libignition-physics2-dartsim-plugin.2.dylib, 5): Library not loaded: @rpath/libIrrXML.dylib
  Referenced from: /usr/local/opt/assimp/lib/libassimp.5.dylib
  Reason: image not found
[Err] [Physics.cc:275] Unable to load the /Users/edifice/edifice_ws/install/lib//libignition-physics2-dartsim-plugin.2.dylib library.
Escalating to SIGKILL on [Ignition Gazebo Server]
```

The issue is related to OSX System Integrity Protection(SIP). The workaround is to run `ign` with a different ruby then make sure that ruby is loaded.

```bash
brew install ruby

# Add the following to ~/.bashrc
export PATH=/usr/local/Cellar/ruby/2.6.5/bin:$PATH

# Source ~/.bashrc in terminal
. ~/.bashrc
```
