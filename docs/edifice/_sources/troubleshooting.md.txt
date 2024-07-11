# Troubleshooting

## macOS

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
Escalating to SIGKILL on [Gazebo Sim Server]
```

The issue is related to OSX System Integrity Protection(SIP). The workaround is to run `ign` with a different ruby then make sure that ruby is loaded.

```bash
brew install ruby

# Add the following to ~/.bashrc
export PATH=/usr/local/Cellar/ruby/2.6.5/bin:$PATH

# Source ~/.bashrc in terminal
. ~/.bashrc
```

## Ignition libraries are not found

If you see this error message:

```bash
I cannot find any available 'ign' command:
	* Did you install any ignition library?
	* Did you set the IGN_CONFIG_PATH environment variable?
	    E.g.: export IGN_CONFIG_PATH=$HOME/local/share/ignition
```

You should set up the environment variable `IGN_CONFIG_PATH=/usr/local/share/ignition/`

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
Escalating to SIGKILL on [Gazebo Sim Server]
```

The issue is related to OSX System Integrity Protection(SIP). The workaround is to run `ign` with a different ruby then make sure that ruby is loaded.

```bash
brew install ruby

# Add the following to ~/.bashrc
export PATH=/usr/local/Cellar/ruby/2.6.5/bin:$PATH

# Source ~/.bashrc in terminal
. ~/.bashrc
```

## Ubuntu

### Unable to create the rendering window

If you're getting errors like "Unable to create the rendering window", it could
mean you're using an old OpenGL version. Gazebo Sim uses the Ogre 2
rendering engine by default, which requires an OpenGL version higher than 3.3.

This can be confirmed by checking the Ogre 2 logs at `~/.ignition/rendering/ogre2.log`,
which should have an error like:

"OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported. Please update your graphics card drivers."

You can also check your OpenGL version running:

    glxinfo | grep "OpenGL version"

You should be able to use Ogre 1 without any issues however. You can check if
that's working by running a world which uses Ogre 1 instead of Ogre 2, such as:

    ign gazebo -v 3 lights.sdf

If that loads, you can continue to use Ignition with Ogre 1, just use the
`--render-engine ogre` option.

To enable Ogre 2 support, you'll need to update your computer's OpenGL version.
As suggested on the Ogre logs, this may require updating your graphics card
drivers.

The Ogre 2 debs from the osrfoundation repository are built from a fork of
Ogre's `v2-1` branch with changes needed for deb packaging and allowing it to
be co-installable with Ogre 1.x. The code can be found here:

https://github.com/osrf/ogre-2.1-release

## Windows

### VisualStudioVersion is not set, please run within a Visual Studio Command Prompt.

When you try to compile Ignition Robotics you might see an error in your prompt like:

    VisualStudioVersion is not set, please run within a Visual Studio Command Prompt.

 - CMD
```bash
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
```

 - PowerShell:
```bash
pushd "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools"
cmd /c "VsDevCmd.bat&set" |
foreach {
  if ($_ -match "=") {
    $v = $_.split("="); set-item -force -path "ENV:\$($v[0])"  -value "$($v[1])"
  }
}
popd
```
