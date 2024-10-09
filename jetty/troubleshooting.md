# Troubleshooting

## Gazebo libraries are not found
If you see this error message:

```bash
I cannot find any available 'gz' command:
	* Did you install any Gazebo library?
	* Did you set the GZ_CONFIG_PATH environment variable?
	    E.g.: export GZ_CONFIG_PATH=$HOME/local/share/gz
```

You should set up the environment variable:

```
# replace <path_to_install_dir> to your Gazebo installation directory
GZ_CONFIG_PATH=<path_to_install_dir>/share/gz/
```

## macOS

###  Maximum number of open files reached `ulimit` error
When installing using homebrew, you may see the following error message:

```bash
Error: The maximum number of open files on this system has been reached. Use 'ulimit -n' to increase this limit.
```

As suggested in the error message, run the command below and check the output. The default value is set to `256` which is too low.
```bash
ulimit -n
```

Run the following command to increase the open files `ulimit` and then proceed with the homebrew install:
```bash
ulimit -n 10240
```

### Unable to find `urdf_model.h` error
After installing all the dependencies and starting the build process, you may encounter an error that looks like this:

```bash
/Users/user/jetty_ws/src/sdformat/src/parser_urdf.cc:30:10: fatal error: 'urdf_model/model.h' file not found
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
When running the `gz sim -s` command, an error like the one below may show up:

```bash
Error while loading the library [/Users/jetty/jetty_ws/install/lib//libgz-physics6-dartsim-plugin.6.dylib]: dlopen(/Users/jetty/jetty_ws/install/lib//libgz-physics6-dartsim-plugin.6.dylib, 5): Library not loaded: @rpath/libIrrXML.dylib
  Referenced from: /usr/local/opt/assimp/lib/libassimp.5.dylib
  Reason: image not found
[Err] [Physics.cc:275] Unable to load the /Users/jetty/jetty_ws/install/lib//libgz-physics6-dartsim-plugin.6.dylib library.
Escalating to SIGKILL on [Gazebo Sim Server]
```

The issue is related to OSX System Integrity Protection (SIP). The workaround is to run `gz` with a different ruby then make sure that ruby is loaded.

```bash
brew install ruby

# Add the following to ~/.bashrc
export PATH=/usr/local/Cellar/ruby/2.6.5/bin:$PATH

# Source ~/.bashrc in terminal
. ~/.bashrc
```

### No rule to make target `'/usr/lib/libm.dylib', needed by 'lib/libgz-physics6-dartsim-plugin.6.1.0.dylib'. Stop.`
Try to run `brew outdated` followed by a `brew upgrade` may fix some of it.

## Ubuntu

### Out of memory issues

There can be out of memory issue during the compilation of Gazebo, especially during
compilation of gz-physics. To prevent out of memory issues you can restrict the number of jobs:

```bash
MAKEFLAGS="-j<Number of jobs> " colcon build --executor sequential
```

### Problems with dual Intel and Nvidia GPU systems

If you are using a dual Intel/Nvidia system it could be the case that the
simulator is being run under Intel instead of using the Nvidia GPU. Bugs can
vary but there could problems with shadows, incorrect laser scans or other
rendering related issues.

#### prime-select command line tool

Hybrid Intel/Nvidia systems can be configured using the command line tool prime-select.
One option is to use always Nvidia:

    sudo prime-select nvidia
    # logout user session and login again

Other option is to configure the render offload for OpenGL applications to use
Nvidia. This means that your X screen and all normal applications are handled
by the Intel GPU, but all OpenGL applications that you start from the terminal
(including Gazebo) are rendered on the Nvidia GPU.

    # place the lines in your .bashrc if you want the change to be permanent
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    # logout user session and login again

#### nvidia-settings GUI tool

nvidia-settings is a GUI program that helps to configure the options for the Nvidia
graphic cards and includes some controls for hybrid Intel/Nvidia:

The section "PRIME Profiles" can be used to select that the Nvidia card controls
all the GUI applications by selecting "NVIDIA (Performance Mode)".

The "Application Profiles" can control the use of the Nvidia GPU per application.

### Unable to create the rendering window

If you're getting errors like "Unable to create the rendering window", it could
mean you're using an old OpenGL version. Gazebo Sim uses the Ogre 2
rendering engine by default, which requires an OpenGL version higher than 3.3,
preferrably 4.3+.

This can be confirmed by checking the Ogre 2 logs at `~/.gz/rendering/ogre2.log`,
which should have an error like:

    "OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported. Please update your graphics card drivers."

You can also check your OpenGL version running:

    glxinfo | grep "OpenGL version"

To enable Ogre 2 support, you'll need to update your computer's OpenGL version.
As suggested on the Ogre logs, this may require updating your graphics card
drivers.

If you still run into OpenGL issues when running Gazebo with Ogre 2, it could
be that certain extensions are not supported by your driver or you are running
inside a virtual machine. In this case, you can try disabling DRI:

    export LIBGL_DRI3_DISABLE=1

or force software rendering

    export LIBGL_ALWAYS_SOFTWARE=1

If you are using MESA drivers, you can also try overriding the OpenGL version

    export MESA_GL_VERSION_OVERRIDE=3.3

The Ogre 2 debs from the osrfoundation repository are built from a fork of
Ogre's `v2-3` branch with changes needed for deb packaging and allowing it to
be co-installable with Ogre 1.x. The code can be found here:

https://github.com/osrf/ogre-2.3-release

You should be able to use Ogre 1 without any issues however. You can check if
that's working by running with Ogre 1 instead of Ogre 2, such as:

    gz sim -v 3 shapes.sdf --render-engine ogre

If that loads, you can continue to use Gazebo with Ogre 1, just use the
`--render-engine ogre` option.

### Wayland issues

There's an issue with the interaction of Ogre and Qt in Gazebo that prevents wayland from
working properly. You might see an error message like the one below:

```
Unable to create the rendering window: OGRE EXCEPTION(3:RenderingAPIException): currentGLContext was specified with no current GL context in GLXWindow::create at ./RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXWindow.cpp (line 165)
```

A workaround is to set `QT_QPA_PLATFORM=xcb`. e.g.:

```
QT_QPA_PLATFORM=xcb gz sim -v 4 shapes.sdf
```

Another workaround to try is to make sure Gazebo is launched with
XWayland by unsetting the `WAYLAND_DISPLAY` environment variable, e.g.

```sh
env -u WAYLAND_DISPLAY gz sim -v 4 shapes.sdf
```

### EGL warnings

On startup, Gazebo prints out EGL warning messages like the one below:

```
libEGL warning: DRI2: failed to create dri screen
```

This message is printed out by Ogre 2 on initialization when it enumerates
through devices to query for EGL support. The warning is fine to ignore. Gazebo
should continue to operate without any issues.

### Network Configuration Issue
Improper network configuration can cause Gazebo to open a window and then become unresponsive. This issue can be diagnosed by running ```gz sim -v 4 shapes.sdf``` and checking for the following output:

```[GUI] [Dbg] [Gui.cc:343] GUI requesting list of world names. The server may be busy downloading resources. Please be patient.```

To fix this, enable multicast by following the steps at [ROS Enable Multicast](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast).


## Windows

### VisualStudioVersion is not set, please run within a Visual Studio Command Prompt.
When you try to compile Gazebo you might see an error in your prompt like:

    VisualStudioVersion is not set, please run within a Visual Studio Command Prompt.

In this case execute one of the following commands:
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

### Many errors from setuptools when running colcon
When calling `colcon` to build the packages, you may face a wall of Python errors similar to this one:

```
> colcon graph
Traceback (most recent call last):
  File "<string>", line 1, in <module>
ModuleNotFoundError: No module named 'setuptools.extern'
[10.385s] colcon.colcon_core.package_identification ERROR Exception in package identification extension 'python_setup_py' in 'conda\Lib\site-packages\adodbapi': Command '['D:\\programovani\\gz-ws\\conda\\python.exe', '-c', "import sys;from setuptools.extern.packaging.specifiers import SpecifierSet;from distutils.core import run_setup;dist = run_setup(    'setup.py', script_args=('--dry-run',), stop_after='config');skip_keys = ('cmdclass', 'distclass', 'ext_modules', 'metadata');data = {    key: value for key, value in dist.__dict__.items()     if (        not key.startswith('_') and         not callable(value) and         key not in skip_keys and         key not in dist.display_option_names    )};data['metadata'] = {    k: v for k, v in dist.metadata.__dict__.items()     if k not in ('license_files', 'provides_extras')};sys.stdout.buffer.write(repr(data).encode('utf-8'))"]' returned non-zero exit status 1.
```

The messages are quite cryptic and do not point at the root cause. The root cause is that you have probably created a conda env directory in the same directory where you have the `src` directory containing the Gazebo sources (you have probably used `conda create --prefix ...` to create the env directory at a non-default destination).

The solution is to move the conda env directory one level up.

E.g., this is the problematic folder structure:

```
gz-ws\
  conda\  # The conda env
  src\    # The Gazebo sources
    gz-sim\
```

To fix it, change the structure to something like this:

```
gz-ws\
  src\
    gz-sim\
conda\
```
