<div class="warning">
WARNING: Current Windows support is experimental.
</div>

# Binary Installation on Windows 11

Binaries for all the dependencies used by Gazebo can be found in the [conda-forge](https://conda-forge.org/)
package repository. The Gazebo buildfarm and these instructions use the [Pixi](https://pixi.sh/) package manager.

## Install dependencies

1. Follow the Pixi installer instructions from https://pixi.sh/latest/ to install pixi.
   Once pixi has been installed, close the terminal session and start it again,
   which will ensure pixi is on the PATH.

2. Make a folder to initialize the Pixi project. 
  Just make sure that the directory does not contain any spaces.
 Open up a command prompt and type the following
   ```bash
      cd C:\Users\%USERNAME%\ 
      mkdir gz-ws
      cd gz-ws
      pixi init
   ```

  You should see this:

   ```bash
      ✔ Created C:\Users\USER\ws_gz\pixi.toml
   ```

3. Prepare the pixi.toml file
   Now that the Pixi environment has been installed, you can add the packaged gazebo binary to it and some necessary environment variables for the QT library.

   Open up the pixi.toml file in your editor of choice or simple notepad.

   ```bash
   notepad pixi.toml
   ```

   Replace `[dependencies]` with the following in the pixi.toml file

   ```
   [target.win-64.activation.env]
   QT_QPA_PLATFORM_PLUGIN_PATH="%CONDA_PREFIX%\\Library\\lib\\qt6\\plugins\\platforms"
   QML2_IMPORT_PATH="%CONDA_PREFIX%\\Library\\lib\\qt6\\qml"

   [dependencies]
   gz-sim = "10.*"
   ```

   Save and close the pixi.toml file, and then let Pixi pull all the dependencies and install Gazebo.

   ```bash
   pixi install
   ```


You should now be able to launch gazebo normally within a pixi shell:

```bash
pixi shell
gz sim -v4 shapes.sdf
```

or do a pixi run which is the same thing but just one line:

```bash
pixi run gz sim -v4 empty.sdf
```

> Note, currently just running `gz sim` directly doesn't work due to [this issue](https://github.com/gazebosim/gz-sim/issues/3859), so make sure to to always assign a world sdf file until this has been solved. 


This is the end of the binary install instructions; head back to the [Getting started](getstarted)
page to start using Gazebo!


## Uninstalling binary-based gazebo install

Uninstalling the Gazebo binary is as simple as removing the full folder that was created with the Pixi environment. 

In a command prompt, navigate to the root directory of where the folder is (depending on the location you chose at the first step), and remove it.

   ```bash
      cd C:\Users\%USERNAME%\ 
      rmdir /s /q gz-ws
   ```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#windows)
