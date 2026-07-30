<div class="warning">
WARNING: Current Windows support is experimental.
</div>

# Binary Installation on Windows 11

Binaries for all the dependencies used by Gazebo can be found in the [conda-forge](https://conda-forge.org/)
package repository. The Gazebo buildfarm and these instructions uses the [Pixi](https://pixi.sh/) package manager.

## Install dependencies

1. Follow the Pixi installer instructions from https://pixi.sh/latest/ to install pixi.
   Once pixi has been installed, close the terminal session and start it again,
   which will ensure pixi is on the PATH.

2. Make a folder to initialize the pixi project. 
  Just make sure that the directory does not contain any spaces.
 Open up a command prompt and type the following
   ```bash
      cd C:\Users\%USERNAME%\ # 
      mkdir gz-ws
      cd gz-ws
      pixi init
   ```

   You should be seeing this

   ```bash
      ✔ Created C:\Users\USER\ws_gz\pixi.toml
   ```

3. Prepare the pixi.toml file
   Now that the pixi environment has been installed, you can add the packaged gazebo binary to it and some necessary environment variables for the QT library.

   Open up the pixi.toml file in your editor by choice or simple notepad.

   ```bash
   notepad pixi.toml
   ```

   Replace `[dependencies]` the following to the pixi.toml file

   ```
   [target.win-64.activation.env]
   QT_QPA_PLATFORM_PLUGIN_PATH="%CONDA_PREFIX%\\Library\\lib\\qt6\\plugins\\platforms"
   QML2_IMPORT_PATH="%CONDA_PREFIX%\\Library\\lib\\qt6\\qml"

   [dependencies]
   gz-sim = "10.*"
   ```

   Save and close the pixi.toml file, and then let pixi install and pull all the dependencies

   ```bash
   pixi install
   ```

You should now be able to launch gazebo normally:

```bash
gz sim --verbose
```

Alternativally launching the server and the client in two different terminales (after sourcing
in both the install scripts, the pixi shell and the QT env variables):

```bash
# Launch server in one terminal
gz sim -s

# In separate terminal, launch gui
gz sim -g
```

This is the end of the source install instructions; head back to the [Getting started](getstarted)
page to start using Gazebo!


## Uninstalling binary-based gazebo install

Uninstalling the gazebo binary is as simple as removing the full folder that was created with the pixi.toml and the installed gazebo binaries. 

In a command prompt, navigate to the root directory of where the folder is (dependend on the location you choose at the first step), and remove it.

   ```bash
      cd C:\Users\%USERNAME%\ 
      rmdir /s /q gz-ws
   ```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#windows)
