<div class="warning">
WARNING: Current Windows support is experimental.
</div>

# Binary Installation on Windows 10

Most Gazebo packages are available in Windows 10 using the [conda-forge package manager](https://conda-forge.org/),
and the Gazebo feedstock recipes can be found [here](https://github.com/search?q=org:conda-forge+libgz&type=code).

Additionally, command line tools, the DART physics engine, and some tests are not currently supported in Windows.

In order to use `conda-forge`, you will need to
1. Install a [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
   Miniconda suffices. You will likely want to check the box to add `conda` to your `PATH`
   during the installation process so that you won't have to do this step manually.

2. Open a Windows command prompt, being sure to have `conda` added to your
   Windows `PATH` system environment variable (you may also need to open
   a new command prompt to see any `PATH` changes reflected).

  If you did not add Conda to your `PATH` environment variable
  during Conda installation, you may need to navigate to the
  location of `condabin` in order to use the `conda` command.
  To find `condabin`, search for "Anaconda Prompt" in the
  Windows search field near the Windows button, open it, run
  `where conda`, and look for a line containing the directory `condabin`.

3. Create and activate a new Conda environment:
  ```bash
  conda create -n gz-env
  conda activate gz-env
  ```
4. Install desired Gazebo packages you want to install based on your application. Packages with the prefix `libgz-`
   contain only the C++ libraries while the Python bindings are available separately as `gz-<package_name><#>-python`.
   To install both with a single command use `gz-<package_name><#>`.
   Thus you can use `gz-sim<#>` to fully install the latest version of Gazebo.
  ```bash
  conda install libgz-<package_name><#> --channel conda-forge
  ```
  Be sure to replace `<package_name>` with your desired package name (ie, common, msgs, etc.)
  and `<#>` with the release version.  If left unspecified, `conda-forge` will install the
  most recently stable release packages.  Be sure to check the
  [high level install instructions](install) for corresponding version numbers.

**Note**

You can view all available versions of a specific package with:
```bash
conda search libgz-<package_name>* --channel conda-forge
```
and view their dependencies with
```bash
conda search libgz-<package_name>* --channel conda-forge --info
```
and install a specific minor version with
```bash
conda install libgz-<package_name>=<major>.<minor>.<patch> --channel conda-forge
```
where `<major>` is the major release number, `<minor>` is the minor release number, and `<patch` is the patch release number.

## Uninstalling binary install

If you need to uninstall Gazebo or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
conda uninstall libgz-<package_name> --channel conda-forge
```

## Troubleshooting

See [Troubleshooting](/docs/garden/troubleshooting#windows)
