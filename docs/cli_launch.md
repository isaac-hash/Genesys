# CLI Command: `genesys launch`

The `genesys launch` command is a powerful wrapper for `ros2 launch` that adds convenience features for launching nodes and systems in a Genesys workspace.

## Usage

```bash
genesys launch <target> [launch_arguments]
```
or
```bash
genesys launch --all
```

## Description

This command handles sourcing the workspace automatically before executing the launch file.

### Launching a Specific File

You can specify a launch file using the `<package_name>:<launch_file.py>` format.

**Example:**
```bash
genesys launch my_package:my_launch.py
```

If you omit the launch file name, it defaults to `<package_name>_launch.py`.

**Example:**
```bash
genesys launch my_package
```
This will attempt to launch `my_package_launch.py` from the `my_package` package.

### Launching Everything with `--all`

The `--all` flag provides a powerful way to launch a distributed system. When used, `genesys launch` will search every package in your `src/` directory for a file named `default.launch.py`. It then generates a single master launch file that includes all of the found `default.launch.py` files and executes it.

This allows each package to define its own default entry point, and you can bring up your entire system with one command.

**Example:**
```bash
genesys launch --all
```

### Passing Launch Arguments

You can pass arguments to your launch files just as you would with `ros2 launch`.

**Example:**
```bash
genesys launch my_package:my_launch.py log_level:=debug use_sim_time:=true
```
