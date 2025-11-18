# CLI Command: `genesys build`

The `genesys build` command compiles your workspace using `colcon`, the standard build tool for ROS 2.

## Usage

```bash
genesys build [options]
```

## Options

- **`--packages <pkg1> <pkg2>...`** or **`-p <pkg1>...`**:
  Build only the specified packages. If omitted, all packages in the workspace are built.

- **`--persist`**:
  After a successful build, this flag will add the command to source your workspace's `install/setup.bash` file to your shell's startup script (e.g., `~/.bashrc` or `~/.zshrc`). This makes your workspace's packages available in new terminal sessions automatically.

## Description

This command is a wrapper around `colcon build`. It ensures that the build process is run in a properly sourced environment and provides helpful feedback. It uses `--symlink-install` by default, which allows you to edit Python files and have the changes take effect without needing to rebuild (unless you add new files or entry points).

### Example: Build the entire workspace

```bash
genesys build
```

### Example: Build specific packages

```bash
genesys build --packages my_package_1 my_package_2
```

### Example: Build and persist the workspace overlay

```bash
genesys build --persist
```
After running this, you can open a new terminal, and your workspace will be automatically sourced.
