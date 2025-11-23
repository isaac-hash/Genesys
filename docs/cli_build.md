# CLI Command: `genesys build`

The `genesys build` command compiles your workspace using `colcon`, the standard build tool for ROS 2. It provides a convenient wrapper with helpful defaults and extra features.

## Usage

```bash
genesys build [options]
```

## Options

- **`--packages <pkg1> <pkg2>...`** or **`-p <pkg1>...`**:
  Build only the specified packages. If omitted, all packages in the workspace are built.

- **`--persist`**:
  After a successful build, this flag will add a command to source your workspace's `install/setup.bash` file to your shell's startup script (e.g., `~/.bashrc` or `~/.zshrc`).

## How It Works

1.  **Workspace Validation**: The command first checks that you are in the root of a Genesys workspace by looking for the `src/` directory.
2.  **Command Execution**: It then constructs and runs the following `colcon` command:
    ```bash
    colcon build --symlink-install --cmake-clean-first
    ```
    - `--symlink-install`: This is a crucial flag for rapid development. It creates symbolic links from the `install` directory to your source files instead of copying them. This means you can edit Python files and the changes will take effect immediately without needing to run `genesys build` again. (Note: You still need to rebuild if you add new files, messages, or change entry points in `setup.py`).
    - `--cmake-clean-first`: This helps avoid issues with cached variables when you make significant changes to your `CMakeLists.txt` files, ensuring a cleaner and more reliable build.
    - If you use the `--packages` option, it appends `--packages-select <pkg1> <pkg2>` to the command.
3.  **Real-time Output**: The output from the `colcon` command is streamed directly to your terminal in real-time, so you can monitor the build progress exactly as it happens.
4.  **Persistent Sourcing**: If you use the `--persist` flag, the tool will automatically perform the following action upon a successful build:
    - It identifies your shell (`bash` or `zsh`).
    - It appends a line like `source /path/to/your/project/install/setup.bash` to your `~/.bashrc` or `~/.zshrc` file.
    - This is idempotent; it will not add the line if it already exists.
    - This action makes your workspace's packages, nodes, and launch files automatically available in any new terminal you open.

## Examples

### Build the entire workspace
This is the most common use case.
```bash
genesys build
```

### Build specific packages
Useful when you are working on a few packages in a large workspace.
```bash
genesys build --packages my_package_1 my_package_2
```

### Build and persist the workspace overlay
A "fire-and-forget" command for a new workspace. After this, you can open a new terminal and your workspace will be ready to use.
```bash
genesys build --persist
```
