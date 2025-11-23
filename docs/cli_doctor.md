# CLI Command: `genesys doctor`

The `genesys doctor` command is a diagnostic tool that checks your local environment for common configuration problems that could prevent Genesys or ROS 2 from working correctly.

It's a good first step for troubleshooting your development environment.

## Usage

```bash
genesys doctor
```

## Checks Performed

The doctor runs the following checks in order.

---

### 1. `PATH` Configuration

This check ensures that you can run `genesys` and other Python-installed tools from any directory.

- **What it does**: Verifies that the directory where `pip` installs command-line scripts is included in your system's `PATH` environment variable.
- **How it works**: It uses Python's `sysconfig.get_path('scripts')` to find the correct local scripts directory for your environment and then checks if that directory is present in the `os.environ['PATH']` variable.
- **Why it matters**: If this path is missing, you might get a "command not found" error when trying to run `genesys` after installing it. The doctor will provide the exact `export` command needed to fix this, both for the current session and permanently by adding it to your `.bashrc` or `.zshrc` file.

---

### 2. ROS 2 Environment

This check verifies that a ROS 2 distribution has been sourced.

- **What it does**: Checks if the `ROS_DISTRO` environment variable is set and if the corresponding ROS 2 `setup.bash` script can be found.
- **Why it matters**: If you haven't sourced a ROS 2 distribution, most `ros2` and `genesys` commands will fail because they can't find the core ROS 2 packages.

---

### 3. Missing System Dependencies

This check finds and installs missing system dependencies for the packages in your workspace.

- **What it does**: If run from within a workspace (i.e., a `src` directory is present), it will run `rosdep` to check for any missing system dependencies (e.g., `libopencv-dev`) required by the packages in `src/`.
- **How it works**: It executes the following command:
  ```bash
  rosdep install --from-paths src -y --ignore-src
  ```
  - `--from-paths src`: Tells `rosdep` to look at the `package.xml` files of all packages inside the `src` directory.
  - `-y`: Automatically answers "yes" to all prompts from the system's package manager (e.g., `apt-get`), installing dependencies non-interactively.
  - `--ignore-src`: Prevents `rosdep` from trying to resolve the packages in your `src` directory as system packages, which is the correct behavior.
- **Why it matters**: This ensures that when you run `genesys build`, the compilation won't fail due to missing system libraries that your ROS 2 packages depend on.
