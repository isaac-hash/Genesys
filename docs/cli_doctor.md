# CLI Command: `genesys doctor`

The `genesys doctor` command is a diagnostic tool that checks your local environment for common configuration problems that could prevent Genesys or ROS 2 from working correctly.

## Usage

```bash
genesys doctor
```

## Description

When you run `genesys doctor`, it performs several checks and provides feedback and solutions for any issues it finds.

### Checks Performed

1.  **`PATH` Configuration**:
    - It verifies that the directory where `pip` installs command-line scripts (like `genesys`) is included in your system's `PATH` environment variable.
    - If it's not, you might get a "command not found" error when trying to run `genesys`. The doctor will provide the `export` command needed to fix this, both temporarily and permanently.

2.  **ROS 2 Environment**:
    - It checks if the `ROS_DISTRO` environment variable is set and if the corresponding ROS 2 `setup.bash` script can be found.
    - If you haven't sourced a ROS 2 distribution, most `ros2` and `genesys` commands will fail.

3.  **Missing Dependencies (`rosdep`)**:
    - If run from within a workspace (i.e., a `src` directory is present), it will run `rosdep install` to check for any missing system dependencies required by the packages in your workspace.
    - It will prompt to install them if any are found to be missing.

## When to Use It

You should run `genesys doctor` if you encounter any strange behavior, "command not found" errors, or build failures. It's a good first step for troubleshooting your development environment.
