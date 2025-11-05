import os
import click
import subprocess
import sys

def get_sourcing_command(exit_on_error=True, clean_env=False):
    """
    Returns the platform-specific command to source the ROS 2 and local workspace environments.

    :param clean_env: If True, unsets common ROS environment variables for a clean build.
    """
    ros_distro = os.environ.get('ROS_DISTRO')
    if not ros_distro:
        if exit_on_error:
            click.secho("Error: ROS_DISTRO environment variable not set.", fg="red")
            click.secho("Cannot find ROS 2 installation to source.", fg="yellow")
            sys.exit(1)
        return None, None

    # Platform-specific setup
    if sys.platform.startswith('linux') or sys.platform == 'darwin':
        shell_exec = '/bin/bash'
        distro_setup_script = f"/opt/ros/{ros_distro}/setup.bash"
        ws_setup_script = "./install/setup.bash" # Relative to workspace root
        
        if not os.path.exists(distro_setup_script):
            if exit_on_error:
                click.secho(f"Error: ROS 2 setup script not found at {distro_setup_script}", fg="red")
                sys.exit(1)
            return None, None
            
        # Chain the sourcing commands
        command_parts = []
        if clean_env:
            # These are the most common variables that cause cross-workspace contamination.
            command_parts.extend(["unset AMENT_PREFIX_PATH", "unset COLCON_PREFIX_PATH"])

        command_parts.append(f"source {distro_setup_script}")
        if os.path.exists(ws_setup_script):
            command_parts.append(f"source {ws_setup_script}")
            
        source_prefix = " && ".join(command_parts) + " && "
        return source_prefix, shell_exec
    
    elif sys.platform == 'win32':
        click.secho("Warning: Auto-sourcing on Windows is not fully implemented. Please run this from a sourced ROS 2 terminal.", fg="yellow", err=True)
        return "", None # No prefix command, use default shell
    
    else:
        click.secho(f"Unsupported platform for auto-sourcing: {sys.platform}", fg="red")
        if exit_on_error:
            sys.exit(1)
        return None, None

def run_sourcing_command(command_str, interactive=True, exit_on_error=True):
    """
    Runs a command string within a sourced environment.

    :param command_str: The command to run (e.g., 'ros2 node list').
    :param interactive: If True, streams output and allows user interruption.
                        If False, captures output and prints at the end.
    :param exit_on_error: If True, exits the CLI on command failure.
    :return: True on success, False on failure.
    """
    source_prefix, shell_exec = get_sourcing_command()
    full_command = source_prefix + command_str

    click.echo(f"Executing: {command_str}")

    try:
        if interactive:
            process = subprocess.Popen(
                full_command,
                shell=True,
                executable=shell_exec,
                stdout=sys.stdout,
                stderr=sys.stderr
            )
            process.wait()
            if process.returncode != 0:
                raise subprocess.CalledProcessError(process.returncode, full_command)
        else:
            result = subprocess.run(
                full_command,
                shell=True,
                executable=shell_exec,
                check=True,
                capture_output=True,
                text=True
            )
            if result.stdout:
                click.echo(result.stdout)
            if result.stderr:
                click.echo(result.stderr, err=True)
        return True
    except KeyboardInterrupt:
        click.echo("\nCommand interrupted by user.")
        return False
    except subprocess.CalledProcessError as e:
        click.secho(f"\nCommand failed with exit code {e.returncode}.", fg="red")
        if not interactive and hasattr(e, 'stderr') and e.stderr:
            click.secho("Error output:", fg="yellow")
            click.echo(e.stderr)
        if exit_on_error:
            sys.exit(1)
        return False
