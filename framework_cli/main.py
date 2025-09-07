import os
import click
import subprocess
import sys

@click.group()
def cli():
    """Genesys CLI for ROS 2 workspace management."""
    pass

def _get_sourcing_command():
    """Returns the platform-specific command to source the ROS 2 environment."""
    ros_distro = os.environ.get('ROS_DISTRO')
    if not ros_distro:
        click.secho("Error: ROS_DISTRO environment variable not set.", fg="red")
        click.secho("Cannot find ROS 2 installation to source.", fg="yellow")
        sys.exit(1)

    if sys.platform.startswith('linux') or sys.platform == 'darwin':
        setup_script = f"/opt/ros/{ros_distro}/setup.bash"
        if not os.path.exists(setup_script):
            click.secho(f"Error: ROS 2 setup script not found at {setup_script}", fg="red")
            sys.exit(1)
        return f"source {setup_script} && ", '/bin/bash'
    
    elif sys.platform == 'win32':
        click.secho("Warning: Auto-sourcing on Windows is not fully implemented. Please run this from a sourced ROS 2 terminal.", fg="yellow", err=True)
        return "", None # No prefix command, use default shell
    
    else:
        click.secho(f"Unsupported platform for auto-sourcing: {sys.platform}", fg="red")
        sys.exit(1)

@cli.command()
@click.argument('project_name')
def new(project_name):
    """Creates a new ROS 2 workspace with the Genesys framework structure."""
    
    workspace_root = project_name
    click.echo(f"Creating new Genesys project at: ./{workspace_root}")

    if os.path.exists(workspace_root):
        click.secho(f"Error: Directory '{workspace_root}' already exists.", fg="red")
        return

    # Define the structure from the "Workspace Structure" reference
    subdirs = [
        "src",
        "launch",
        "config",
        "sim/worlds",
        "sim/models",
        "tests",
        "scripts",
        "tools"
    ]

    try:
        os.makedirs(workspace_root)
        for subdir in subdirs:
            os.makedirs(os.path.join(workspace_root, subdir))
        click.secho(f"✓ Project '{project_name}' created successfully.", fg="green")
        click.echo("Next steps: 'cd {}' and start creating packages!".format(project_name))
    except Exception as e:
        click.secho(f"Failed to create project: {e}", fg="red")

@cli.command()
def build():
    """Builds the workspace with colcon, applying symlink-install and auto-sourcing."""
    # 1. Verify we are in a Genesys workspace root.
    if not os.path.isdir('src'):
        click.secho("Error: This command must be run from the root of a Genesys workspace.", fg="red")
        click.secho("(A 'src' directory was not found.)", fg="yellow")
        sys.exit(1)

    click.echo("Building the workspace...")

    source_prefix, shell_exec = _get_sourcing_command()
    colcon_command = ['colcon', 'build', '--symlink-install']
    command_to_run = source_prefix + ' '.join(colcon_command)

    click.echo(f"Running build command...")

    try:
        # Use Popen to stream output in real-time, which is better for build commands.
        process = subprocess.Popen(
            command_to_run,
            shell=True,
            executable=shell_exec,
            stdout=sys.stdout,
            stderr=sys.stderr
        )
        process.wait() # Wait for the build to finish
        
        if process.returncode == 0:
            click.secho("\n✓ Build completed successfully.", fg="green")
            click.echo("To use the new executables, you may need to source the workspace or start a new terminal.")
        else:
            raise subprocess.CalledProcessError(process.returncode, command_to_run)

    except subprocess.CalledProcessError as e:
        click.secho(f"\nBuild failed with exit code {e.returncode}.", fg="red")
        sys.exit(1)

@cli.command(name='make:pkg')
@click.argument('package_name')
@click.option('--with-node', is_flag=True, help='Create an initial node for the package.')
def make_pkg(package_name, with_node):
    """Creates a new ROS 2 package inside the src/ directory."""
    
    # First, verify we are in a Genesys workspace root.
    if not os.path.isdir('src'):
        click.secho("Error: This command must be run from the root of a Genesys workspace.", fg="red")
        click.secho("(A 'src' directory was not found.)", fg="yellow")
        sys.exit(1)

    click.echo(f"Creating new ROS 2 package: {package_name}")

    # Default to ament_python. This can be overridden by the interactive prompt.
    build_type = 'ament_python'

    if with_node:
        # Per guard rails, prompt for language when creating a node.
        language = click.prompt(
            'Choose a language for the new node',
            type=click.Choice(['python', 'cpp'], case_sensitive=False),
            default='python',
            show_choices=True
        )
        if language == 'cpp':
            build_type = 'ament_cmake'
        click.echo(f"Using build type: {build_type}")

    command = [
        'ros2', 'pkg', 'create',
        '--build-type', build_type,
        package_name
    ]

    if with_node:
        # Per the guard rails, create a default node. A good convention is <pkg_name>_node.
        node_name = f"{package_name}_node"
        command.extend(['--node-name', node_name])
        click.echo(f"Adding initial node: {node_name}")

    source_prefix, shell_exec = _get_sourcing_command()
    command_to_run = source_prefix + ' '.join(command)

    try:
        # We must run the command inside the 'src' directory.
        # Use shell=True to allow the 'source' command to work.
        subprocess.run(
            command_to_run,
            cwd='src',
            check=True,
            capture_output=True,
            text=True,
            shell=True,
            executable=shell_exec
        )
        click.secho(f"✓ Package '{package_name}' created successfully in 'src/'.", fg="green")

    except subprocess.CalledProcessError as e:
        click.secho(f"Error creating package '{package_name}':", fg="red")
        click.echo(e.stderr or e.stdout)
        sys.exit(1)

if __name__ == '__main__':
    cli()
