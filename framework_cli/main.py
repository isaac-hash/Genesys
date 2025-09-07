import os
import click
import subprocess
import sys
import shutil

@click.group()
def cli():
    """Genesys CLI for ROS 2 workspace management."""
    pass

def _get_sourcing_command(exit_on_error=True, clean_env=False):
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

@cli.command()
def doctor():
    """Checks the environment for potential issues and provides solutions."""
    click.secho("Running Genesys environment doctor...", fg="cyan", bold=True)
    all_ok = True

    # 1. Check if 'framework' command is on the PATH
    click.echo("\nChecking PATH configuration...")
    script_path = shutil.which('framework')
    if script_path and os.path.dirname(script_path) not in os.environ.get('PATH', '').split(os.pathsep):
        all_ok = False
        script_dir = os.path.dirname(script_path)
        click.secho("[X] PATH Issue Detected", fg="red")
        click.echo(f"  The 'framework' command is in a directory not on your system's PATH: {script_dir}")
        click.echo("\n  To fix this for your current session, run:")
        click.secho(f'  export PATH="{script_dir}:$PATH"', fg="yellow")
        click.echo("\n  To fix this permanently, copy and paste the following command:")
        click.secho(f"  echo 'export PATH=\"{script_dir}:$PATH\"' >> ~/.bashrc && source ~/.bashrc", fg="green")

    else:
        click.secho("[✓] PATH configuration is correct.", fg="green")

    # 2. Check for ROS_DISTRO and sourcing ability
    click.echo("\nChecking ROS 2 environment...")
    source_prefix, _ = _get_sourcing_command(exit_on_error=False)
    if source_prefix is None:
        all_ok = False
        click.secho("[X] ROS 2 Environment Issue Detected", fg="red")
        click.echo("  The ROS_DISTRO environment variable is not set or the setup script is missing.")
        click.echo("  Please ensure a ROS 2 distribution is installed and the ROS_DISTRO variable is set.")
    else:
        click.secho("[✓] ROS 2 environment sourcing is configured.", fg="green")

    click.echo("-" * 40)
    if all_ok:
        click.secho("✨ Your Genesys environment is ready to go!", fg="cyan", bold=True)
    else:
        click.secho("Please address the issues above to ensure Genesys works correctly.", fg="yellow")

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

    source_prefix, shell_exec = _get_sourcing_command(clean_env=True)
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
        '--destination-directory', 'src',
        package_name
    ]

    if with_node:
        # Per the guard rails, create a default node. A good convention is <pkg_name>_node.
        node_name = f"{package_name}_node"
        command.extend(['--node-name', node_name])
        click.echo(f"Adding initial node: {node_name}")

    source_prefix, shell_exec = _get_sourcing_command(clean_env=True)
    command_to_run = source_prefix + ' '.join(command)

    try:
        # Run the command from the workspace root.
        # Use shell=True to allow the 'source' command to work.
        subprocess.run(
            command_to_run,
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

@cli.command()
@click.argument('node_name')
def run(node_name):
    """Runs a ROS 2 node by its executable name, automatically finding the package."""
    # 1. Verify we are in a workspace that has been built.
    if not os.path.isdir('install'):
        click.secho("Error: 'install' directory not found. Have you built the workspace yet?", fg="red")
        click.secho("Try running 'framework build' first.", fg="yellow")
        sys.exit(1)

    click.echo(f"Attempting to run node: {node_name}")

    # 2. Get the sourcing command, which now includes the local install space.
    source_prefix, shell_exec = _get_sourcing_command(clean_env=True)
    
    # 3. Find the package for the given node by listing all executables.
    list_exec_command = source_prefix + "ros2 pkg executables"
    try:
        result = subprocess.run(
            list_exec_command,
            check=True, capture_output=True, text=True, shell=True, executable=shell_exec
        )
    except subprocess.CalledProcessError as e:
        click.secho("Error: Failed to list ROS 2 executables.", fg="red")
        click.echo(e.stderr or e.stdout)
        sys.exit(1)

    # 4. Parse the output to find the package name.
    package_name = None
    available_nodes = []
    for line in result.stdout.strip().split('\n'):
        if ':' not in line:
            continue
        pkg, nodes_str = line.split(':', 1)
        nodes = nodes_str.strip().split()
        available_nodes.extend(nodes)
        if node_name in nodes:
            package_name = pkg.strip()
            break
    
    if not package_name:
        click.secho(f"Error: Node '{node_name}' not found in any package.", fg="red")
        click.echo("Please ensure you have built your workspace and the node name is correct.")
        if available_nodes:
            click.echo("\nAvailable nodes are:")
            for node in sorted(available_nodes):
                click.echo(f"  - {node}")
        sys.exit(1)

    click.echo(f"Found node '{node_name}' in package '{package_name}'. Starting node...")

    # 5. Construct and run the final command.
    run_command = f"ros2 run {package_name} {node_name}"
    command_to_run = source_prefix + run_command

    try:
        # Use Popen to stream output and allow user to Ctrl+C the node.
        process = subprocess.Popen(command_to_run, shell=True, executable=shell_exec)
        process.wait()
    except KeyboardInterrupt:
        click.echo("\nNode execution interrupted by user.")
        process.terminate()

if __name__ == '__main__':
    cli()
