import os
import click
import subprocess
import sys
import re
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
@click.option('--packages', '-p', multiple=True, help='Specific packages to build. Builds all if not specified.')
def build(packages):
    """Builds the entire workspace or specific packages."""
    # 1. Verify we are in a Genesys workspace root.
    if not os.path.isdir('src'):
        click.secho("Error: This command must be run from the root of a Genesys workspace.", fg="red")
        click.secho("(A 'src' directory was not found.)", fg="yellow")
        sys.exit(1)

    click.echo("Building the workspace...")

    source_prefix, shell_exec = _get_sourcing_command(clean_env=True)
    colcon_command = ['colcon', 'build', '--symlink-install']
    if packages:
        colcon_command.extend(['--packages-select'] + list(packages))

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

def _add_python_entry_point(pkg_name, node_name):
    """Adds a new console_script entry to a package's setup.py file."""
    setup_file = os.path.join('src', pkg_name, 'setup.py')
    node_module_name = node_name.replace('.py', '')

    with open(setup_file, 'r') as f:
        content = f.read()

    # Use re.DOTALL to match newlines. Use named groups for clarity.
    match = re.search(
        r"(?P<pre>('|\")console_scripts('|\")\s*:\s*\[)(?P<scripts>[^\]]*)(?P<post>\])",
        content,
        re.DOTALL
    )

    if not match:
        click.secho(f"Error: Could not find 'console_scripts' in {setup_file}.", fg="red")
        return

    scripts_content = match.group('scripts')

    # Check if node is already registered
    if f"'{node_name} =" in scripts_content or f'"{node_name} =' in scripts_content:
        click.secho(f"Node '{node_name}' already exists in {setup_file}.", fg="yellow")
        return

    new_entry = f"'{node_name} = {pkg_name}.{node_module_name}:main'"

    # Find the last non-empty line in the scripts block
    lines = [line for line in scripts_content.split('\n') if line.strip()]

    if lines:
        # The list has existing entries.
        last_line = lines[-1]
        indentation = " " * (len(last_line) - len(last_line.lstrip()))
        text_to_insert = ""
        if not last_line.strip().endswith(','):
            text_to_insert += ","
        text_to_insert += f"\n{indentation}{new_entry}"
        updated_content = content.replace(last_line, last_line + text_to_insert)
    else:
        # The list is empty.
        pre_match_line_start = content.rfind('\n', 0, match.start('scripts')) + 1
        indentation = " " * (match.start('scripts') - pre_match_line_start) + "    "
        insertion = f"\n{indentation}{new_entry}\n"
        insertion_point = match.end('scripts')
        updated_content = content[:insertion_point] + insertion + content[insertion_point:]

    with open(setup_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Registered '{node_name}' in {setup_file}", fg="green")

def _add_cpp_executable(pkg_name, node_name):
    """Adds a new executable and install rule to a package's CMakeLists.txt."""
    cmake_file = os.path.join('src', pkg_name, 'CMakeLists.txt')
    node_src_file = f"src/{node_name}.cpp"

    with open(cmake_file, 'r') as f:
        content = f.read()

    if f'add_executable({node_name}' in content:
        click.secho(f"Node '{node_name}' already appears to be registered in {cmake_file}.", fg="yellow")
        return

    # Find the ament_package() call to insert before it
    ament_package_call = re.search(r"ament_package\(\)", content)
    if not ament_package_call:
        click.secho(f"Error: Could not find ament_package() call in {cmake_file}.", fg="red")
        return

    insert_pos = ament_package_call.start()
    new_cmake_commands = f"""
add_executable({node_name} {node_src_file})
ament_target_dependencies({node_name} rclcpp)

install(TARGETS
  {node_name}
  DESTINATION lib/${{PROJECT_NAME}}
)

"""
    updated_content = content[:insert_pos] + new_cmake_commands + content[insert_pos:]

    with open(cmake_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Registered '{node_name}' in {cmake_file}", fg="green")

def _add_launch_file_boilerplate(pkg_name, node_name):
    """Auto-generates a boilerplate launch file for a new node."""
    launch_dir = os.path.join('src', pkg_name, 'launch')
    os.makedirs(launch_dir, exist_ok=True)
    launch_file = os.path.join(launch_dir, f"{pkg_name}_launch.py")
    
    # Only create a launch file if it doesn't already exist to avoid overwriting a custom one
    if not os.path.exists(launch_file):
        boilerplate = f"""from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='{pkg_name}',
            executable='{node_name}',
            name='{node_name}',
            output='screen',
            emulate_tty=True
        )
    ])
"""
        with open(launch_file, 'w') as f:
            f.write(boilerplate)
        click.secho(f"✓ Auto-generated launch file: {launch_file}", fg="green")

@cli.command(name='make:node')
@click.argument('node_name')
@click.option('--pkg', 'pkg_name', required=True, help='The name of the package to add the node to.')
def make_node(node_name, pkg_name):
    """Creates a new node file and registers it in an existing package."""
    pkg_path = os.path.join('src', pkg_name)
    if not os.path.isdir(pkg_path):
        click.secho(f"Error: Package '{pkg_name}' not found at {pkg_path}", fg="red")
        sys.exit(1)

    # Determine package type and create node
    if os.path.exists(os.path.join(pkg_path, 'setup.py')):
        # Python package
        node_dir = os.path.join(pkg_path, pkg_name)
        os.makedirs(node_dir, exist_ok=True)
        node_file = os.path.join(node_dir, f"{node_name}.py")
        boilerplate = f"""import rclpy
from rclpy.node import Node

class {node_name.capitalize()}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.get_logger().info('Node {node_name} has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = {node_name.capitalize()}()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        with open(node_file, 'w') as f:
            f.write(boilerplate)
        click.secho(f"✓ Created Python node file: {node_file}", fg="green")
        _add_python_entry_point(pkg_name, node_name)
        _add_launch_file_boilerplate(pkg_name, node_name) # Call the new function
    elif os.path.exists(os.path.join(pkg_path, 'CMakeLists.txt')):
        # C++ package
        node_dir = os.path.join(pkg_path, 'src')
        os.makedirs(node_dir, exist_ok=True)
        node_file = os.path.join(node_dir, f"{node_name}.cpp")
        boilerplate = f"""#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("{node_name}");
    RCLCPP_INFO(node->get_logger(), "Node {node_name} has been started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}}
"""
        with open(node_file, 'w') as f:
            f.write(boilerplate)
        click.secho(f"✓ Created C++ node file: {node_file}", fg="green")
        _add_cpp_executable(pkg_name, node_name)
    else:
        click.secho(f"Error: Could not determine package type for '{pkg_name}'. No setup.py or CMakeLists.txt found.", fg="red")
        sys.exit(1)

    click.echo("\nRun 'framework build' to make the new node available.")

@cli.command(name='make:pkg')
@click.argument('package_name')
@click.option('--with-node', is_flag=True, help='Create an initial node for the package.')
@click.option('--build-type', '-t', default='ament_python', type=click.Choice(['ament_python', 'ament_cmake']), help='The build type for the package.')
@click.option('--dependencies', '-d', multiple=True, help='ROS 2 package dependencies.')
@click.pass_context
def make_pkg(ctx, package_name, with_node, build_type, dependencies):
    """Creates a new ROS 2 package inside the src/ directory."""

    # Verify workspace root
    if not os.path.isdir('src'):
        click.secho("Error: This command must be run from the root of a Genesys workspace.", fg="red")
        click.secho("(A 'src' directory was not found.)", fg="yellow")
        sys.exit(1)

    click.echo(f"Creating new ROS 2 package: {package_name}")

    command = [
        'ros2', 'pkg', 'create',
        '--build-type', build_type,
        '--destination-directory', 'src',
        package_name
    ]
    
    if dependencies:
        command.extend(['--dependencies'] + list(dependencies))

    source_prefix, shell_exec = _get_sourcing_command(clean_env=True)
    command_to_run = source_prefix + ' '.join(command)

    try:
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

    if with_node:
        ctx.invoke(make_node, node_name=f"{package_name}_node", pkg_name=package_name)

@cli.command()
@click.argument('launch_target')
def launch(launch_target):
    """
    Launches a ROS 2 launch file.

    Can be used in two ways:\n
    - framework launch <pkg_name>:<launch_file.py>\n
    - framework launch <pkg_name> (launches <pkg_name>_launch.py by default)
    """
    # 1. Verify we are in a workspace that has been built.
    if not os.path.isdir('install'):
        click.secho("Error: 'install' directory not found. Have you built the workspace yet?", fg="red")
        click.secho("Try running 'framework build' first.", fg="yellow")
        sys.exit(1)

    # 2. Parse the launch target
    if ':' in launch_target:
        pkg_name, launch_file = launch_target.split(':', 1)
    else:
        pkg_name = launch_target
        launch_file = f"{pkg_name}_launch.py"
        click.echo(f"No launch file specified, defaulting to '{launch_file}'")

    # 3. Get the sourcing command
    source_prefix, shell_exec = _get_sourcing_command(clean_env=True)

    # 4. Construct and run the final command
    launch_command = f"ros2 launch {pkg_name} {launch_file}"
    command_to_run = source_prefix + launch_command

    click.echo(f"Executing: {launch_command}")

    try:
        # Use Popen to stream output and allow user to Ctrl+C
        process = subprocess.Popen(command_to_run, shell=True, executable=shell_exec)
        process.wait()
    except KeyboardInterrupt:
        click.echo("\nLaunch interrupted by user.")
        process.terminate()
    except Exception as e:
        click.secho(f"An error occurred during launch: {e}", fg="red")

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
        parts = line.split()
        if len(parts) < 2:
            continue
        pkg = parts[0]
        nodes = parts[1:]
        available_nodes.extend(nodes)
        if node_name in nodes:
            package_name = pkg
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
