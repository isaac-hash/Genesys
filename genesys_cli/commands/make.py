import os
import click
import sys
import subprocess
from genesys_cli.utils import get_sourcing_command
from genesys_cli.scaffolding import (
    add_python_entry_point,
    add_install_rule_for_launch_dir,
    add_cpp_executable,
    add_install_rule_for_launch_dir_cpp,
    add_launch_file_boilerplate,
    add_node_to_launch,
    add_install_rule_for_launch_dir_cpp,
)
from .templates import get_python_node_template, get_cpp_node_template

@click.group("make")
def make():
    # Convert node_name (e.g. my_awesome_node) to ClassName (e.g. MyAwesomeNode)
    """Scaffolding commands for creating new ROS 2 entities."""
    pass

@make.command("pkg")
@click.argument("pkg_name")
@click.option("--build-type", default="ament_python", type=click.Choice(['ament_python', 'ament_cmake']), help="The build type for the package (e.g., ament_python, ament_cmake).")
@click.option("--dependencies", "-d", multiple=True, help="A list of dependencies for the package. Can be specified multiple times.")
def pkg(pkg_name, build_type, dependencies):
    """Creates a new ROS 2 package in the 'src' directory."""
    src_path = os.path.join(os.getcwd(), 'src')
    if not os.path.isdir(src_path):
        click.secho(f"Error: 'src' directory not found in {os.getcwd()}.", fg="red")
        click.secho("Please run this command from the root of your ROS 2 workspace.", fg="red")
        sys.exit(1)

    command = ['ros2', 'pkg', 'create', '--build-type', build_type, pkg_name]
    if dependencies:
        command.extend(['--dependencies', *dependencies])

    click.secho(f"Running command: {' '.join(command)}", fg="blue")
    
    # The command should be run inside the 'src' directory
    subprocess.run(command, check=True, cwd=src_path)
    click.secho(f"✓ Successfully created package '{pkg_name}' in '{src_path}'.", fg="green")

@make.command("node")
@click.argument("pkg_name")
@click.argument("node_name")
@click.option("--node-type", default="publisher", type=click.Choice(['publisher', 'subscriber', 'service', 'client']), help="Type of the node to create.")
def node(pkg_name, node_name, node_type):
    """Creates a new ROS 2 node in the specified package."""
    # Find the package path
    try:
        pkg_path = subprocess.check_output(['ros2', 'pkg', 'prefix', pkg_name], text=True).strip()
        pkg_path = pkg_path.replace('/install/', '/src/')
    except subprocess.CalledProcessError:
        click.secho(f"Error: Package '{pkg_name}' not found. Please make sure it's in your workspace.", fg="red")
        sys.exit(1)

    class_name = "".join(word.capitalize() for word in node_name.split('_'))

    # Determine package type and create node
    if os.path.exists(os.path.join(pkg_path, 'setup.py')):
        # Python package
        os.makedirs(node_dir, exist_ok=True)
        node_file = os.path.join(node_dir, f"{node_name}.py")
        with open(node_file, 'w') as f:
            py_boilerplate = get_python_node_template(node_type, node_name, class_name)
            f.write(py_boilerplate)
        click.secho(f"✓ Created Python node file: {node_file}", fg="green")
        add_python_entry_point(pkg_name, node_name)
        add_install_rule_for_launch_dir(pkg_name)
    elif os.path.exists(os.path.join(pkg_path, 'CMakeLists.txt')):
        # C++ package
        os.makedirs(node_dir, exist_ok=True)
        node_file = os.path.join(node_dir, f"{node_name}.cpp")
        if node_type.lower() != 'publisher':
            click.secho(
                "Warning: C++ node scaffolding currently only supports the 'Publisher' type. "
                "A publisher node will be created.",
                fg="yellow"
            )
        
        cpp_boilerplate = get_cpp_node_template(node_name, class_name)
        with open(node_file, 'w') as f:
            f.write(cpp_boilerplate)
        click.secho(f"✓ Created C++ node file: {node_file}", fg="green")
        add_cpp_executable(pkg_name, node_name)
        add_install_rule_for_launch_dir_cpp(pkg_name)
    else:
        click.secho(f"Error: Could not determine package type for '{pkg_name}'. No setup.py or CMakeLists.txt found.", fg="red")
        sys.exit(1)

    # Common scaffolding for both Python and C++
    add_launch_file_boilerplate(pkg_name, node_name)
    add_node_to_launch(pkg_name, node_name)
    add_default_launch_file(pkg_name)

    click.echo("\nRun 'genesys build' to make the new node available.")

@make.command("interface")
def interface():
    """(Not yet implemented) Creates a new ROS 2 interface (msg, srv, action)."""
    click.secho("Command 'make interface' is not yet implemented.", fg="yellow")
    pass
