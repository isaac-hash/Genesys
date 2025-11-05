import os
import click
import sys
import re
import subprocess
from genesys_cli.utils import get_sourcing_command
from genesys_cli.scaffolding import (
    add_python_entry_point,
    add_install_rule_for_launch_dir, # This is duplicated, but I'll leave it for now
    add_cpp_executable,
    add_install_rule_for_launch_dir_cpp,
    add_launch_file_boilerplate,
    add_node_to_launch,
    add_default_launch_file,
)
from .templates import get_python_node_template, get_cpp_node_template

@click.group("make")
def make():
    """Scaffold ROS 2 components."""
    pass

@make.command("node")
@click.argument("node_name")
@click.option('--pkg', 'pkg_name', required=True, help='The name of the package to add the node to.')
@click.pass_context
def make_node(ctx, node_name, pkg_name):
    """Creates a new node file and registers it in an existing package."""
    node_type = click.prompt(
        'Select node type',
        type=click.Choice(['Publisher', 'Subscriber', 'Service', 'ActionServer', 'Lifecycle'], case_sensitive=False),
        default='Publisher'
    )
    click.echo(f"Scaffolding a '{node_type}' node named '{node_name}' in package '{pkg_name}'.")

    pkg_path = os.path.join('src', pkg_name)
    if not os.path.isdir(pkg_path):
        click.secho(f"Error: Package '{pkg_name}' not found at {pkg_path}", fg="red")
        sys.exit(1)

    class_name = "".join(word.capitalize() for word in node_name.split('_'))

    # Determine package type and create node
    if os.path.exists(os.path.join(pkg_path, 'setup.py')):
        # Python package
        node_dir = os.path.join(pkg_path, pkg_name)
        os.makedirs(node_dir, exist_ok=True)
        node_file = os.path.join(node_dir, f"{node_name}.py")
        with open(node_file, 'w') as f:
            py_boilerplate = get_python_node_template(node_type.lower(), node_name, class_name)
            f.write(py_boilerplate)
        click.secho(f"✓ Created Python node file: {node_file}", fg="green")
        add_python_entry_point(pkg_name, node_name)
        add_install_rule_for_launch_dir(pkg_name)
    elif os.path.exists(os.path.join(pkg_path, 'CMakeLists.txt')):
        # C++ package
        node_dir = os.path.join(pkg_path, 'src')
        os.makedirs(node_dir, exist_ok=True)
        node_file = os.path.join(node_dir, f"{node_name}.cpp")
        if node_type.lower() != 'publisher': # Based on original logic
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

    add_launch_file_boilerplate(pkg_name, node_name)
    add_node_to_launch(pkg_name, node_name)
    add_default_launch_file(pkg_name)

    click.echo("\nRun 'genesys build' to make the new node available.")

@make.command("interface")
@click.argument('interface_name')
@click.option('--pkg', 'pkg_name', required=True, help='The name of the package to add the interface to.')
def make_interface(interface_name, pkg_name):
    """Scaffold custom msg/srv/action files."""
    click.secho(f"Scaffolding for interface '{interface_name}' in package '{pkg_name}' is not yet implemented.", fg="yellow")
    click.echo("You will need to manually:")
    click.echo("1. Place .msg/.srv/.action files under src/<pkg>/msg|srv|action/")
    click.echo("2. Update package.xml with <build_depend>rosidl_default_generators</build_depend> and <exec_depend>rosidl_default_runtime</exec_depend>")
    click.echo("3. Update CMakeLists.txt with find_package(rosidl_default_generators REQUIRED) and rosidl_generate_interfaces()")


@make.command('pkg')
@click.argument('package_name')
@click.option('--with-node', is_flag=True, help='Create an initial node for the package.')
@click.option('--dependencies', '-d', multiple=True, help='ROS 2 package dependencies.')
@click.pass_context
def make_pkg(ctx, package_name, with_node, dependencies):
    """Creates a new ROS 2 package inside the src/ directory."""

    # Verify workspace root
    if not os.path.isdir('src'):
        click.secho("Error: This command must be run from the root of a Genesys workspace.", fg="red")
        click.secho("(A 'src' directory was not found.)", fg="yellow")
        sys.exit(1)

    click.echo(f"Creating new ROS 2 package: {package_name}")

    # Interactive prompt for language choice
    lang_choice = click.prompt(
        'Choose a language for the package',
        type=click.Choice(['Python', 'C++'], case_sensitive=False),
        default='Python',
        show_default=True
    )
    build_type = 'ament_python' if lang_choice.lower() == 'python' else 'ament_cmake'

    command = [
        'ros2', 'pkg', 'create',
        '--build-type', build_type,
        '--destination-directory', 'src',
        package_name
    ]
    
    if dependencies:
        command.extend(['--dependencies'] + list(dependencies))

    source_prefix, shell_exec = get_sourcing_command(clean_env=True)
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
