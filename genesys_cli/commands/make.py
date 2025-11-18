import os
import click
import sys
import re
import subprocess
from genesys_cli.utils import get_sourcing_command
from genesys_cli.scaffolding import (
    add_python_entry_point,
    add_python_component_entry_point,
    add_install_rule_for_launch_dir,
    add_cpp_executable,
    add_install_rule_for_launch_dir_cpp,
    add_cpp_dependencies_to_package_xml,
    add_launch_file_boilerplate,
    add_node_to_launch,
    add_node_to_mixed_launch,
    add_component_to_mixed_launch,
    add_default_launch_file,
    add_component_to_regular_launch,
)
from .templates import get_python_node_template, get_python_component_template, get_mixed_launch_template, get_cpp_node_template, get_cmakelists_template

def _get_choice_from_numbered_list(prompt_message, choices, default_index=0):
    """
    Presents a numbered list of choices to the user and returns the selected choice.
    """
    click.echo(prompt_message)
    for i, choice in enumerate(choices):
        click.echo(f"  {i+1}. {choice}")
    
    while True:
        try:
            response = click.prompt(
                f"Enter your choice (1-{len(choices)})",
                type=click.IntRange(1, len(choices)),
                default=default_index + 1 # Adjust default to be 1-indexed
            )
            return choices[response - 1] # Convert back to 0-indexed
        except click.exceptions.Abort:
            click.secho("Aborted.", fg="red")
            sys.exit(1)
        except Exception as e:
            click.secho(f"Invalid input: {e}. Please try again.", fg="red")

@click.group("make")
def make():
    """Scaffold ROS 2 components."""
    pass

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

        if lang_choice.lower() == 'c++':
            cmake_path = os.path.join('src', package_name, 'CMakeLists.txt')
            cmakelists_content = get_cmakelists_template(package_name)
            with open(cmake_path, 'w') as f:
                f.write(cmakelists_content)

    except subprocess.CalledProcessError as e:
        click.secho(f"Error creating package '{package_name}':", fg="red")
        click.echo(e.stderr or e.stdout)
        sys.exit(1)

    if with_node:
        ctx.invoke(make_node, node_name=f"{package_name}_node", pkg_name=package_name)

@make.command("node")
@click.argument("node_name")
@click.option('--pkg', 'pkg_name', required=True, help='The name of the package to add the node to.')
@click.option('--component', 'is_component', is_flag=True, help='Create a component instead of a regular node.')
@click.pass_context
def make_node(ctx, node_name, pkg_name, is_component):
    """Creates a new node file and registers it in an existing package."""
    if is_component:
        ctx.invoke(make_component, component_name=node_name, pkg_name=pkg_name)
        return

    node_choices = ['Publisher', 'Subscriber', 'Service', 'ActionServer', 'Lifecycle']
    node_type = _get_choice_from_numbered_list(
        'Select node type:',
        node_choices,
        default_index=node_choices.index('Publisher')
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
        add_node_to_mixed_launch(pkg_name, node_name)
    elif os.path.exists(os.path.join(pkg_path, 'CMakeLists.txt')): # C++ package
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
        add_cpp_dependencies_to_package_xml(pkg_name, ["rclcpp", "std_msgs"])
    else:
        click.secho(f"Error: Could not determine package type for '{pkg_name}'. No setup.py or CMakeLists.txt found.", fg="red")
        sys.exit(1)

    add_launch_file_boilerplate(pkg_name, node_name)
    add_node_to_launch(pkg_name, node_name)
    add_default_launch_file(pkg_name)
    
    # Add install rules *after* launch files are created.
    add_install_rule_for_launch_dir(pkg_name)
    add_install_rule_for_launch_dir_cpp(pkg_name)

    click.echo("\nRun 'genesys build' to make the new node available.")

@make.command("component")
@click.argument("component_name")
@click.option('--pkg', 'pkg_name', required=True, help='The name of the package to add the component to.')
@click.pass_context
def make_component(ctx, component_name, pkg_name):
    """Creates a new component file and registers it in an existing package."""
    component_choices = ['Publisher', 'Subscriber', 'Service', 'ActionServer', 'Lifecycle']
    component_type = _get_choice_from_numbered_list(
        'Select component type:',
        component_choices,
        default_index=component_choices.index('Publisher')
    )
    click.echo(f"Scaffolding a '{component_type}' component named '{component_name}' in package '{pkg_name}'.")

    pkg_path = os.path.join('src', pkg_name)
    if not os.path.isdir(pkg_path):
        click.secho(f"Error: Package '{pkg_name}' not found at {pkg_path}", fg="red")
        sys.exit(1)

    class_name = "".join(word.capitalize() for word in component_name.split('_'))

    # Determine package type and create node
    if os.path.exists(os.path.join(pkg_path, 'setup.py')):
        # Python package
        comp_dir = os.path.join(pkg_path, pkg_name)
        os.makedirs(comp_dir, exist_ok=True)
        comp_file = os.path.join(comp_dir, f"{component_name}.py")
        with open(comp_file, 'w') as f:
            # Pass component_type to the template function
            py_boilerplate = get_python_component_template(component_type.lower(), component_name, class_name)
            f.write(py_boilerplate)
        click.secho(f"✓ Created Python component file: {comp_file}", fg="green")
        add_python_component_entry_point(pkg_name, component_name)
        
        mixed_launch_file_path = os.path.join(pkg_path, 'launch', 'mixed_launch.py')
        regular_launch_file_path = os.path.join(pkg_path, 'launch', f"{pkg_name}_launch.py")

        if os.path.exists(mixed_launch_file_path):
            click.secho(f"Mixed launch file found for '{pkg_name}'. Adding component...", fg="green")
            add_component_to_mixed_launch(pkg_name, component_name)
            add_default_launch_file(pkg_name) # Ensure default launch file includes the mixed launch
        elif os.path.exists(regular_launch_file_path):
            click.secho(f"Regular launch file found for '{pkg_name}'. Adding component to it...", fg="green")
            add_component_to_regular_launch(pkg_name, component_name)
            add_default_launch_file(pkg_name) # Ensure default launch file includes the regular launch
        else:
            click.secho(f"No existing launch file found for '{pkg_name}'. Creating a mixed launch file...", fg="yellow")
            ctx.invoke(make_launch, pkg_name=pkg_name, launch_name='mixed_launch')
            add_component_to_mixed_launch(pkg_name, component_name)
            add_default_launch_file(pkg_name) # Ensure default launch file includes the mixed launch
    else:
        click.secho(f"Error: Could not determine package type for '{pkg_name}'. No setup.py found.", fg="red")
        sys.exit(1)

    click.echo("\nRun 'genesys build' to make the new component available.")

@make.command("launch")
@click.option('--pkg', 'pkg_name', required=True, help='The name of the package to add the launch file to.')
@click.option('--name', 'launch_name', default='mixed_launch', help='The name of the launch file.')
def make_launch(pkg_name, launch_name):
    """Creates a new mixed launch file."""
    click.echo(f"Scaffolding a mixed launch file named '{launch_name}.py' in package '{pkg_name}'.")

    pkg_path = os.path.join('src', pkg_name)
    if not os.path.isdir(pkg_path):
        click.secho(f"Error: Package '{pkg_name}' not found at {pkg_path}", fg="red")
        sys.exit(1)

    launch_dir = os.path.join(pkg_path, 'launch')
    os.makedirs(launch_dir, exist_ok=True)
    launch_file = os.path.join(launch_dir, f"{launch_name}.py")

    if os.path.exists(launch_file):
        click.secho(f"Error: Launch file '{launch_name}.py' already exists in {launch_dir}", fg="red")
        sys.exit(1)

    with open(launch_file, 'w') as f:
        launch_boilerplate = get_mixed_launch_template()
        f.write(launch_boilerplate)
    
    click.secho(f"✓ Created mixed launch file: {launch_file}", fg="green")

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
