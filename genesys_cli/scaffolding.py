import os
import re
import click
import sys

def persist_workspace_sourcing():
    """Appends the workspace sourcing command to the user's shell startup file."""
    if not (sys.platform.startswith('linux') or sys.platform == 'darwin'):
        click.secho("Warning: Persistent sourcing is only supported on Linux and macOS.", fg="yellow")
        return

    shell_path = os.environ.get("SHELL", "")
    rc_file = None
    if "zsh" in shell_path:
        rc_file = os.path.expanduser("~/.zshrc")
    elif "bash" in shell_path:
        rc_file = os.path.expanduser("~/.bashrc")
    else:
        click.secho(f"Warning: Unsupported shell '{shell_path}' for persistent sourcing. Please add sourcing manually.", fg="yellow")
        return

    workspace_path = os.getcwd()
    setup_script_path = os.path.join(workspace_path, 'install', 'setup.bash')
    
    if not os.path.exists(setup_script_path):
        click.secho(f"Error: Build seems to have finished, but '{setup_script_path}' not found. Cannot persist sourcing.", fg="red")
        return

    source_line = f"source {setup_script_path}"
    comment_line = f"# Sourced by Genesys CLI for workspace: {workspace_path}"
    
    try:
        # Idempotency check: only add if the line is not already present.
        if os.path.exists(rc_file):
            with open(rc_file, 'r') as f:
                if source_line in f.read():
                    click.secho(f"✓ Sourcing for this workspace already exists in {os.path.basename(rc_file)}.", fg="green")
                    return
        
        # Safety: Append to the file, never overwrite.
        with open(rc_file, 'a') as f:
            f.write(f"\n{comment_line}\n{source_line}\n")
        
        click.secho(f"✓ Workspace sourcing added to {os.path.basename(rc_file)}.", fg="green")
        click.echo("  Please open a new terminal session for the changes to take effect.")
    except Exception as e:
        click.secho(f"Error: Failed to write to {rc_file}: {e}", fg="red")

def add_python_entry_point(pkg_name, node_name):
    """Adds a new console_script entry to a package's setup.py file."""
    setup_file = os.path.join('src', pkg_name, 'setup.py')
    node_module_name = node_name.replace('.py', '')

    with open(setup_file, 'r') as f:
        content = f.read()

    # Use re.DOTALL to match newlines. Use named groups for clarity.
    match = re.search(
        r'(?P<pre>(["\'])console_scripts\2\s*:\s*\[)(?P<scripts>.*?)(?P<post>\])',
        content,
        re.DOTALL
    )


    if not match:
        click.secho(f"Error: Could not find 'console_scripts' in {setup_file}.", fg="red")
        return

    scripts_content = match.group('scripts')

    # Check if node is already registered
    if f"'{node_name} ='" in scripts_content or f'"{node_name} ="' in scripts_content:
        click.secho(f"Node '{node_name}' already exists in {setup_file}.", fg="yellow")
        return


    # Always add a comma at the end for consistency, making it easier to append new entries.
    new_entry = f"'{node_name} = {pkg_name}.{node_module_name}:main',"

    # Determine indentation from the line before the closing bracket
    pre_match_line_start = content.rfind('\n', 0, match.end('scripts')) + 1
    indentation = " " * (match.start('scripts') - pre_match_line_start)

    # If the list is not empty, add a newline before the new entry.
    if scripts_content.strip():
        insertion = f"\n{indentation}{new_entry}"
    else: # The list is empty, add indentation and newlines around it.
        indentation += "    " # Add extra indent for the first item
        insertion = f"\n{indentation}{new_entry}\n{indentation[:-4]}"

    # Insert the new entry right before the closing bracket of the list.
    insertion_point = match.end('scripts')
    updated_content = content[:insertion_point] + insertion + content[insertion_point:]

    with open(setup_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Registered '{node_name}' in {setup_file}", fg="green")

def add_install_rule_for_launch_dir(pkg_name):
    """Adds the install rule for the launch directory to setup.py."""
    setup_file = os.path.join('src', pkg_name, 'setup.py')
    if not os.path.exists(setup_file):
        return  # Not a python package

    with open(setup_file, 'r') as f:
        content = f.read()

    # Check if the rule already exists to avoid duplicates
    if "glob(os.path.join('launch'" in content:
        return

    # Add necessary imports if they are missing
    imports_to_add = []
    if 'import os' not in content:
        imports_to_add.append('import os')
    if 'from glob import glob' not in content:
        imports_to_add.append('from glob import glob')
    
    if imports_to_add:
        content = "\n".join(imports_to_add) + "\n" + content

    # Find the line installing package.xml to insert our rule after it
    package_xml_line = "('share/' + package_name, ['package.xml'])"
    match = re.search(re.escape(package_xml_line), content)
    if not match:
        click.secho(f"Warning: Could not find package.xml install rule in {setup_file}. Cannot add launch install rule.", fg="yellow")
        return
    
    # Determine the indentation from the found line
    line_start = content.rfind('\n', 0, match.start()) + 1
    indentation = " " * (match.start() - line_start)

    # Note the comma at the beginning to correctly extend the list
    new_rule = f",\n{indentation}(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))"
    
    # Insert the new rule right after the package.xml line
    insertion_point = match.end()
    updated_content = content[:insertion_point] + new_rule + content[insertion_point:]

    with open(setup_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Added launch directory install rule to {setup_file}", fg="green")

def add_cpp_executable(pkg_name, node_name):
    """Adds a new executable and install rule to a package's CMakeLists.txt."""
    cmake_file = os.path.join('src', pkg_name, 'CMakeLists.txt')
    node_src_file = f"src/{node_name}.cpp"

    with open(cmake_file, 'r') as f:
        content = f.read()

    if f'add_executable({node_name}' in content:
        click.secho(f"Node '{node_name}' already appears to be registered in {cmake_file}.", fg="yellow")
        return

    # Find the ament_package() call to insert before it
    ament_package_call = re.search(r"ament_package\(\"", content)
    if not ament_package_call:
        click.secho(f"Error: Could not find ament_package() call in {cmake_file}.", fg="red")
        return

    insert_pos = ament_package_call.start()
    new_cmake_commands = f"""add_executable({node_name} {node_src_file})
ament_target_dependencies({node_name} rclcpp)

install(TARGETS
  {node_name}
  DESTINATION lib/${{PROJECT_NAME}})

"""
    updated_content = content[:insert_pos] + new_cmake_commands + content[insert_pos:]

    with open(cmake_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Registered '{node_name}' in {cmake_file}", fg="green")

def add_install_rule_for_launch_dir_cpp(pkg_name):
    """Adds the install rule for the launch directory to CMakeLists.txt."""
    cmake_file = os.path.join('src', pkg_name, 'CMakeLists.txt')
    if not os.path.exists(cmake_file):
        return # Not a C++ package

    with open(cmake_file, 'r') as f:
        content = f.read()

    # Check if the rule already exists
    if 'install(DIRECTORY launch' in content:
        return

    # Find the ament_package() call to insert before it
    ament_package_call = re.search(r"ament_package\(\"", content)
    if not ament_package_call:
        click.secho(f"Warning: Could not find ament_package() call in {cmake_file}. Cannot add launch install rule.", fg="yellow")
        return

    insert_pos = ament_package_call.start()
    new_cmake_commands = f"""install(
  DIRECTORY launch
  DESTINATION share/${{PROJECT_NAME}})

"""
    updated_content = content[:insert_pos] + new_cmake_commands + content[insert_pos:]

    with open(cmake_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Added launch directory install rule to {cmake_file}", fg="green")

def add_launch_file_boilerplate(pkg_name, node_name):
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
        ),
    ])
"""
        with open(launch_file, 'w') as f:
            f.write(boilerplate)
        click.secho(f"✓ Auto-generated launch file: {launch_file}", fg="green")

def add_node_to_launch(pkg_name, node_name):
    """Adds a new Node entry into the package's launch file if it exists."""
    launch_file = os.path.join('src', pkg_name, 'launch', f"{pkg_name}_launch.py")
    if not os.path.exists(launch_file):
        return  # no launch file yet (handled in add_launch_file_boilerplate)

    with open(launch_file, 'r') as f:
        content = f.read()

    # Build the new Node block (with a trailing comma, as per the original design)
    new_node_block = f"""Node(
            package='{pkg_name}',
            executable='{node_name}',
            name='{node_name}',
            output='screen',
            emulate_tty=True
        ),"""

    if f"executable='{node_name}'" in content:
        click.secho(f"Launch file already contains '{node_name}'.", fg="yellow")
        return

    # Insert the new node block before the closing bracket of the LaunchDescription list.
    updated_content = re.sub(r"(\s*)(\]\s*\))",  # Find the closing bracket and parenthesis
        rf"\g<1>    {new_node_block}\n\g<1>\g<2>", # Indent and insert the new block
        content,
        flags=re.MULTILINE
    )

    with open(launch_file, 'w') as f:
        f.write(updated_content)

    click.secho(f"✓ Added '{node_name}' to launch file: {launch_file}", fg="green")

def add_default_launch_file(pkg_name):
    """Auto-generates a default.launch.py that includes the main package launch file."""
    launch_dir = os.path.join('src', pkg_name, 'launch')
    os.makedirs(launch_dir, exist_ok=True)
    default_launch_file = os.path.join(launch_dir, "default.launch.py")
    pkg_specific_launch_file = f"{pkg_name}_launch.py"

    # Don't overwrite if it exists to preserve user customizations
    if os.path.exists(default_launch_file):
        return

    boilerplate = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ""
    This is the default launch file for the '{pkg_name}' package.
    It is launched when running 'framework launch --all'.
    By default, it includes the package-specific launch file.
    ""
    pkg_specific_launch_file_path = os.path.join(
        get_package_share_directory('{pkg_name}'),
        'launch',
        '{pkg_specific_launch_file}'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg_specific_launch_file_path))
    ])
"""
    with open(default_launch_file, 'w') as f:
        f.write(boilerplate)
    click.secho(f"✓ Auto-generated default launch file: {default_launch_file}", fg="green")
