import os
import re
import click
import sys
from genesys_cli.commands.templates import get_cpp_component_templates

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

    # If 'console_scripts' doesn't exist, add it inside 'entry_points'.
    if "'console_scripts'" not in content and '"console_scripts"' not in content:
        entry_points_match = re.search(r'entry_points\s*=\s*\{', content)
        if not entry_points_match:
            click.secho(f"Error: Could not find 'entry_points' in {setup_file}.", fg="red")
            return

        # Determine indentation from the 'entry_points' line
        line_start = content.rfind('\n', 0, entry_points_match.start()) + 1
        indentation = " " * (entry_points_match.start() - line_start)

        insertion_point = entry_points_match.end()
        # Insert the new 'console_scripts' list.
        new_entry_point = f"\n{indentation}    'console_scripts': [],"
        content = content[:insertion_point] + new_entry_point + content[insertion_point:]


    # Use re.DOTALL to match newlines. Use named groups for clarity.
    match = re.search(
        r"(?P<pre>'console_scripts'\s*:\s*\[|\"console_scripts\"\s*:\s*\[)(?P<scripts>.*?)(?P<post>\])",
        content,
        re.DOTALL
    )



    if not match:
        click.secho(f"Error: Could not find 'console_scripts' in {setup_file}.", fg="red")
        return

    scripts_content = match.group('scripts')

    # Always add a comma at the end for consistency, making it easier to append new entries.
    new_entry = f"'{node_name} = {pkg_name}.{node_module_name}:main',"

    # Check if node is already registered
    if f"'{node_name} ='" in scripts_content or f'"{node_name} ="' in scripts_content:
        click.secho(f"Node '{node_name}' already exists in {setup_file}.", fg="yellow")
        return


    # Determine the base indentation of the 'entry_points' dictionary
    entry_points_match = re.search(r'^\s*entry_points\s*=\s*\{', content, re.MULTILINE)
    if not entry_points_match:
        click.secho(f"Error: Could not find 'entry_points' dictionary in {setup_file}.", fg="red")
        return
    base_indentation = entry_points_match.start() - content.rfind('\n', 0, entry_points_match.start()) - 1
    
    # The indentation for items within the list should be base_indentation + 8 spaces
    item_indentation = " " * (base_indentation + 8)

    # If the list is not empty, add a newline before the new entry.
    if scripts_content.strip():
        insertion = f"\n{item_indentation}{new_entry}"
    else: # The list is empty, add indentation and newlines around it.
        # For an empty list, we want the new entry to be indented by 8 spaces,
        # and the closing bracket to be indented by 4 spaces relative to entry_points.
        insertion = f"\n{item_indentation}{new_entry}\n" + (" " * (base_indentation + 4))

    # Insert the new entry right before the closing bracket of the list.
    insertion_point = match.end('scripts')
    updated_content = content[:insertion_point] + insertion + content[insertion_point:]

    with open(setup_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Registered '{node_name}' in {setup_file}", fg="green")


def add_python_component_entry_point(pkg_name, component_name):
    """Adds a new rclpy_components entry to a package's setup.py file."""
    setup_file = os.path.join('src', pkg_name, 'setup.py')
    component_module_name = component_name.replace('.py', '')

    with open(setup_file, 'r') as f:
        content = f.read()

    # Check if rclpy_components entry point exists, if not, add it.
    if 'rclpy_components' not in content:
        entry_points_match = re.search(r'entry_points\s*=\s*\{', content)
        if not entry_points_match:
            click.secho(f"Error: Could not find 'entry_points' in {setup_file}.", fg="red")
            return
        
        insertion_point = entry_points_match.end()
        new_entry_point = """
    'rclpy_components': [],
"""
        content = content[:insertion_point] + new_entry_point + content[insertion_point:]

    # Use re.DOTALL to match newlines. Use named groups for clarity.
    match = re.search(
        r"(?P<pre>(['\"])rclpy_components\1\s*:\s*\[)(?P<scripts>.*?)(?P<post>\])",
        content,
        re.DOTALL
    )

    if not match:
        click.secho(f"Error: Could not find 'rclpy_components' in {setup_file}.", fg="red")
        return

    scripts_content = match.group('scripts')

    # Check if component is already registered
    if f"'{component_name} ='" in scripts_content or f'"{component_name} ="' in scripts_content:
        click.secho(f"Component '{component_name}' already exists in {setup_file}.", fg="yellow")
        return

    # Always add a comma at the end for consistency, making it easier to append new entries.
    new_entry = f"'{component_name} = {pkg_name}.{component_module_name}:get_node_factory',"

    # Determine the base indentation of the 'entry_points' dictionary
    entry_points_match = re.search(r'^\s*entry_points\s*=\s*\{', content, re.MULTILINE)
    if not entry_points_match:
        click.secho(f"Error: Could not find 'entry_points' dictionary in {setup_file}.", fg="red")
        return
    base_indentation = entry_points_match.start() - content.rfind('\n', 0, entry_points_match.start()) - 1
    
    # The indentation for items within the list should be base_indentation + 8 spaces
    item_indentation = " " * (base_indentation + 8)

    # If the list is not empty, add a newline before the new entry.
    if scripts_content.strip():
        insertion = f"\n{item_indentation}{new_entry}"
    else: # The list is empty, add indentation and newlines around it.
        # For an empty list, we want the new entry to be indented by 8 spaces,
        # and the closing bracket to be indented by 4 spaces relative to entry_points.
        insertion = f"\n{item_indentation}{new_entry}\n" + (" " * (base_indentation + 4))

    # Insert the new entry right before the closing bracket of the list.
    insertion_point = match.end('scripts')
    updated_content = content[:insertion_point] + insertion + content[insertion_point:]

    with open(setup_file, 'w') as f:
        f.write(updated_content)
    
    click.secho(f"✓ Registered '{component_name}' as a component in {setup_file}", fg="green")

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

def add_action_definition(pkg_name):
    """Creates a local Fibonacci.action and updates CMake/package.xml to generate interfaces."""
    pkg_path = os.path.join('src', pkg_name)
    
    # 1. Create the .action file
    action_dir = os.path.join(pkg_path, 'action')
    os.makedirs(action_dir, exist_ok=True)
    action_file = "Fibonacci.action"
    action_file_path = os.path.join(action_dir, action_file)
    
    if not os.path.exists(action_file_path):
        with open(action_file_path, 'w') as f:
            f.write("int32 order\n---\nint32[] sequence\n---\nint32[] partial_sequence\n")
        click.secho(f"✓ Created action definition: {action_file_path}", fg="green")

    # 2. Update package.xml
    package_xml_path = os.path.join(pkg_path, 'package.xml')
    with open(package_xml_path, 'r') as f: content = f.read()
    
    tags_to_add = []
    if "rosidl_default_generators" not in content:
        tags_to_add.append("  <build_depend>rosidl_default_generators</build_depend>")
    if "rosidl_default_runtime" not in content:
        tags_to_add.append("  <exec_depend>rosidl_default_runtime</exec_depend>")
    if "rosidl_interface_packages" not in content:
        tags_to_add.append("  <member_of_group>rosidl_interface_packages</member_of_group>")

    if tags_to_add and "</package>" in content:
        insertion = "\n" + "\n".join(tags_to_add) + "\n"
        content = content.replace("</package>", insertion + "</package>")
        with open(package_xml_path, 'w') as f: f.write(content)
        click.secho(f"✓ Updated {package_xml_path} for action generation.", fg="green")

    # 3. Update CMakeLists.txt safely
    cmake_path = os.path.join(pkg_path, "CMakeLists.txt")
    with open(cmake_path, 'r') as f: content = f.read()

    # Add generators dependency if missing
    if "rosidl_default_generators" not in content:
        content = content.replace(
            "find_package(ament_cmake REQUIRED)", 
            "find_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)"
        )

    # Handle rosidl_generate_interfaces
    action_entry = f'"action/{action_file}"'
    if "rosidl_generate_interfaces" in content:
        # Block exists, append action if not present
        if action_file not in content:
            pattern = f"rosidl_generate_interfaces(${{PROJECT_NAME}}"
            replacement = f"{pattern}\n  {action_entry}"
            content = content.replace(pattern, replacement)
    else:
        # Block missing, create it
        rosidl_block = f'''
rosidl_generate_interfaces(${{PROJECT_NAME}}
  {action_entry}
)
'''
        # FIX: Insert BEFORE the node definitions so targets exist when nodes are added
        if "# --- REGULAR NODES INSERTION POINT ---" in content:
            content = content.replace(
                "# --- REGULAR NODES INSERTION POINT ---", 
                f"{rosidl_block}\n# --- REGULAR NODES INSERTION POINT ---"
            )
        elif "ament_package()" in content:
             # Fallback
             content = content.replace("ament_package()", f"{rosidl_block}\nament_package()")
    
    with open(cmake_path, 'w') as f: f.write(content)
    click.secho(f"✓ Updated {cmake_path} for action generation.", fg="green")

def add_cpp_executable(pkg_name, node_name, dependencies=None):
    """Adds a new executable to CMakeLists.txt with dynamic dependencies."""
    cmake_file = os.path.join('src', pkg_name, 'CMakeLists.txt')
    if not os.path.exists(cmake_file):
        click.secho(f"Error: {cmake_file} not found.", fg="red")
        return

    with open(cmake_file, 'r') as f: content = f.read()

    if f"add_executable({node_name} " in content:
        click.secho(f"Node '{node_name}' already registered.", fg="yellow")
        return

    if dependencies is None: dependencies = ["rclcpp", "rclcpp_lifecycle", "std_msgs"]
    deps_str = "\n  ".join(dependencies)
    node_src_file = f"src/{node_name}.cpp"

    # FIX: Removed 'if(TARGET...)' wrapper. 
    # If rosidl_generate_interfaces is called, we MUST link it. 
    # We rely on CMake's lazy target resolution or correct ordering from add_action_definition.
    new_block = f'''
# --- Node: {node_name} ---
add_executable({node_name} {node_src_file})

target_include_directories({node_name} PUBLIC
  $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
  $<INSTALL_INTERFACE:include/${{PROJECT_NAME}}>
)

ament_target_dependencies({node_name}
  {deps_str}
)

# Link against generated interfaces using modern CMake targets
rosidl_get_typesupport_target(ts_target "${{PROJECT_NAME}}" "rosidl_typesupport_cpp")
if(TARGET "${{ts_target}}")
  target_link_libraries({node_name} "${{ts_target}}")
endif()

install(TARGETS
  {node_name}
  DESTINATION lib/${{PROJECT_NAME}}
)
'''

    marker = "# --- REGULAR NODES INSERTION POINT ---"
    if marker in content:
        content = content.replace(marker, f"{marker}\n{new_block}")
        with open(cmake_file, 'w') as f: f.write(content)
        click.secho(f"✓ Registered executable '{node_name}' in {cmake_file}", fg="green")
    else:
        click.secho(f"Error: Insertion marker not found in {cmake_file}.", fg="red")
def add_install_rule_for_launch_dir_cpp(pkg_name):
    """Adds the install rule for the launch directory to CMakeLists.txt."""
    cmake_file = os.path.join('src', pkg_name, 'CMakeLists.txt')
    if not os.path.exists(cmake_file):
        return  # Not a C++ package

    with open(cmake_file, 'r') as f:
        lines = f.readlines()

    # Check if the rule already exists using a more robust regex
    content = "".join(lines)
    if re.search(r"install\s*\(\s*DIRECTORY\s+launch", content):
        return

    # Find the ament_package() call to insert before it
    ament_line_idx = None
    for i, line in enumerate(lines):
        if re.search(r"^\s*ament_package\s*\(\s*\)", line):
            ament_line_idx = i
            break

    if ament_line_idx is None:
        click.secho(f"Warning: Could not find ament_package() call in {cmake_file}. Cannot add launch install rule.", fg="yellow")
        return

    new_cmake_commands = f'''install(
  DIRECTORY launch
  DESTINATION share/${{PROJECT_NAME}})

'''
    # insert before ament_package()
    lines.insert(ament_line_idx, new_cmake_commands)

    with open(cmake_file, 'w') as f:
        f.write("".join(lines))
    
    click.secho(f"✓ Added launch directory install rule to {cmake_file}", fg="green")

def add_launch_file_boilerplate(pkg_name, node_name):
    """Auto-generates a boilerplate launch file for a new node."""
    launch_dir = os.path.join('src', pkg_name, 'launch')
    os.makedirs(launch_dir, exist_ok=True)
    launch_file = os.path.join(launch_dir, f"{pkg_name}_launch.py")
    
    # Only create a launch file if it doesn't already exist to avoid overwriting a custom one
    if not os.path.exists(launch_file):
        boilerplate = f'''from launch import LaunchDescription
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
'''
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
    new_node_block = f'''Node(
    package='{pkg_name}',
    executable='{node_name}',
    name='{node_name}',
    output='screen',
    emulate_tty=True
)'''

    if f"executable='{node_name}'" in content:
        click.secho(f"Launch file already contains '{node_name}'.", fg="yellow")
        return

    # Find the LaunchDescription list content
    match = re.search(
        r'(return LaunchDescription\(\[)(?P<nodes>.*?)(?P<post>\]\))',
        content,
        re.DOTALL
    )

    if not match:
        click.secho(f"Error: Could not find 'return LaunchDescription([' in {launch_file}.", fg="red")
        return

    nodes_in_ld = match.group('nodes').strip()
    insertion_point = match.end('nodes')

    # Determine indentation
    return_ld_start_index = content.rfind('\n', 0, match.start()) + 1
    base_indentation = match.start() - return_ld_start_index
    item_indentation = " " * (base_indentation + 4)

    # Remove any trailing comma from the existing nodes content
    if nodes_in_ld.endswith(','):
        nodes_in_ld = nodes_in_ld.rstrip(',')

    if nodes_in_ld:
        # If there are existing nodes, add a comma and a newline before the new node
        new_list_content = f"{nodes_in_ld},\n{item_indentation}{new_node_block}"
    else:
        # If no existing nodes, just add a newline and the new node
        new_list_content = f"\n{item_indentation}{new_node_block}"

    updated_content = content[:match.start('nodes')] + new_list_content + content[match.end('nodes'):]

    with open(launch_file, 'w') as f:
        f.write(updated_content)

    click.secho(f"✓ Added '{node_name}' to launch file: {launch_file}", fg="green")

def add_node_to_mixed_launch(pkg_name, node_name):
    """Adds a new Node entry into the package's mixed launch file if it exists."""
    launch_file = os.path.join('src', pkg_name, 'launch', "mixed_launch.py")
    if not os.path.exists(launch_file):
        return

    with open(launch_file, 'r') as f:
        content = f.read()

    new_node_block = f"""
        Node(
            package='{pkg_name}',
            executable='{node_name}',
            name='{node_name}'
        ),
"""

    if f"executable='{node_name}'" in content:
        click.secho(f"Launch file already contains '{node_name}'.", fg="yellow")
        return

    # Insert the new node block into the regular_nodes list.
    updated_content = re.sub(r"(regular_nodes\s*=\s*\[\n)",
                           rf"\g<1>{new_node_block}",
                           content)

    with open(launch_file, 'w') as f:
        f.write(updated_content)

    click.secho(f"✓ Added '{node_name}' to mixed launch file: {launch_file}", fg="green")

def add_component_to_mixed_launch(pkg_name, component_name):
    """Adds a new ComposableNode entry into the package's mixed launch file if it exists."""
    launch_file = os.path.join('src', pkg_name, 'launch', "mixed_launch.py")
    if not os.path.exists(launch_file):
        return

    with open(launch_file, 'r') as f:
        content = f.read()

    # Convert snake_case to PascalCase for the C++ class name
    class_name = "".join(word.capitalize() for word in component_name.split('_'))
    plugin_name = f"{pkg_name}::{class_name}"

    new_component_block = f"""
        ComposableNode(
            package='{pkg_name}',
            plugin='{plugin_name}',
            name='{component_name.lower()}'
        ),
"""

    if plugin_name in content:
        click.secho(f"Launch file already contains '{component_name}'.", fg="yellow")
        return

    # Insert the new component block into the composable_nodes list.
    updated_content = re.sub(r"(composable_nodes\s*=\s*\[\n)", rf"\g<1>{new_component_block}", content)

    with open(launch_file, 'w') as f:
        f.write(updated_content)

    click.secho(f"✓ Added '{component_name}' to mixed launch file: {launch_file}", fg="green")


def add_default_launch_file(pkg_name):
    """Auto-generates a default.launch.py that includes the main package launch file."""
    launch_dir = os.path.join('src', pkg_name, 'launch')
    os.makedirs(launch_dir, exist_ok=True)
    default_launch_file = os.path.join(launch_dir, "default.launch.py")
    pkg_specific_launch_file = f"{pkg_name}_launch.py"

    # Don't overwrite if it exists to preserve user customizations
    if os.path.exists(default_launch_file):
        return

    boilerplate = f'''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    This is the default launch file for the '{pkg_name}' package.
    It is launched when running 'framework launch --all'.
    By default, it includes the package-specific launch file.
    """
    pkg_specific_launch_file_path = os.path.join(
        get_package_share_directory('{pkg_name}'),
        'launch',
        '{pkg_specific_launch_file}'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg_specific_launch_file_path))
    ])
'''
    with open(default_launch_file, 'w') as f:
        f.write(boilerplate)
    click.secho(f"✓ Auto-generated default launch file: {default_launch_file}", fg="green")

def add_cpp_dependencies_to_package_xml(pkg_name, dependencies):
    """Adds <depend> tags to a package.xml file for C++ packages."""
    package_xml_file = os.path.join('src', pkg_name, 'package.xml')
    if not os.path.exists(package_xml_file):
        click.secho(f"Warning: {package_xml_file} not found. Cannot add dependencies.", fg="yellow")
        return

    with open(package_xml_file, 'r') as f:
        content = f.read()

    # Find the buildtool_depend to insert after
    buildtool_depend_match = re.search(r'(<buildtool_depend>ament_cmake</buildtool_depend>)', content)
    if not buildtool_depend_match:
        buildtool_depend_match = re.search(r'(<buildtool_depend>ament_cmakepp</buildtool_depend>)', content)
        if not buildtool_depend_match:
            click.secho(f"Warning: Could not find <buildtool_depend> in {package_xml_file}. Cannot add dependencies.", fg="yellow")
            return

    insertion_point = buildtool_depend_match.end()
    
    deps_to_add_str = ""
    for dep in dependencies:
        if f"<depend>{dep}</depend>" not in content and f"<build_depend>{dep}</build_depend>" not in content:
             deps_to_add_str += f"\n  <depend>{dep}</depend>"

    if deps_to_add_str:
        updated_content = content[:insertion_point] + deps_to_add_str + content[insertion_point:]
        with open(package_xml_file, 'w') as f:
            f.write(updated_content)
        click.secho(f"✓ Added dependencies to {package_xml_file}", fg="green")

def add_service_definition(pkg_name):
    """Creates the AddTwoInts.srv file and updates CMake/package.xml for it."""
    pkg_path = os.path.join('src', pkg_name)
    
    # 1. Create the .srv file
    srv_dir = os.path.join(pkg_path, 'srv')
    os.makedirs(srv_dir, exist_ok=True)
    srv_file = "AddTwoInts.srv"
    srv_file_path = os.path.join(srv_dir, srv_file)
    
    if not os.path.exists(srv_file_path):
        with open(srv_file_path, 'w') as f:
            f.write("int64 a\nint64 b\n---\nint64 sum\n")
        click.secho(f"✓ Created service definition: {srv_file_path}", fg="green")

    # 2. Update package.xml
    package_xml_path = os.path.join(pkg_path, 'package.xml')
    with open(package_xml_path, 'r') as f:
        content = f.read()
    
    tags_to_add = []
    if "rosidl_default_generators" not in content:
        tags_to_add.append("  <build_depend>rosidl_default_generators</build_depend>")
    if "rosidl_default_runtime" not in content:
        tags_to_add.append("  <exec_depend>rosidl_default_runtime</exec_depend>")
    if "rosidl_interface_packages" not in content:
        tags_to_add.append("  <member_of_group>rosidl_interface_packages</member_of_group>")

    if tags_to_add and "</package>" in content:
        insertion = "\n" + "\n".join(tags_to_add) + "\n"
        content = content.replace("</package>", insertion + "</package>")
        with open(package_xml_path, 'w') as f:
            f.write(content)
        click.secho(f"✓ Updated {package_xml_path} for service generation.", fg="green")

    # 3. Update CMakeLists.txt
    cmake_path = os.path.join(pkg_path, "CMakeLists.txt")
    with open(cmake_path, 'r') as f:
        content = f.read()

    # Add dependency package finding if missing
    if "rosidl_default_generators" not in content:
        content = content.replace(
            "find_package(ament_cmake REQUIRED)", 
            "find_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)"
        )
    
    srv_entry = f'"srv/{srv_file}"'

    if "rosidl_generate_interfaces" in content:
        # FIX: If block exists (e.g. from Action), append the srv file if not present
        if srv_file not in content:
            pattern = f"rosidl_generate_interfaces(${{PROJECT_NAME}}"
            replacement = f"{pattern}\n  {srv_entry}"
            content = content.replace(pattern, replacement)
    else:
        # Create new block
        rosidl_block = f'''
rosidl_generate_interfaces(${{PROJECT_NAME}}
  {srv_entry}
)
'''
        # Insert BEFORE nodes to ensure targets are generated first
        if "# --- REGULAR NODES INSERTION POINT ---" in content:
            content = content.replace(
                "# --- REGULAR NODES INSERTION POINT ---", 
                f"{rosidl_block}\n# --- REGULAR NODES INSERTION POINT ---"
            )
        elif "ament_package()" in content:
            content = content.replace(
                "ament_package()", 
                f"{rosidl_block}\nament_package()"
            )
        
    with open(cmake_path, 'w') as f:
        f.write(content)

    click.secho(f"✓ Updated {cmake_path} for service generation.", fg="green")
def add_component_to_regular_launch(pkg_name, component_name):
    """
    Adds a new ComposableNode entry into the package's regular launch file (pkg_name_launch.py).
    If a ComposableNodeContainer is not present, it will be added.
    """
    launch_file = os.path.join('src', pkg_name, 'launch', f"{pkg_name}_launch.py")
    if not os.path.exists(launch_file):
        click.secho(f"Error: Regular launch file '{launch_file}' not found.", fg="red")
        return

    with open(launch_file, 'r') as f:
        content = f.read()

    # Check if component is already registered
    if f"plugin='{component_name}'" in content:
        click.secho(f"Launch file already contains '{component_name}'.", fg="yellow")
        return

    # Imports to ensure are present
    imports_to_add = []
    if 'from launch_ros.actions import ComposableNodeContainer' not in content:
        imports_to_add.append('from launch_ros.actions import ComposableNodeContainer')
    if 'from launch_ros.descriptions import ComposableNode' not in content:
        imports_to_add.append('from launch_ros.descriptions import ComposableNode')
    
    if imports_to_add:
        # Find a good place to insert imports, e.g., after existing launch imports
        match_launch_import = re.search(r'(from launch import LaunchDescription)', content)
        if match_launch_import:
            insertion_point = match_launch_import.end()
            content = content[:insertion_point] + "\n" + "\n".join(imports_to_add) + content[insertion_point:]
        else:
            # Fallback if no launch import found, add at top
            content = "\n".join(imports_to_add) + "\n" + content

    # Check if a ComposableNodeContainer already exists
    container_match = re.search(r'ComposableNodeContainer\(', content)
    
    new_component_block = f"""ComposableNode(
    package='{pkg_name}',
    plugin='{component_name}',
    name='{component_name}'
)
"""

    if container_match:
        # Container exists, find its composable_node_descriptions list and insert
        match = re.search(
            r'(composable_node_descriptions\s*=\s*\[)(?P<nodes>.*?)(?P<post>\])',
            content,
            re.DOTALL
        )
        if match:
            scripts_content = match.group('nodes').strip() # strip to check if truly empty
            insertion_point = match.end('nodes')
            
            # Determine indentation
            # Find the start of the line where 'composable_node_descriptions = [' begins
            list_start_index = content.rfind('\n', 0, match.start()) + 1
            # Calculate the indentation of 'composable_node_descriptions = ['
            base_indentation = match.start() - list_start_index
            # The items in the list should be indented by base_indentation + 4 spaces
            item_indentation = " " * (base_indentation + 4)

            # Remove any trailing comma from the existing components content
            if scripts_content.endswith(','):
                scripts_content = scripts_content.rstrip(',')

            if scripts_content: # If there are existing nodes
                new_list_content = f"{scripts_content},\n{item_indentation}{new_component_block}"
            else: # If the list is empty
                new_list_content = f"\n{item_indentation}{new_component_block}"

            content = content[:match.start('nodes')] + new_list_content + content[match.end('nodes'):]
        else:
            click.secho(f"Warning: Could not find 'composable_node_descriptions' in existing container in {launch_file}. Component not added.", fg="yellow")
            return
    else:
        # No container, add a new one and the component
        indented_component_block = new_component_block.replace('\n', '\n        ').strip()
        container_block = f"""container = ComposableNodeContainer(
    name='{pkg_name}_component_container',
    namespace='',
    package='rclpy_components',
    executable='component_container',
    composable_node_descriptions=[
        {indented_component_block}
    ],
    output='screen',
)"""
        # Find the return LaunchDescription([ and insert container before it
        match_return = re.search(r'(return LaunchDescription\(\[)(?P<nodes>.*?)(?P<post>\]\))', content, re.DOTALL)
        if match_return:
            insertion_point_for_container_ref = match_return.end('nodes')
            nodes_in_ld = match_return.group('nodes').strip()
            
            container_ref_to_add = "container"
            # if nodes_in_ld:
            #     # If there are existing nodes, add a comma before inserting the container reference
            #     container_ref_to_add = ",\n        " + container_ref_to_add
            # else:
            #     # If no existing nodes, just add the container reference with proper indentation
            #     container_ref_to_add = "\n        " + container_ref_to_add
            container_ref_to_add = "\n        " + container_ref_to_add + ",\n"

            # Insert the container reference into the LaunchDescription list
            content = content[:insertion_point_for_container_ref] + container_ref_to_add + content[insertion_point_for_container_ref:]
            
            # Insert the container block itself before the return statement
            # Find the line before 'return LaunchDescription'
            return_ld_start = content.find('return LaunchDescription')
            if return_ld_start != -1:
                # Find the start of the line containing 'return LaunchDescription'
                line_start = content.rfind('\n', 0, return_ld_start) + 1
                # Get the indentation of the 'return LaunchDescription' line
                return_indentation = content[line_start:return_ld_start]
                # Insert the container block at the start of that line with the same indentation
                content = content[:line_start] + return_indentation + container_block + "\n\n" + content[line_start:]
            else:
                click.secho(f"Warning: Could not find 'return LaunchDescription' to insert container block in {launch_file}. Component container not added.", fg="yellow")
                return
        else:
            click.secho(f"Warning: Could not find 'return LaunchDescription' in {launch_file}. Component container not added.", fg="yellow")
            return

    with open(launch_file, 'w') as f:
        f.write(content)

    click.secho(f"✓ Added '{component_name}' to regular launch file: {launch_file}", fg="green")


def make_cpp_component(pkg_name, component_name, component_type):
    """Creates a new C++ component and correctly registers it, adding component-specific
    build infrastructure to CMakeLists.txt."""
    
    class_name = "".join(word.capitalize() for word in component_name.split('_'))
    pkg_path = os.path.join('src', pkg_name)
    
    # --- 1. Get templates ---
    from genesys_cli.commands.templates import get_cpp_component_templates
    hpp_content, cpp_content, _, plugin_content = get_cpp_component_templates(component_type, pkg_name, class_name, component_name, f"A C++ component of type {component_type}")

    # --- 2. Create directories and .hpp/.cpp files ---
    include_dir = os.path.join(pkg_path, 'include', pkg_name)
    src_dir = os.path.join(pkg_path, 'src')
    resource_dir = os.path.join(pkg_path, 'resource')
    os.makedirs(include_dir, exist_ok=True)
    os.makedirs(src_dir, exist_ok=True)
    os.makedirs(resource_dir, exist_ok=True)

    hpp_file_path = os.path.join(include_dir, f"{class_name}.hpp")
    with open(hpp_file_path, 'w') as f: f.write(hpp_content)
    click.secho(f"✓ Created C++ component header: {hpp_file_path}", fg="green")

    cpp_file_path = os.path.join(src_dir, f"{class_name}.cpp")
    with open(cpp_file_path, 'w') as f: f.write(cpp_content)
    click.secho(f"✓ Created C++ component source: {cpp_file_path}", fg="green")

    if component_type == 'Service':
        add_service_definition(pkg_name)

    # --- 3. Create or update register_components.cpp ---
    register_components_path = os.path.join(src_dir, "register_components.cpp")
    include_line = f'#include "{pkg_name}/{class_name}.hpp"'
    register_macro_line = f'RCLCPP_COMPONENTS_REGISTER_NODE({pkg_name}::{class_name})'
    if not os.path.exists(register_components_path):
        initial_content = (
            '// This file is automatically generated by the Genesys CLI.\n'
            '#include "rclcpp_components/register_node_macro.hpp"\n\n'
            f'{include_line}\n{register_macro_line}\n'
        )
        with open(register_components_path, "w") as f: f.write(initial_content)
        click.secho(f"✓ Created component registration file: {register_components_path}", fg="green")
    else:
        with open(register_components_path, "r+") as f:
            content = f.read()
            if class_name not in content:
                f.seek(0, 2)
                f.write(f"\n{include_line}\n{register_macro_line}\n")
                click.secho(f"✓ Updated component registration file for {class_name}", fg="green")

    # --- 4. Update CMakeLists.txt conditionally ---
    cmake_path = os.path.join(pkg_path, "CMakeLists.txt")
    with open(cmake_path, 'r') as f:
        content = f.read()

    component_marker = "# --- Component library ---"
    component_src_file = f"src/{class_name}.cpp"
    
    # Logic to link the component library against generated interfaces (srv/msg/action)
    # This is required so the component can find headers like "cpp_pkg/srv/add_two_ints.hpp"
    link_interfaces_block = f'''
# Link against generated interfaces (messages/services)
rosidl_get_typesupport_target(ts_target "${{PROJECT_NAME}}" "rosidl_typesupport_cpp")
if(TARGET "${{ts_target}}")
  target_link_libraries({pkg_name}_components "${{ts_target}}")
endif()
'''

    if component_marker not in content:
        # --- First component: add the entire component infrastructure ---
        ament_package_marker = "# --- Must be last ---"
        
        find_package_marker = "# --- Find dependencies ---"
        additional_find_packages = (
            '\nfind_package(rclcpp_components REQUIRED)\n'
            'find_package(pluginlib REQUIRED)'
        )
        content = content.replace(find_package_marker, f"{find_package_marker}{additional_find_packages}")

        component_block = f'''
# --- Component library ---
# This library is for all components in the package.
add_library({pkg_name}_components SHARED
  src/register_components.cpp
  {component_src_file}
  # --- COMPONENTS INSERTION POINT ---
)

target_include_directories({pkg_name}_components PUBLIC
  $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
  $<INSTALL_INTERFACE:include/${{PROJECT_NAME}}>)

ament_target_dependencies({pkg_name}_components
  rclcpp
  rclcpp_components
  std_msgs
  pluginlib
)

{link_interfaces_block}

rclcpp_components_register_nodes({pkg_name}_components
  "{pkg_name}::{class_name}"
  # --- COMPONENTS REGISTER INSERTION POINT ---
)

# Add component library to install rules
install(TARGETS 
  {pkg_name}_components
  DESTINATION lib
)

# Install the plugin XML file.
install(
  FILES resource/{pkg_name}_plugin.xml
  DESTINATION share/{pkg_name}
)

# Install the package include directory.
install(
  DIRECTORY include/
  DESTINATION include
)
'''
        content = content.replace(ament_package_marker, f"{component_block}\n{ament_package_marker}")
        click.secho(f"✓ Added component build infrastructure to {cmake_path}", fg="green")

    else:
        # --- Subsequent component: just add to existing infrastructure ---
        
        # FIX: Check if the interface linking is missing (repair existing files)
        if "rosidl_get_typesupport_target" not in content and f"{pkg_name}_components" in content:
            register_marker = f"rclcpp_components_register_nodes({pkg_name}_components"
            if register_marker in content:
                content = content.replace(register_marker, f"{link_interfaces_block}\n{register_marker}")
                click.secho(f"✓ Patched {cmake_path} to link interfaces.", fg="green")

        if component_src_file not in content:
            src_marker = "# --- COMPONENTS INSERTION POINT ---"
            content = content.replace(src_marker, f"{src_marker}\n  {component_src_file}")
            
            reg_marker = "# --- COMPONENTS REGISTER INSERTION POINT ---"
            reg_line = f'  "{pkg_name}::{class_name}"'
            content = content.replace(reg_marker, f"{reg_marker}\n{reg_line}")
            click.secho(f"✓ Updated {cmake_path} for component {class_name}", fg="green")
        else:
            click.secho(f"Component '{class_name}' already in {cmake_path}.", fg="yellow")

    with open(cmake_path, 'w') as f:
        f.write(content)

    # --- 5. Create or update plugin.xml ---
    plugin_xml_path = os.path.join(resource_dir, f"{pkg_name}_plugin.xml")
    plugin_entry_snippet = (
        f'  <class type="{pkg_name}::{class_name}" base_class_type="rclcpp::Node">\n'
        f'    <description>A C++ component of type {component_type}.</description>\n'
        '  </class>'
    )
    if os.path.exists(plugin_xml_path):
        with open(plugin_xml_path, "r+") as f:
            content = f.read()
            if f'type="{pkg_name}::{class_name}"' not in content:
                updated_content = content.replace("</library>", f"{plugin_entry_snippet}\n</library>")
                f.seek(0); f.write(updated_content); f.truncate()
                click.secho(f"✓ Added component entry to plugin XML: {plugin_xml_path}", fg="green")
    else:
        base_plugin_xml = plugin_content.replace("</library>", f"{plugin_entry_snippet}\n</library>")
        with open(plugin_xml_path, "w") as f:
            f.write(base_plugin_xml)
        click.secho(f"✓ Created plugin XML file: {plugin_xml_path}", fg="green")

    # --- 6. Update package.xml for component support ---
    add_cpp_dependencies_to_package_xml(pkg_name, ["rclcpp_components", "pluginlib"])
    package_xml_path = os.path.join(pkg_path, "package.xml")
    with open(package_xml_path, "r+") as f:
        content = f.read()
        if "<export>" not in content:
            content = content.replace("</package>", "\n<export>\n</export>\n</package>")
        if f'plugin="resource/{pkg_name}_plugin.xml"' not in content:
            content = content.replace("</export>", f'  <rclcpp_components plugin="resource/{pkg_name}_plugin.xml"/>\n</export>')
        f.seek(0); f.write(content); f.truncate()
    click.secho(f"✓ Updated package.xml for components", fg="green")
def add_cmake_find_package(pkg_name, package_to_find):
    """Ensures a find_package() call exists in CMakeLists.txt."""
    cmake_path = os.path.join('src', pkg_name, "CMakeLists.txt")
    if not os.path.exists(cmake_path): return

    with open(cmake_path, 'r') as f:
        content = f.read()
    
    if f"find_package({package_to_find}" in content:
        return

    # Insert after the mandatory ament_cmake find_package
    if "find_package(ament_cmake REQUIRED)" in content:
        content = content.replace(
            "find_package(ament_cmake REQUIRED)", 
            f"find_package(ament_cmake REQUIRED)\nfind_package({package_to_find} REQUIRED)"
        )
        with open(cmake_path, 'w') as f:
            f.write(content)
        click.secho(f"✓ Added find_package({package_to_find}) to CMakeLists.txt", fg="green")

def make_cpp_node(pkg_name, node_name, node_type):
    """Creates a new C++ node and registers it in the package."""
    
    class_name = "".join(word.capitalize() for word in node_name.split('_'))
    
    # 1. Get templates
    from genesys_cli.commands.templates import get_cpp_node_template, get_cpp_node_header_template
    hpp_content = get_cpp_node_header_template(node_type, pkg_name, class_name, node_name)
    cpp_content = get_cpp_node_template(node_type, pkg_name, class_name, node_name)

    # 2. Create directories
    pkg_path = os.path.join('src', pkg_name)
    include_dir = os.path.join(pkg_path, 'include', pkg_name)
    src_dir = os.path.join(pkg_path, 'src')
    os.makedirs(include_dir, exist_ok=True)
    os.makedirs(src_dir, exist_ok=True)

    # 3. Create node files
    hpp_file_path = os.path.join(include_dir, f"{class_name}.hpp")
    cpp_file_path = os.path.join(src_dir, f"{node_name}.cpp")

    with open(hpp_file_path, 'w') as f: f.write(hpp_content)
    click.secho(f"✓ Created C++ node header: {hpp_file_path}", fg="green")

    with open(cpp_file_path, 'w') as f: f.write(cpp_content)
    click.secho(f"✓ Created C++ node source: {cpp_file_path}", fg="green")

    if node_type == 'Service':
        add_service_definition(pkg_name)

    # 4. Define Dependencies based on Node Type
    dependencies = ["rclcpp", "std_msgs", "rclcpp_lifecycle"]
    
    if node_type == 'ActionServer' or node_type == 'ActionClient':
        dependencies.append("rclcpp_action")
        add_cmake_find_package(pkg_name, "rclcpp_action")
        
        # A. Generate local action definition
        add_action_definition(pkg_name)
        
        # B. Patch the generated C++ files to use the local action instead of tutorial interfaces
        # This fixes the 'fatal error: action_tutorials_interfaces/action/fibonacci.hpp: No such file'
        for file_path in [hpp_file_path, cpp_file_path]:
            with open(file_path, 'r') as f: content = f.read()
            
            # Replace header include
            content = content.replace(
                "action_tutorials_interfaces/action/fibonacci.hpp", 
                f"{pkg_name}/action/fibonacci.hpp"
            )
            # Replace namespace
            content = content.replace(
                "action_tutorials_interfaces::action::Fibonacci", 
                f"{pkg_name}::action::Fibonacci"
            )
            
            with open(file_path, 'w') as f: f.write(content)
        click.secho(f"✓ Patched C++ files to use local '{pkg_name}::action::Fibonacci'", fg="green")

    # 5. Update CMakeLists.txt (Executable + Dependencies)
    add_cpp_executable(pkg_name, node_name, dependencies=dependencies)

    # 6. Update package.xml
    add_cpp_dependencies_to_package_xml(pkg_name, dependencies)    
