import os
import click
import sys
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from genesys_cli.utils import get_sourcing_command

@click.command()
@click.argument('world_file')
def sim(world_file):
    """Launches a simulation with a specified world file and robot models (supports multiple URDF/SDF, namespaced)."""

    # 1. Verify workspace state
    if not os.path.isdir('install'):
        click.secho("Error: 'install' directory not found. Have you built the workspace yet?", fg="red")
        click.secho("Try running 'genesys build' first.", fg="yellow")
        sys.exit(1)

    sim_worlds_dir = 'sim/worlds'
    if not os.path.isdir(sim_worlds_dir):
        click.secho(f"Error: Simulation worlds directory not found at './{sim_worlds_dir}'", fg="red")
        click.secho("Ensure your project was created with 'genesys new'.", fg="yellow")
        sys.exit(1)

    world_path = os.path.join(sim_worlds_dir, world_file)
    if not os.path.exists(world_path):
        click.secho(f"Error: World file not found: {world_path}", fg="red")
        sys.exit(1)

        # 2. Find robot models in sim/models
    def is_valid_robot_model(path: str) -> bool:
        """Check if the XML root is <sdf> or <robot> (valid for ROS2 spawn_entity)."""
        try:
            tree = ET.parse(path)
            root = tree.getroot().tag.lower()
            return root in ("sdf", "robot")
        except Exception:
            return False

    sim_models_dir = 'sim/models'
    robot_models = []
    if os.path.isdir(sim_models_dir):
        for file in os.listdir(sim_models_dir):
            if file.endswith(('.urdf', '.xacro', '.sdf')):
                model_path = os.path.join(sim_models_dir, file)

                if is_valid_robot_model(model_path):
                    robot_models.append(model_path)
                    click.echo(f"Found robot model: {model_path}")
                else:
                    click.secho(
                        f"⚠️  Skipping {model_path} (not a valid URDF/SDF root element for ROS2)",
                        fg="yellow"
                    )

    if not robot_models:
        click.secho("Warning: No valid robot models (.urdf/.xacro/.sdf) found in 'sim/models/'.", fg="yellow")
        click.secho("Gazebo will be launched without robots.", fg="yellow")

    # 3. Generate temporary launch file
    workspace_root_abs = os.getcwd()
    world_path_abs = os.path.join(workspace_root_abs, world_path)

    launch_content_parts = [
        "import os",
        "from ament_index_python.packages import get_package_share_directory",
        "from launch import LaunchDescription",
        "from launch.actions import IncludeLaunchDescription",
        "from launch.launch_description_sources import PythonLaunchDescriptionSource",
        "from launch_ros.actions import Node",
        "",
        "def generate_launch_description():",
        "    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')",
        f"    world_path = '{world_path_abs}'",
        "    gzserver_cmd = IncludeLaunchDescription(",
        "        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),", 
        "        launch_arguments={'world': world_path, 'verbose': 'true'}.items()",
        "    )",
        "    gzclient_cmd = IncludeLaunchDescription(",
        "        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))",
        "    )",
        "    ld = LaunchDescription([gzserver_cmd, gzclient_cmd])",
    ]

    for idx, model_path in enumerate(robot_models, start=1):
        abs_model_path = os.path.join(workspace_root_abs, model_path)
        ext = os.path.splitext(abs_model_path)[1].lower()
        base_name = os.path.splitext(os.path.basename(abs_model_path))[0]

        # Namespace for this robot (robot1, robot2, ...)
        ns = f"robot{idx}"
        robot_name = f"{base_name}_{ns}"

        if ext in [".urdf", ".xacro"]:
            launch_content_parts.extend([
                f"    with open('{abs_model_path}', 'r') as infp:",
                "        robot_desc = infp.read()",
                f"    robot_state_publisher_node_{ns} = Node(",
                "        package='robot_state_publisher',",
                "        executable='robot_state_publisher',",
                "        output='screen',",
                f"        namespace='{ns}',",
                "        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]",
                "    )",
                f"    spawn_entity_node_{ns} = Node(",
                "        package='gazebo_ros',",
                "        executable='spawn_entity.py',",
                f"        arguments=['-entity', '{robot_name}', '-topic', 'robot_description'],",
                f"        namespace='{ns}',",
                "        output='screen'",
                "    )",
                f"    ld.add_action(robot_state_publisher_node_{ns})",
                f"    ld.add_action(spawn_entity_node_{ns})",
            ])

        elif ext == ".sdf":
            launch_content_parts.extend([
                f"    spawn_entity_node_{ns} = Node(",
                "        package='gazebo_ros',",
                "        executable='spawn_entity.py',",
                f"        arguments=['-entity', '{robot_name}', '-file', '{abs_model_path}'],",
                f"        namespace='{ns}',",
                "        output='screen'",
                "    )",
                f"    ld.add_action(spawn_entity_node_{ns})",
            ])

    launch_content_parts.append("    return ld")
    launch_content = "\n".join(launch_content_parts)

    temp_launch_file = None
    process = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='_sim_launch.py') as f:
            temp_launch_file = f.name
            f.write(launch_content)
        source_prefix, shell_exec = get_sourcing_command(clean_env=True)
        command_to_run = source_prefix + f"ros2 launch {temp_launch_file}"
        click.echo(f"Executing: ros2 launch {os.path.basename(temp_launch_file)}")
        process = subprocess.Popen(command_to_run, shell=True, executable=shell_exec)
        click.secho("\n✓ Simulation is starting...", fg="cyan")
        if robot_models:
            click.echo(f"  {len(robot_models)} robot(s) have been spawned with namespaces /robot1, /robot2, ...")
            click.echo("  Run control/logic nodes in the correct namespace (e.g., `ros2 run pkg node --ros-args -r __ns:=/robot1`).")
        process.wait()
    except KeyboardInterrupt:
        click.echo("\nSimulation interrupted by user.")
        if process and process.poll() is None:
            process.terminate()
    except Exception as e:
        click.secho(f"An error occurred during simulation launch: {e}", fg="red")
    finally:
        if temp_launch_file and os.path.exists(temp_launch_file):
            os.remove(temp_launch_file)
