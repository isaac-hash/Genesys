import os
import click
import subprocess
import sys
import tempfile
from genesys_cli.utils import get_sourcing_command

@click.command(context_settings=dict(ignore_unknown_options=True))
@click.argument('launch_target', required=False)
@click.option('--all', 'launch_all', is_flag=True, help='Launch the default.launch.py from all packages.')
@click.argument('launch_args', nargs=-1, type=click.UNPROCESSED)
@click.pass_context
def launch(ctx, launch_target, launch_all, launch_args):
    """
    Launches ROS 2 nodes.

    Can be used in several ways:
    - genesys launch --all (launches default.launch.py from all packages)
    - genesys launch <pkg_name>:<launch_file.py>
    - genesys launch <pkg_name> (launches <pkg_name>_launch.py by default)
    - You can pass launch arguments at the end, e.g., `log_level:=debug`
    """
    if launch_all and launch_target:
        click.secho("Error: Cannot use --all with a specific launch target.", fg="red")
        sys.exit(1)
    
    if not launch_all and not launch_target:
        click.secho("Error: Must provide a launch target or use the --all flag.", fg="red")
        click.echo(ctx.get_help())
        sys.exit(1)

    # Verify we are in a workspace that has been built.
    if not os.path.isdir('install'):
        click.secho("Error: 'install' directory not found. Have you built the workspace yet?", fg="red")
        click.secho("Try running 'genesys build' first.", fg="yellow")
        sys.exit(1)

    source_prefix, shell_exec = get_sourcing_command(clean_env=True)

    if launch_all:
        click.echo("Searching for 'default.launch.py' in all packages...")
        
        src_dir = 'src'
        if not os.path.isdir(src_dir):
            click.secho("Error: 'src' directory not found. This command must be run from the workspace root.", fg="red")
            sys.exit(1)

        packages = [d for d in os.listdir(src_dir) if os.path.isdir(os.path.join(src_dir, d))]
        
        default_launches = []
        for pkg in packages:
            default_launch_file_src = os.path.join(src_dir, pkg, 'launch', 'default.launch.py')
            if os.path.exists(default_launch_file_src):
                default_launches.append((pkg, 'default.launch.py'))
        
        if not default_launches:
            click.secho("No 'default.launch.py' files found in any package.", fg="yellow")
            return

        click.echo("Found default launch files in:")
        for pkg, _ in default_launches:
            click.echo(f"  - {pkg}")

        # Generate a master launch file
        
        launch_includes = []
        for pkg, launch_file in default_launches:
            launch_includes.append(
                f"        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('{pkg}'), 'launch', '{launch_file}'))),"
            )
        
        launch_content_parts = [
            "import os",
            "from ament_index_python.packages import get_package_share_directory",
            "from launch import LaunchDescription",
            "from launch.actions import IncludeLaunchDescription",
            "from launch.launch_description_sources import PythonLaunchDescriptionSource",
            "",
            "def generate_launch_description():",
            "    return LaunchDescription([",
            "        " + "\n".join(launch_includes) + "\n",
            "    ])"
        ]
        launch_content = "\n".join(launch_content_parts)

        temp_launch_file = None
        process = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='_all_launch.py') as f:
                temp_launch_file = f.name
                f.write(launch_content)
            
            command_to_run = source_prefix + f"ros2 launch {temp_launch_file}"
            click.echo(f"\nExecuting master launch file: {os.path.basename(temp_launch_file)}")
            process = subprocess.Popen(command_to_run, shell=True, executable=shell_exec)
            process.wait()
        except KeyboardInterrupt:
            click.echo("\nLaunch interrupted by user.")
            if process and process.poll() is None:
                process.terminate()
        except Exception as e:
            click.secho(f"An error occurred during launch: {e}", fg="red")
        finally:
            if temp_launch_file and os.path.exists(temp_launch_file):
                os.remove(temp_launch_file)

    else: # launch_target is provided
        if ':' in launch_target:
            pkg_name, launch_file = launch_target.split(':', 1)
        else:
            pkg_name = launch_target
            launch_file = f"{pkg_name}_launch.py"
            click.echo(f"No launch file specified, defaulting to '{launch_file}'")

        launch_command = f"ros2 launch {pkg_name} {launch_file}"
        command_to_run = source_prefix + launch_command
        if launch_args:
            command_to_run += " " + " ".join(launch_args)

        click.echo(f"Executing: {launch_command}")

        try:
            process = subprocess.Popen(command_to_run, shell=True, executable=shell_exec)
            process.wait()
        except KeyboardInterrupt:
            click.echo("\nLaunch interrupted by user.")
            if process and process.poll() is None:
                process.terminate()
        except Exception as e:
            click.secho(f"An error occurred during launch: {e}", fg="red")
