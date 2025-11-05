import os
import click
import sys
import subprocess
import shutil
from genesys_cli.utils import get_sourcing_command

@click.group()
def sim():
    """Manages ROS 2 Gazebo simulations in the sim/ workspace."""
    pass

def _render_template(template_content, context):
    """A simple Jinja2-like renderer."""
    for key, value in context.items():
        template_content = template_content.replace(f'{{{{ {key} }}}}', str(value))
    return template_content

def _get_template_path(template_name):
    """Gets the full path to a template file."""
    return os.path.join(os.path.dirname(__file__), 'templates', 'gazebo', template_name)

@sim.command()
@click.argument('package_name', callback=lambda ctx, param, value: value.lower())
@click.option('--from', 'from_pkg', required=True, help='Source robot description package (e.g., ur_description).')
def create(package_name, from_pkg):
    """Creates a new *_gazebo package in sim/ for a robot."""
    
    if not package_name.endswith('_gazebo'):
        click.secho("Error: package_name must end with '_gazebo'.", fg='red')
        sys.exit(1)

    sim_dir = 'sim'
    if not os.path.isdir(sim_dir):
        os.makedirs(sim_dir)

    package_path = os.path.join(sim_dir, package_name)
    if os.path.exists(package_path):
        click.secho(f"Error: Package '{package_name}' already exists at '{package_path}'.", fg='red')
        sys.exit(1)

    click.echo(f"Creating Gazebo package at '{package_path}' from '{from_pkg}'...")

    # 1. Create directories
    dirs_to_create = ["launch", "config", "worlds", "models", "urdf", "plugins", "scripts"]
    for d in dirs_to_create:
        os.makedirs(os.path.join(package_path, d))
    
    # 2. Create files from templates
    robot_name = package_name.replace('_gazebo', '')
    template_context = {
        'package_name': package_name,
        'source_robot_package': from_pkg,
        'robot_name': robot_name
    }

    files_to_create = {
        'CMakeLists.txt.j2': 'CMakeLists.txt',
        'package.xml.j2': 'package.xml',
        'launch/main.launch.py.j2': f'launch/{package_name}.launch.py',
        'launch/spawn.launch.py.j2': f'launch/spawn_{robot_name}.launch.py',
        'config/controllers.yaml.j2': 'config/controllers.yaml',
        'worlds/empty.world': 'worlds/empty.world'
    }

    for template, output_file in files_to_create.items():
        template_path = _get_template_path(template)
        output_path = os.path.join(package_path, output_file)
        
        with open(template_path, 'r') as f:
            template_content = f.read()
        
        if not template.endswith('.j2'):
            rendered_content = template_content
        else:
            rendered_content = _render_template(template_content, template_context)

        with open(output_path, 'w') as f:
            f.write(rendered_content)
        click.echo(f"  Created {output_path}")

    # 3. Symlink URDF
    source_urdf_rel = os.path.join('..', '..', 'src', from_pkg, 'urdf')
    target_urdf = os.path.join(package_path, 'urdf')
    
    os.rmdir(target_urdf)

    click.echo(f"Symlinking URDF from '{source_urdf_rel}' to '{target_urdf}'")
    try:
        os.symlink(source_urdf_rel, target_urdf, target_is_directory=True)
    except OSError as e:
        click.secho(f"Warning: Failed to create symlink. You may need to run as administrator on Windows.", fg='yellow')
        click.secho(f"  Error: {e}", fg='yellow')
        click.secho(f"  Please manually link '{os.path.abspath(source_urdf_rel)}' to '{os.path.abspath(target_urdf)}'.", fg='yellow')
        os.makedirs(target_urdf)
    except Exception as e:
        click.secho(f"An unexpected error occurred during symlinking: {e}", fg='red')
        os.makedirs(target_urdf)

    click.secho(f"\n✓ Package '{package_name}' created successfully.", fg='green')
    click.echo("Next steps: Run 'genesys build' to build the new package.")


@sim.command(name='run')
@click.argument('package_name')
@click.option('--world', default=None, help="Optional world file name (default: empty.world).")
@click.option('--headless', is_flag=True, help='Run Gazebo in headless mode.')
def run_sim(package_name, world, headless):
    """Runs a Gazebo simulation from a *_gazebo package."""

    if not os.path.isdir('install'):
        click.secho("Error: 'install' directory not found. Have you built the workspace yet?", fg="red")
        click.secho("Try running 'genesys build' first.", fg="yellow")
        sys.exit(1)

    package_path = os.path.join('sim', package_name)
    if not os.path.isdir(package_path):
        click.secho(f"Error: Simulation package '{package_name}' not found at '{package_path}'.", fg="red")
        click.secho("Try running 'genesys sim create ...' first.", fg='yellow')
        sys.exit(1)

    launch_file_name = f'{package_name}.launch.py'
    
    env = os.environ.copy()
    
    gazebo_model_path = os.path.abspath(os.path.join(package_path, 'models'))
    if 'GAZEBO_MODEL_PATH' in env:
        env['GAZEBO_MODEL_PATH'] = f"{env['GAZEBO_MODEL_PATH']}{os.pathsep}{gazebo_model_path}"
    else:
        env['GAZEBO_MODEL_PATH'] = gazebo_model_path
        
    gazebo_world_path = os.path.abspath(os.path.join(package_path, 'worlds'))
    if 'GAZEBO_RESOURCE_PATH' in env:
        env['GAZEBO_RESOURCE_PATH'] = f"{env['GAZEBO_RESOURCE_PATH']}{os.pathsep}{gazebo_world_path}"
    else:
        env['GAZEBO_RESOURCE_PATH'] = gazebo_world_path

    if world:
        env['GAZEBO_WORLD'] = world
    
    if headless:
        env['HEADLESS'] = '1'

    source_prefix, shell_exec = get_sourcing_command(clean_env=False)
    command_to_run = source_prefix + f"ros2 launch {package_name} {launch_file_name}"

    click.echo(f"Executing: {command_to_run}")
    if world:
        click.echo(f"  with world: {world}")
    if headless:
        click.echo("  in headless mode.")
        
    process = None
    try:
        process = subprocess.Popen(command_to_run, shell=True, executable=shell_exec, env=env)
        click.secho("\n✓ Simulation is starting...", fg="cyan")
        process.wait()
    except KeyboardInterrupt:
        click.echo("\nSimulation interrupted by user.")
        if process and process.poll() is None:
            process.terminate()
    except Exception as e:
        click.secho(f"An error occurred during simulation launch: {e}", fg="red")