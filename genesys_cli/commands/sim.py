import os
import click
import sys
import subprocess
import shutil
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from genesys_cli.utils import get_sourcing_command

@click.group()
def sim():
    """Manages ROS 2 Gazebo simulations in the sim/ workspace."""
    pass

def _render_template(template_content, context):
    for key, value in context.items():
        placeholder = f'{{{{ {key} }}}}'
        template_content = template_content.replace(placeholder, str(value))
    return template_content

def _get_template_path(template_name):
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
    Path(sim_dir).mkdir(exist_ok=True)

    package_path = Path(sim_dir) / package_name
    if package_path.exists():
        click.secho(f"Error: Package '{package_name}' already exists.", fg='red')
        sys.exit(1)

    click.echo(f"Creating Gazebo package: {package_path}")

    # === 1. Create directories ===
    dirs = ["launch", "config", "worlds", "models", "urdf", "plugins", "scripts"]
    for d in dirs:
        (package_path / d).mkdir(parents=True)

    # === 2. Render templates ===
    robot_name = package_name.replace('_gazebo', '')
    context = {
        'package_name': package_name,
        'source_robot_package': from_pkg,
        'robot_name': robot_name
    }

    templates = {
        'CMakeLists.txt.j2': 'CMakeLists.txt',
        'package.xml.j2': 'package.xml',
        'launch/main.launch.py.j2': f'launch/{package_name}.launch.py',
        'launch/spawn.launch.py.j2': f'launch/spawn_{robot_name}.launch.py',
        'config/controllers.yaml.j2': 'config/controllers.yaml',
        'worlds/empty.world': 'worlds/empty.world'
    }

    for tmpl, output in templates.items():
        tmpl_path = _get_template_path(tmpl)
        output_path = package_path / output
        with open(tmpl_path, 'r') as f:
            content = f.read()
        rendered = content if not tmpl.endswith('.j2') else _render_template(content, context)
        output_path.write_text(rendered)
        click.echo(f"  Created {output_path}")

    # === 3. Symlink URDF using ament_index ===
    try:
        source_urdf_path = Path(get_package_share_directory(from_pkg)) / 'urdf'
        if not source_urdf_path.exists():
            raise PackageNotFoundError(f"URDF not found in {from_pkg}")
    except PackageNotFoundError:
        click.secho(f"Error: Package '{from_pkg}' not found or not built.", fg='red')
        click.secho("Run 'genesys build' and try again.", fg='yellow')
        sys.exit(1)

    target_urdf = package_path / 'urdf'
    if target_urdf.exists():
        if target_urdf.is_symlink():
            target_urdf.unlink()
        else:
            shutil.rmtree(target_urdf)
    
    relative_source = os.path.relpath(source_urdf_path, package_path)
    click.echo(f"Symlinking urdf → {relative_source}")
    try:
        os.symlink(relative_source, target_urdf)  # Standard symlink
    except OSError as e:
        click.secho(f"Warning: Symlink failed (may need admin): {e}", fg='yellow')
        click.secho("  Falling back to copy...", fg='yellow')
        shutil.copytree(source_urdf_path, target_urdf)

    click.secho(f"\nPackage '{package_name}' created successfully!", fg='green')
    click.echo("Next: Run 'genesys build' then 'genesys sim run {package_name}'")


@sim.command(name='run')
@click.argument('package_name')
@click.option('--world', default='empty.world', help="World file (in package's worlds/).")
@click.option('--headless', is_flag=True, help='Run without GUI.')
def run_sim(package_name, world, headless):
    """Runs a Gazebo simulation from a *_gazebo package."""

    if not Path('install').exists():
        click.secho("Error: Workspace not built.", fg='red')
        click.secho("Run: genesys build", fg='yellow')
        sys.exit(1)

    package_path = Path('sim') / package_name
    if not package_path.exists():
        click.secho(f"Error: Package '{package_name}' not found in sim/.", fg='red')
        sys.exit(1)

    launch_file = package_path / 'launch' / f'{package_name}.launch.py'
    if not launch_file.exists():
        click.secho(f"Error: Launch file not found: {launch_file}", fg='red')
        sys.exit(1)

    # === Environment ===
    env = os.environ.copy()
    models_path = str(package_path / 'models')
    worlds_path = str(package_path / 'worlds')

    # Gazebo Classic → GAZEBO_MODEL_PATH
    # Gazebo Sim (Ignition) → GZ_SIM_RESOURCE_PATH
    env['GAZEBO_MODEL_PATH'] = models_path
    env['GZ_SIM_RESOURCE_PATH'] = f"{worlds_path}{os.pathsep}{models_path}"

    if world != 'empty.world':
        env['GAZEBO_WORLD'] = world
    if headless:
        env['HEADLESS'] = '1'

    # === Launch ===
    source_cmd, shell = get_sourcing_command(clean_env=False)
    cmd = f"{source_cmd} ros2 launch {package_name} {launch_file.name}"

    click.echo(f"Launching: {package_name} with world='{world}'")
    if headless:
        click.echo("  (headless mode)")

    try:
        process = subprocess.Popen(
            cmd, shell=True, executable=shell, env=env,
            cwd=os.getcwd()
        )
        click.secho("\nSimulation running... (Ctrl+C to stop)", fg='cyan')
        process.wait()
    except KeyboardInterrupt:
        click.echo("\nShutting down...")
        process.terminate()
        process.wait()
    except Exception as e:
        click.secho(f"Launch failed: {e}", fg='red')
