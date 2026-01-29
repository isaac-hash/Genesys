import click
import yaml
from pathlib import Path
from pydantic import ValidationError
import os

from genesys_cli.config.navigation_config import (
    NavigationConfig,
    DriveType,
    EnvironmentType,
    GeometryConfig,
    SensorConfig,
    OdometryConfig,
    NavParams,
    FrameDefaults
)

@click.group()
def nav2():
    """Manage Nav2 integration for Genesys robots."""
    pass

def prompt_for_config() -> NavigationConfig:
    click.echo("Configuring Nav2 integration...")
    
    robot_name = click.prompt("Robot Name (snake_case)", type=str)
    
    # Enum prompts
    # Drive Type Menu
    click.echo("Select Drive Type:")
    drive_type_choices = list(DriveType)
    for i, choice in enumerate(drive_type_choices, 1):
        click.echo(f"  {i}. {choice.value}")
    
    choice_idx = click.prompt(
        "Enter selection", 
        type=click.IntRange(1, len(drive_type_choices)), 
        default=1
    )
    drive_type = drive_type_choices[choice_idx - 1]
    
    simulation = click.confirm("Enable Simulation Support?", default=True)
    
    # Geometry
    click.echo("\nRobot Geometry (meters):")
    length = click.prompt("Length", type=float, default=0.35)
    width = click.prompt("Width", type=float, default=0.25)
    height = click.prompt("Height", type=float, default=0.12)
    wheel_base = click.prompt("Wheel Base", type=float, default=0.28)
    wheel_radius = click.prompt("Wheel Radius", type=float, default=0.08)
    geometry = GeometryConfig(
        length=length, width=width, height=height, 
        wheel_base=wheel_base, wheel_radius=wheel_radius
    )
    
    # Sensors
    click.echo("\nSensors:")
    lidar_2d = click.confirm("Enable 2D Lidar?", default=True)
    lidar_3d = click.confirm("Enable 3D Lidar?", default=False)
    depth_camera = click.confirm("Enable Depth Camera?", default=False)
    rgb_camera = click.confirm("Enable RGB Camera?", default=False)
    imu = click.confirm("Enable IMU?", default=True)
    sensors = SensorConfig(
        lidar_2d=lidar_2d, lidar_3d=lidar_3d, 
        depth_camera=depth_camera, rgb_camera=rgb_camera, imu=imu
    )
    
    # Odometry
    click.echo("\nOdometry:")
    fuse_imu = click.confirm("Fuse IMU data via robot_localization?", default=True)
    odometry = OdometryConfig(fuse_imu=fuse_imu)
    
    # Params
    click.echo("\nNavigation Parameters:")
    env_choices = [e.value for e in EnvironmentType]
    environment = click.prompt("Environment", type=click.Choice(env_choices), default=EnvironmentType.INDOOR.value)
    max_linear_vel = click.prompt("Max Linear Velocity (m/s)", type=float, default=0.5)
    max_angular_vel = click.prompt("Max Angular Velocity (rad/s)", type=float, default=1.0)
    params = NavParams(
        environment=environment, 
        max_linear_velocity=max_linear_vel, 
        max_angular_velocity=max_angular_vel
    )
    
    try:
        config = NavigationConfig(
            robot_name=robot_name,
            drive_type=drive_type,
            simulation=simulation,
            geometry=geometry,
            sensors=sensors,
            odometry=odometry,
            params=params
        )
        return config
    except ValidationError as e:
        click.echo(f"Configuration invalid: {e}")
        raise click.Abort()

@nav2.command()
@click.option('--output', '-o', default='config/navigation.yaml', help='Output configuration file path')
def init(output):
    """Initialize Nav2 configuration interactively."""
    config = prompt_for_config()
    
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        # Dump using JSON-compatible dict and then to YAML for better readability if needed
        # Or directly dump pydantic model to dict
        yaml.dump(config.model_dump(mode='json'), f, default_flow_style=False)
    
    click.echo(f"Navigation configuration saved to {output_path}")

@nav2.command()
@click.option('--config', default='config/navigation.yaml', help='Path to navigation configuration file')
def generate(config):
    """Generate Nav2 scaffolding based on configuration."""
    config_path = Path(config)
    if not config_path.exists():
        click.secho(f"Error: Configuration file not found at {config_path}", fg="red")
        return

    try:
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
            
        nav_config = NavigationConfig(**config_data)
        
        # Determine output root (workspace root, assumed to be CWD or parent of config)
        # We'll generate relative to CWD/src so packages are found by colcon
        output_root = Path.cwd() / "src"
        output_root.mkdir(parents=True, exist_ok=True)
        
        from genesys_cli.generators.navigation import generate_navigation_package
        generate_navigation_package(nav_config, output_root)
        
        click.secho("\nGeneration complete!", fg="green")
        click.echo("Next steps:")
        click.echo("  1. colcon build")
        click.echo("  2. source install/setup.bash")
        click.echo(f"  3. ros2 launch {nav_config.robot_name}_navigation bringup.launch.py")
        
        if nav_config.simulation:
             click.echo(f"  (For simulation: ros2 launch {nav_config.robot_name}_simulation simulation.launch.py)")

    except ValidationError as e:
        click.secho(f"Configuration is invalid: {e}", fg="red")
    except Exception as e:
        click.secho(f"Generation failed: {e}", fg="red")

@nav2.command()
@click.option('--watch', is_flag=True, help='Continuously monitor system status')
def doctor(watch):
    """Diagnose Nav2 environment health."""
    import time
    import subprocess
    
    def run_check(name, check_func, fix_suggestion):
        click.echo(f"Checking {name}...", nl=False)
        try:
            result = check_func()
            if result:
                click.secho("\t[PASS]", fg="green")
                return True
            else:
                click.secho("\t[FAIL]", fg="red")
                click.secho(f"  -> {fix_suggestion}", fg="yellow")
                return False
        except Exception as e:
            click.secho(f"\t[ERROR: {e}]", fg="red")
            return False

    def check_package(pkg_name):
        try:
            # Silence output
            with open(os.devnull, 'w') as devnull:
                subprocess.check_call(f"ros2 pkg prefix {pkg_name}", shell=True, stdout=devnull, stderr=devnull)
            return True
        except subprocess.CalledProcessError:
            return False

    def check_nav2_installed():
        return check_package("nav2_bringup")

    def check_tf_chain():
        # Fundamental: Check /tf topic
        try:
           output = subprocess.check_output("ros2 topic list", shell=True).decode()
           return "/tf" in output and "/tf_static" in output
        except:
           return False

    def check_topics():
        required = ["/scan", "/odom", "/tf", "/tf_static"]
        missing = []
        try:
           output = subprocess.check_output("ros2 topic list", shell=True).decode()
           for t in required:
               if t not in output:
                   missing.append(t)
        except:
           return False
        
        if missing:
            print(f" (Missing: {missing})", end="")
            return False
        return True

    def check_nodes():
        try:
            output = subprocess.check_output("ros2 node list", shell=True).decode()
            critical_nodes = ["controller_server", "planner_server", "bt_navigator", "recoveries_server"]
            found_count = 0
            for node in critical_nodes:
                if node in output:
                     found_count += 1
            
            return found_count >= len(critical_nodes)
        except:
            return False

    while True:
        click.clear()
        click.secho("=== Nav2 Doctor Diagnostics ===", bold=True)
        
        checks = [
            ("Nav2 Packages", check_nav2_installed, "Install Nav2: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"),
            ("Required Topics", check_topics, "Check if your sensors and odometry are publishing. Is Gazebo/Hardware running?"),
            ("TF Topics", check_tf_chain, "Check /tf and /tf_static. Ensure robot_state_publisher is running."),
            ("Nav2 Nodes", check_nodes, "Check if Nav2 stack is launched. Run 'genesys nav2 launch sim' or 'navigate'.")
        ]
        
        all_ok = True
        for name, func, fix in checks:
            if not run_check(name, func, fix):
                all_ok = False
        
        if all_ok:
            click.secho("\n[OK] System looks healthy!", fg="green")
        else:
            click.secho("\n[!] Issues detected.", fg="yellow")

        if not watch:
            break
        time.sleep(2)

@nav2.command()
@click.argument('mode', type=click.Choice(['sim', 'slam', 'navigate']))
@click.option('--config', default='config/navigation.yaml', help='Path to navigation configuration file')
@click.option('--map', 'map_file', default=None, help='Path to map.yaml for navigate mode')
def launch(mode, config, map_file):
    """Launch Nav2 in various modes (shortcuts)."""
    import subprocess

    config_path = Path(config)
    if not config_path.exists():
        click.secho(f"Error: Configuration file not found at {config_path}", fg="red")
        return

    try:
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
        nav_config = NavigationConfig(**config_data)
    except Exception as e:
        click.secho(f"Failed to load config: {e}", fg="red")
        return

    robot_name = nav_config.robot_name

    # Pre-flight Check: Ensure packages are found
    def check_package(pkg_name):
        try:
            with open(os.devnull, 'w') as devnull:
                subprocess.check_call(f"ros2 pkg prefix {pkg_name}", shell=True, stdout=devnull, stderr=devnull)
            return True
        except:
            return False

    required_pkgs = []
    if mode == 'sim':
        required_pkgs.append(f"{robot_name}_simulation")
    
    # All modes need navigation package or at least the description for transforms usually
    required_pkgs.append(f"{robot_name}_navigation") 
    required_pkgs.append("nav2_bringup")

    missing = []
    for pkg in required_pkgs:
        if not check_package(pkg):
            missing.append(pkg)
    
    if missing:
        click.secho("Error: The following required packages were not found:", fg="red", bold=True)
        for pkg in missing:
            click.secho(f"  - {pkg}", fg="red")
        
        click.echo("\nPossible fixes:")
        if "nav2_bringup" in missing:
             click.secho("  * Install Nav2: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup", fg="yellow")
        if any(robot_name in p for p in missing):
             click.secho("  * Source your workspace: source install/setup.bash", fg="yellow")
             click.secho("  * Ensure you have built the workspace: colcon build", fg="yellow")
        return

    cmd = []
    
    if mode == 'sim':
        click.secho(f"Launching Simulation for {robot_name}...", fg="cyan")
        # Unified launch file handles Gazebo, spawn, RSP, and Nav2 bringup
        cmd = f"ros2 launch {robot_name}_simulation simulation.launch.py"
        os.system(cmd)
        
    elif mode == 'slam':
        click.secho(f"Launching SLAM for {robot_name}...", fg="cyan")
        cmd = f"ros2 launch {robot_name}_navigation bringup.launch.py" 
        os.system(cmd)
        
    elif mode == 'navigate':
        click.secho(f"Launching Navigation for {robot_name}...", fg="cyan")
        if not map_file:
             map_file = click.prompt("Path to map.yaml", type=str)
        
        nav_cmd = f"ros2 launch nav2_bringup bringup_launch.py use_sim_time:={str(nav_config.simulation).lower()} map:={map_file} params_file:=$(ros2 pkg prefix {robot_name}_navigation)/share/{robot_name}_navigation/config/nav2_params.yaml"
        os.system(nav_cmd)

@nav2.command()
@click.argument('section', type=click.Choice(['urdf', 'params', 'launch', 'sensors']))
def regen(section):
    """Regenerate specific Nav2 sections (Stub)."""
    click.echo(f"Regenerating {section}... (Not implemented in Phase 1)")
