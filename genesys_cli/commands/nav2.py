import click
import yaml
from pathlib import Path
from pydantic import ValidationError

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
    length = click.prompt("Length", type=float)
    width = click.prompt("Width", type=float)
    height = click.prompt("Height", type=float)
    wheel_base = click.prompt("Wheel Base", type=float)
    wheel_radius = click.prompt("Wheel Radius", type=float)
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
        yaml.dump(config.dict(), f, default_flow_style=False)
    
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
        # We'll generate relative to CWD for now, or assume CWD is workspace root
        output_root = Path.cwd()
        
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

    def check_tf_chain():
        # Check map->odom->base_link
        # We can use tf2_echo or just check frames existence via tf2_tools view_frames (heavy)
        # Lighter: ros2 run tf2_ros tf2_echo map odom 
        # But running CLI commands captures output.
        # A quick check is `ros2 run tf2_ros tf2_echo` but that blocks.
        # Use `ros2 topic list` to ensure /tf is active?
        # True verification needs a node listener, but we are CLI.
        # We will parse `ros2 run tf2_tools view_frames` output or just check topics first.
        
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
        # Check if lifecycle nodes are active
        # ros2 lifecycle get <node>
        # Just check existence in `ros2 node list` first
        try:
            output = subprocess.check_output("ros2 node list", shell=True).decode()
            critical_nodes = ["controller_server", "planner_server", "bt_navigator", "recoveries_server"]
            # These might be namespaced (e.g. /my_bot/controller_server)
            # Simple check if any substring matches
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
    
    cmd = []
    
    if mode == 'sim':
        click.secho(f"Launching Simulation for {robot_name}...", fg="cyan")
        # 1. Launch Simulation (Gazebo)
        # Note: In a real CLI, we might need subprocess.Popen to run multiple independent things or use a composite launch file.
        # However, our 'simulation.launch.py' typically spawns gazebo.
        # Our 'bringup.launch.py' launches nav2 nodes.
        # Ideally, we want one command. 'genesys nav2 launch sim' implies launching EVERYTHING.
        # For simplicity, let's assume valid sourcing and just run the composite command or tell user we are running it.
        # ROS 2 way: create a temporary launch file or just chain? Chaining is hard.
        # BUT: In Phase 2, we made `simulation.launch.py` spawn the robot.
        # And `bringup.launch.py` brings up Nav2 + SLAM.
        # So we probably want to run simulation first, then bringup.
        # Running two launch files in parallel from python script is untidy without a composite launch.
        # Recommendation: Use a new composite launch or just instruct user?
        # Requirement says: "Shortcut".
        # We will use formatting to construct a command string that the USER can copy-paste if this fails,
        # or execute it if 'system' allows.
        
        # Actually, let's print the command to run. Running `ros2 launch` from python is tricky due to signal handling.
        # But we can try `os.system` or `subprocess.run` (blocking).
        # Since 'sim' is blocking, we can run a combined launch if it existed, OR run separate terminals?
        # No, standard 'ros2 launch' can include other launch files.
        # We don't have a "sim_and_nav.launch.py".
        # Let's run the simulation launch first in background? No.
        
        # WAIT: The implementation plan said: "Launches <robot>_simulation... then <robot>_navigation... "
        # We can construct a command that runs them together? No.
        # Best bet: Run the simulation one in a subprocess, then the navigation one? 
        # Actually, `ros2 launch` allows multiple files? No.
        
        # Let's guide the user to run two commands OR create a 'genesys_sim.launch.py' on the fly?
        # Simpler: Just print the exact command to run a combined setup if possible, or just the main one.
        # OR: Exec the primary bringup if it includes simulation?
        # Our `bringup.launch.py` includes `nav2_bringup` and `slam_toolbox`. It does NOT include gazebo.
        
        # User requirement: "Launch <robot>_simulation... then <robot>_navigation..."
        # If I am a CLI, I can spawn a subprocess for Gazebo and then run Nav2 in foreground.
        
        import subprocess
        
        # Setup commands
        sim_cmd = f"ros2 launch {robot_name}_simulation simulation.launch.py"
        nav_cmd = f"ros2 launch {robot_name}_navigation bringup.launch.py"
        
        click.secho("Starting Simulation (Gazebo)...", fg="green")
        sim_proc = subprocess.Popen(sim_cmd, shell=True)
        
        # Wait a bit
        import time
        time.sleep(5)
        
        click.secho("Starting Nav2 Bringup...", fg="green")
        # Replace current process with Nav2
        os.system(nav_cmd)
        
        # Note: If os.system returns, kill sim?
        sim_proc.terminate()
        
    elif mode == 'slam':
        click.secho(f"Launching SLAM for {robot_name}...", fg="cyan")
        # Assuming robot hardware is running or sim is already up?
        # Usually 'slam' implies mapping on a running robot.
        cmd = f"ros2 launch {robot_name}_navigation bringup.launch.py" 
        # (Our bringup currently has SLAM enabled by default in Phase 2 template)
        os.system(cmd)
        
    elif mode == 'navigate':
        click.secho(f"Launching Navigation for {robot_name}...", fg="cyan")
        if not map_file:
             map_file = click.prompt("Path to map.yaml", type=str)
        
        # We need a launch file that does Localization (AMCL) instead of SLAM.
        # Our generated `bringup.launch.py` currently defaults to SLAM via `online_async_launch.py`.
        # To support localization, we'd need to modify `bringup.launch.py` to accept 'slam:=False' and 'map:=<path>'.
        # For Phase 3 "Core Generation" we hardcoded SLAM.
        # For "Polish", we should probably update `bringup.launch.py` to handle this switch.
        # BUT, for now, we'll just run the Nav2 bringup with localization params if possible.
        # The standard `nav2_bringup/bringup_launch.py` takes `map`.
        
        # Since we are wrapping `nav2_bringup`, we can pass `map` and it might handle AMCL if we verify `bringup.launch.py`.
        # Taking a look at `bringup.launch.py.j2`... it calls `nav2_bringup/navigation_launch.py`.
        # Wait, `navigation_launch.py` is ONLY navigation (planner/controller/bt). It does NOT include AMCL/MapServer.
        # `bringup_launch.py` (standard) includes (localization + navigation).
        # Our template calls `navigation_launch.py` AND `slam_toolbox`.
        # Correct logic for 'navigate': Call `localization_launch.py` (AMCL+Map) + `navigation_launch.py`.
        
        # Since we might not have generated a dedicated navigation launch file, we'll try to rely on standard nav2 packages.
        
        nav_cmd = f"ros2 launch nav2_bringup bringup_launch.py use_sim_time:={str(nav_config.simulation).lower()} map:={map_file} params_file:=$(ros2 pkg prefix {robot_name}_navigation)/share/{robot_name}_navigation/config/nav2_params.yaml"
        os.system(nav_cmd)

@nav2.command()
@click.argument('section', type=click.Choice(['urdf', 'params', 'launch', 'sensors']))
def regen(section):
    """Regenerate specific Nav2 sections (Stub)."""
    click.echo(f"Regenerating {section}... (Not implemented in Phase 1)")
