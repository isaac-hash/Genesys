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
def generate():
    """Generate Nav2 scaffolding (Stub)."""
    click.echo("Generating Nav2 scaffolding... (Not implemented in Phase 1)")

@nav2.command()
@click.argument('section', type=click.Choice(['urdf', 'params', 'launch', 'sensors']))
def regen(section):
    """Regenerate specific Nav2 sections (Stub)."""
    click.echo(f"Regenerating {section}... (Not implemented in Phase 1)")
