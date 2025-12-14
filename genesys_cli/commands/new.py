import os
import click
import shutil

try:
    # Python 3.9+
    import importlib.resources as pkg_resources
except ImportError:
    # Python < 3.9
    import importlib_resources as pkg_resources

# --- Constants for paths ---
MACROS_DESTINATION_NAME = "genesys_macros"

@click.command()
@click.argument('project_name')
def new(project_name):
    """Creates a new ROS 2 workspace with the Genesys framework structure."""
    
    workspace_root = project_name
    click.echo(f"Creating new Genesys project at: ./{workspace_root}")

    if os.path.exists(workspace_root):
        click.secho(f"Error: Directory '{workspace_root}' already exists.", fg="red")
        return

    # Define the structure
    subdirs = [
        "src",
        "launch",
        "config",
        "sim",
        "tests",
        "scripts",
        "tools"
    ]

    try:
        # 1. Create the main workspace directory
        os.makedirs(workspace_root)
        
        # 2. Create standard subdirectories
        for subdir in subdirs:
            os.makedirs(os.path.join(workspace_root, subdir))

        # 3. Copy the custom macros directory using package resources 🚀
        try:
            with pkg_resources.path('genesys_cli', 'genesys_macros') as source_dir:
                destination_dir = os.path.join(workspace_root, 'src', MACROS_DESTINATION_NAME)
                
                if os.path.exists(source_dir):
                    shutil.copytree(source_dir, destination_dir)
                    click.secho(f"✓ Copied '{MACROS_DESTINATION_NAME}' package into 'src/'", fg="green")
                else: # This case should ideally not happen with correct packaging
                    click.secho(f"Warning: Packaged 'genesys_macros' not found at '{source_dir}'.", fg="yellow")
        except FileNotFoundError:
            click.secho("Warning: Could not find 'genesys_macros' package data. It may not be installed correctly.", fg="yellow")

        click.secho(f"✓ Project '{project_name}' created successfully.", fg="green")
        
        # --- Nav2 Integration ---
        if click.confirm("Enable autonomous navigation with Nav2?", default=False):
            from genesys_cli.config.navigation_config import (
                NavigationConfig, GeometryConfig, SensorConfig, OdometryConfig, NavParams
            )
            import yaml
            
            nav_config_path = os.path.join(workspace_root, 'config', 'navigation.yaml')
            
            # Create default config
            default_config = NavigationConfig(
                robot_name=project_name,
                drive_type="differential",
                simulation=True,
                geometry=GeometryConfig(
                    length=0.5, width=0.4, height=0.2, 
                    wheel_base=0.3, wheel_radius=0.1
                ),
                sensors=SensorConfig(),
                odometry=OdometryConfig(),
                params=NavParams()
            )
            
            with open(nav_config_path, 'w') as f:
                yaml.dump(default_config.dict(), f, default_flow_style=False)
            click.secho(f"✓ Created default Nav2 config at config/navigation.yaml", fg="green")
            
            if click.confirm("Do you want to configure navigation settings now?", default=True):
                from genesys_cli.commands.nav2 import prompt_for_config
                try:
                    user_config = prompt_for_config()
                    with open(nav_config_path, 'w') as f:
                        yaml.dump(user_config.dict(), f, default_flow_style=False)
                    click.secho(f"✓ Updated navigation config.", fg="green")
                except click.Abort:
                    click.secho("Configuration aborted. Keeping defaults.", fg="yellow")

        click.echo("Next steps: 'cd {}' and start creating packages!".format(project_name))
        
    except Exception as e:
        # Clean up in case of failure
        if os.path.exists(workspace_root):
            shutil.rmtree(workspace_root) 
        click.secho(f"Failed to create project: {e}", fg="red")