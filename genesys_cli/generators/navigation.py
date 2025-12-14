import os
import shutil
import math
from pathlib import Path
from typing import List, Dict, Tuple
import jinja2

from genesys_cli.config.navigation_config import (
    NavigationConfig, DriveType
)

def compute_footprint(geometry, padding: float = 0.05) -> str:
    """
    Computes a rectangular footprint polygon based on geometry and padding.
    Returns a string representation compatible with Nav2 yaml (list of lists).
    """
    half_len = (geometry.length / 2.0) + padding
    half_width = (geometry.width / 2.0) + padding
    
    # Counter-clockwise from front-right (standard convention, though polygons can be any order if convex)
    # Front-Right, Front-Left, Back-Left, Back-Right
    points = [
        [round(half_len, 4), round(-half_width, 4)],     # Front-Right
        [round(half_len, 4), round(half_width, 4)],      # Front-Left
        [round(-half_len, 4), round(half_width, 4)],     # Back-Left
        [round(-half_len, 4), round(-half_width, 4)]     # Back-Right
    ]
    return str(points)

def select_plugins(drive_type: DriveType) -> Dict[str, str]:
    """
    Selects appropriate planner and controller plugins based on drive type.
    """
    plugins = {
        "planner": "nav2_smac_planner/SmacPlanner2D",
        "controller": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuit"
    }

    if drive_type == DriveType.OMNI:
        plugins["controller"] = "nav2_mppi_controller::MPPIController"
    elif drive_type == DriveType.ACKERMANN:
        plugins["planner"] = "nav2_smac_planner/SmacPlannerHybrid"
        plugins["controller"] = "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuit"
    # Differential/SkidStore defaults to Smac2D + RegulatedPurePursuit (or DWB, but RPP is preferred modern default)
    
    return plugins

def render_template(template_path: Path, output_path: Path, context: Dict):
    """
    Renders a Jinja2 template to the output path.
    """
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(template_path.parent)))
    template = env.get_template(template_path.name)
    content = template.render(context)
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(content)

def generate_navigation_package(config: NavigationConfig, output_root: Path):
    """
    Generates the full navigation package structure.
    """
    robot_name = config.robot_name
    
    # Define package definitions
    pkg_desc = output_root / f"{robot_name}_description"
    pkg_nav = output_root / f"{robot_name}_navigation"
    pkg_sim = output_root / f"{robot_name}_simulation"
    
    # Template Directory
    # Assuming genesys_cli is installed/running from source, find templates relative to this file
    # In a real install, use pkg_resources or proper path finding
    template_dir = Path(__file__).parent.parent / "commands" / "templates" / "navigation"
    
    # Context Preparation
    footprint = compute_footprint(config.geometry)
    plugins = select_plugins(config.drive_type)
    
    # Calculate robot radius for circular footprint fallback
    max_dim = max(config.geometry.length, config.geometry.width)
    robot_radius = round((max_dim / 2.0) + 0.1, 3)

    context = {
        "config": config,
        "footprint": footprint,
        "robot_radius": robot_radius,
        "planner_plugin": plugins["planner"],
        "controller_plugin": plugins["controller"],
        "use_sim_time": str(config.simulation).lower() # yaml scalar
    }

    # --- Navigation Package ---
    # 1. nav2_params.yaml
    render_template(
        template_dir / "params" / "nav2_params.yaml.j2",
        pkg_nav / "config" / "nav2_params.yaml",
        context
    )
    
    # 2. bringup.launch.py (To be created)
    if (template_dir / "launch" / "bringup.launch.py.j2").exists():
        render_template(
            template_dir / "launch" / "bringup.launch.py.j2",
            pkg_nav / "launch" / "bringup.launch.py",
            context
        )
        
    print(f"Generated navigation package at {pkg_nav}")
    
    # --- Description Package ---
    # 1. base.xacro
    render_template(
        template_dir / "urdf" / "base.xacro.j2",
        pkg_desc / "urdf" / "base.xacro",
        context
    )
    
    # Sensors
    if config.sensors.lidar_2d:
        if (template_dir / "urdf" / "sensors" / "lidar.xacro.j2").exists():
             render_template(
                template_dir / "urdf" / "sensors" / "lidar.xacro.j2",
                pkg_desc / "urdf" / "sensors" / "lidar.xacro",
                context
            )
            
    print(f"Generated description package at {pkg_desc}")

    # --- Simulation Package (if enabled) ---
    if config.simulation:
        if (template_dir / "launch" / "simulation.launch.py.j2").exists():
             render_template(
                template_dir / "launch" / "simulation.launch.py.j2",
                pkg_sim / "launch" / "simulation.launch.py",
                context
            )
        print(f"Generated simulation package at {pkg_sim}")
