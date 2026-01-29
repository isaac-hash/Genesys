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
        "controller": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    }

    if drive_type == DriveType.OMNI:
        plugins["controller"] = "nav2_mppi_controller::MPPIController"
    elif drive_type == DriveType.ACKERMANN:
        plugins["planner"] = "nav2_smac_planner/SmacPlannerHybrid"
        plugins["controller"] = "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
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
    
    # 2. slam_params.yaml
    render_template(
        template_dir / "params" / "slam_params.yaml.j2",
        pkg_nav / "config" / "slam_params.yaml",
        context
    )
    
    # 2. bringup.launch.py
    if (template_dir / "launch" / "bringup.launch.py.j2").exists():
        render_template(
            template_dir / "launch" / "bringup.launch.py.j2",
            pkg_nav / "launch" / "bringup.launch.py",
            context
        )
    
    # 3. Scaffolding (package.xml, CMakeLists.txt)
    context_nav = context.copy()
    context_nav["package_suffix"] = "navigation"
    render_template(template_dir / "package_common" / "package.xml.j2", pkg_nav / "package.xml", context_nav)
    render_template(template_dir / "package_common" / "CMakeLists.txt.j2", pkg_nav / "CMakeLists.txt", context_nav)
        
    print(f"Generated navigation package at {pkg_nav}")
    
    # --- Description Package ---
    # 1. base.xacro
    render_template(
        template_dir / "urdf" / "base.xacro.j2",
        pkg_desc / "urdf" / "base.xacro",
        context
    )
    
    # 2. rsp.launch.py
    if (template_dir / "launch" / "rsp.launch.py.j2").exists():
        render_template(
            template_dir / "launch" / "rsp.launch.py.j2",
            pkg_desc / "launch" / "rsp.launch.py",
            context
        )
    
    # 3. Worlds
    worlds_dir = pkg_desc / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy world files
    gazebo_worlds_dir = template_dir.parent / "gazebo" / "worlds"
    if (gazebo_worlds_dir / "nav2_test.world").exists():
        shutil.copy(gazebo_worlds_dir / "nav2_test.world", worlds_dir / "nav2_test.world")
    
    if (gazebo_worlds_dir / "empty.world").exists():
        shutil.copy(gazebo_worlds_dir / "empty.world", worlds_dir / "empty.world")

    # Sensors - Generate sensor xacro files based on config
    sensors_dir = template_dir / "urdf" / "sensors"
    
    if config.sensors.lidar_2d:
        if (sensors_dir / "lidar.xacro.j2").exists():
            render_template(
                sensors_dir / "lidar.xacro.j2",
                pkg_desc / "urdf" / "sensors" / "lidar.xacro",
                context
            )
    
    if config.sensors.imu:
        if (sensors_dir / "imu.xacro.j2").exists():
            render_template(
                sensors_dir / "imu.xacro.j2",
                pkg_desc / "urdf" / "sensors" / "imu.xacro",
                context
            )
    
    if config.sensors.lidar_3d:
        if (sensors_dir / "lidar_3d.xacro.j2").exists():
            render_template(
                sensors_dir / "lidar_3d.xacro.j2",
                pkg_desc / "urdf" / "sensors" / "lidar_3d.xacro",
                context
            )
    
    if config.sensors.depth_camera:
        if (sensors_dir / "depth_camera.xacro.j2").exists():
            render_template(
                sensors_dir / "depth_camera.xacro.j2",
                pkg_desc / "urdf" / "sensors" / "depth_camera.xacro",
                context
            )
    
    if config.sensors.rgb_camera:
        if (sensors_dir / "rgb_camera.xacro.j2").exists():
            render_template(
                sensors_dir / "rgb_camera.xacro.j2",
                pkg_desc / "urdf" / "sensors" / "rgb_camera.xacro",
                context
            )
    
    # Scaffolding
    context_desc = context.copy()
    context_desc["package_suffix"] = "description"
    render_template(template_dir / "package_common" / "package.xml.j2", pkg_desc / "package.xml", context_desc)
    render_template(template_dir / "package_common" / "CMakeLists.txt.j2", pkg_desc / "CMakeLists.txt", context_desc)
            
    print(f"Generated description package at {pkg_desc}")

    # --- Simulation Package (if enabled) ---
    if config.simulation:
        if (template_dir / "launch" / "simulation.launch.py.j2").exists():
             render_template(
                template_dir / "launch" / "simulation.launch.py.j2",
                pkg_sim / "launch" / "simulation.launch.py",
                context
            )
        
        # RViz config
        if (template_dir / "rviz" / "navigation.rviz.j2").exists():
            render_template(
                template_dir / "rviz" / "navigation.rviz.j2",
                pkg_sim / "rviz" / "navigation.rviz",
                context
            )
             
        # Scaffolding
        context_sim = context.copy()
        context_sim["package_suffix"] = "simulation"
        render_template(template_dir / "package_common" / "package.xml.j2", pkg_sim / "package.xml", context_sim)
        render_template(template_dir / "package_common" / "CMakeLists.txt.j2", pkg_sim / "CMakeLists.txt", context_sim)
        
        print(f"Generated simulation package at {pkg_sim}")
