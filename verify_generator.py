
import sys
import os
from pathlib import Path
import yaml

# Add current dir to path to find genesys_cli
sys.path.append(os.getcwd())

from genesys_cli.config.navigation_config import NavigationConfig
from genesys_cli.generators.navigation import generate_navigation_package

def verify():
    config_path = Path('_verification_phase2/navigation.yaml')
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
        
    nav_config = NavigationConfig(**config_data)
    output_root = Path.cwd() / "_verification_output"
    output_root.mkdir(exist_ok=True)
    
    print(f"Generating for {nav_config.robot_name}...")
    generate_navigation_package(nav_config, output_root)
    
    # Assertions
    pkg_nav = output_root / f"{nav_config.robot_name}_navigation"
    pkg_desc = output_root / f"{nav_config.robot_name}_description"
    pkg_sim = output_root / f"{nav_config.robot_name}_simulation"
    
    assert pkg_nav.exists(), "Navigation package not created"
    assert (pkg_nav / "config" / "nav2_params.yaml").exists(), "nav2_params.yaml missing"
    assert (pkg_nav / "launch" / "bringup.launch.py").exists(), "bringup.launch.py missing"
    
    assert pkg_desc.exists(), "Description package not created"
    assert (pkg_desc / "urdf" / "base.xacro").exists(), "base.xacro missing"
    assert (pkg_desc / "urdf" / "sensors" / "lidar.xacro").exists(), "lidar.xacro missing"
    
    assert pkg_sim.exists(), "Simulation package not created"
    
    # Check content
    with open(pkg_nav / "config" / "nav2_params.yaml") as f:
        content = f.read()
        assert "footprint: \"[[" in content, "Footprint not computed/injected"
        assert "plugin: \"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuit\"" in content, "Wrong plugin"
        
    print("Verification Passed!")

if __name__ == "__main__":
    verify()
