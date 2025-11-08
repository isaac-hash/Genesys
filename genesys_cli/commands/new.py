import os
import click
import shutil
from pathlib import Path

@click.command()
@click.argument('project_name')
def new(project_name):
    """Creates a new ROS 2 workspace with the Genesys framework structure."""
    
    workspace_root = project_name
    click.echo(f"Creating new Genesys project at: ./{workspace_root}")

    if os.path.exists(workspace_root):
        click.secho(f"Error: Directory '{workspace_root}' already exists.", fg="red")
        return

    # Define the structure from the "Workspace Structure" reference
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
        os.makedirs(workspace_root)
        for subdir in subdirs:
            os.makedirs(os.path.join(workspace_root, subdir))

        # Copy genesys package
        shutil.copytree(
            Path(__file__).parents[2] / "genesys",
            Path(workspace_root) / "src" / "genesys",
        )

        click.secho(f"✓ Project '{project_name}' created successfully.", fg="green")
        click.echo("Next steps: 'cd {}' and start creating packages!".format(project_name))
    except Exception as e:
        click.secho(f"Failed to create project: {e}", fg="red")
