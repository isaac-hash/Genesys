import os
import click
import subprocess
import sys
from genesys_cli.utils import get_sourcing_command
from genesys_cli.scaffolding import persist_workspace_sourcing

@click.command()
@click.option('--packages', '-p', multiple=True, help='Specific packages to build. Builds all if not specified.')
@click.option('--persist', is_flag=True, help='Add workspace sourcing to shell startup file (e.g., .bashrc).')
def build(packages, persist):
    """Builds the entire workspace or specific packages."""
    # 1. Verify we are in a Genesys workspace root.
    if not os.path.isdir('src'):
        click.secho("Error: This command must be run from the root of a Genesys workspace.", fg="red")
        click.secho("(A 'src' directory was not found.)", fg="yellow")
        sys.exit(1)

    click.echo("Building the workspace...")

    source_prefix, shell_exec = get_sourcing_command(clean_env=True)

    colcon_command = ['colcon', 'build', '--symlink-install', '--cmake-clean-first']
    if packages:
        colcon_command.extend(['--packages-select'] + list(packages))

    command_to_run = source_prefix + ' '.join(colcon_command)

    click.echo(f"Running build command...")

    try:
        # Use Popen to stream output in real-time, which is better for build commands.
        process = subprocess.Popen(
            command_to_run,
            shell=True,
            executable=shell_exec,
            stdout=sys.stdout,
            stderr=sys.stderr
        )
        process.wait() # Wait for the build to finish
        
        if process.returncode == 0:
            click.secho("\n✓ Build completed successfully.", fg="green")
            click.echo("To use the new executables, you may need to source the workspace or start a new terminal.")
            if persist:
                persist_workspace_sourcing()
            else:
                click.echo("To use the new executables, you may need to source the workspace or start a new terminal.")
        else:
            raise subprocess.CalledProcessError(process.returncode, command_to_run)

    except subprocess.CalledProcessError as e:
        click.secho(f"\nBuild failed with exit code {e.returncode}.", fg="red")
        sys.exit(1)
