import os
import click
import sys
import sysconfig
from genesys_cli.utils import get_sourcing_command, run_sourcing_command

@click.command()
def doctor():
    """Checks the environment for potential issues and provides solutions."""
    click.secho("Running Genesys environment doctor...", fg="cyan", bold=True)
    all_ok = True

    # 1. Check if the user's script installation directory is on the PATH
    click.echo("\nChecking PATH configuration...")
    # Get the directory where pip installs scripts for the current python environment
    scripts_dir = sysconfig.get_path('scripts')

    # Check if this directory is in the system's PATH environment variable
    if scripts_dir not in os.environ.get('PATH', '').split(os.pathsep):
        all_ok = False
        click.secho("[X] PATH Issue Detected", fg="red")
        click.echo(f"  Your local scripts directory ('{scripts_dir}') is not on your system's PATH.")
        click.echo("  This can prevent you from running 'genesys' directly after installation.")

        # Provide platform-specific instructions
        if sys.platform.startswith('linux') or sys.platform == 'darwin':
            click.echo("\n  To fix this for your current session, run:")
            click.secho(f'  export PATH="{scripts_dir}:$PATH"', fg="yellow")
            click.echo("\n  To fix this permanently, copy and paste the following command:")
            # Detect shell to suggest the correct rc file (~/.bashrc, ~/.zshrc, etc.)
            shell = os.environ.get("SHELL", "")
            rc_file = ""
            if "zsh" in shell:
                rc_file = "~/.zshrc"
            elif "bash" in shell:
                rc_file = "~/.bashrc"
            else:
                # A safe fallback for other shells
                rc_file = "your shell's startup file (e.g., ~/.bashrc, ~/.zshrc)"

            click.secho(f"  echo 'export PATH=\"{scripts_dir}:$PATH\"' >> {rc_file}", fg="green")

            click.echo(f"  After running the command, please start a new terminal session for the change to take effect.")
    elif sys.platform == 'win32':
                click.echo("\n  To fix this, you need to add the following directory to your 'Path' environment variable:")
                click.secho(f"  {scripts_dir}", fg="yellow")
                click.echo("  You can do this through 'Edit the system environment variables' in the Control Panel.")
    else:
        click.secho("[✓] PATH configuration is correct.", fg="green")

    click.echo("\nChecking ROS 2 environment...")
    source_prefix, _ = get_sourcing_command(exit_on_error=False)
    if source_prefix is None:
        all_ok = False
        click.secho("[X] ROS 2 Environment Issue Detected", fg="red")
        click.echo("  The ROS_DISTRO environment variable is not set or the setup script is missing.")
        click.echo("  Please ensure a ROS 2 distribution is installed and the ROS_DISTRO variable is set.")
    else:
        click.secho("[✓] ROS 2 environment sourcing is configured.", fg="green")

    click.echo("\nChecking for missing dependencies (rosdep)...")
    if os.path.isdir('src'):
        rosdep_ok = run_sourcing_command("rosdep install --from-paths src -y --ignore-src", interactive=True, exit_on_error=False)
        if rosdep_ok:
            click.secho("[✓] rosdep check complete.", fg="green")
        else:
            all_ok = False
            click.secho("[X] rosdep check reported issues. Please check the output above.", fg="red")
    else:
        click.secho("[!] 'src' directory not found, skipping rosdep check.", fg="yellow")

    click.echo("-" * 40)
    if all_ok:
        click.secho("✨ Your Genesys environment is ready to go!", fg="cyan", bold=True)
    else:
        click.secho("Please address the issues above to ensure Genesys works correctly.", fg="yellow")
