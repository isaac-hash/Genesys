import click
import sys
import subprocess
import os
from genesys_cli.utils import get_sourcing_command, run_sourcing_command

@click.command(context_settings=dict(ignore_unknown_options=True))
@click.argument('node_name')
@click.argument('args', nargs=-1, type=click.UNPROCESSED)
def run(node_name, args):
    """
    Runs a ROS 2 node, automatically finding its package.

    Supports simplified remapping, e.g.:
    `genesys run <node_name> --remap <topic>:=<new_topic>`"""
    # 1. Verify we are in a workspace that has been built.
    if not os.path.isdir('install'):
        click.secho("Error: 'install' directory not found. Have you built the workspace yet?", fg="red")
        click.secho("Try running 'genesys build' first.", fg="yellow")
        sys.exit(1)

    click.echo(f"Attempting to run node: {node_name}")

    # 2. Get the sourcing command, which now includes the local install space.
    source_prefix, shell_exec = get_sourcing_command(clean_env=True)
    
    # 3. Find the package for the given node by listing all executables.
    list_exec_command = source_prefix + "ros2 pkg executables"
    try:
        result = subprocess.run(
            list_exec_command,
            check=True, capture_output=True, text=True, shell=True, executable=shell_exec
        )
    except subprocess.CalledProcessError as e:
        click.secho("Error: Failed to list ROS 2 executables.", fg="red")
        click.echo(e.stderr or e.stdout)
        sys.exit(1)

    # 4. Parse the output to find the package name.
    package_name = None
    available_nodes = []
    for line in result.stdout.strip().split('\n'):
        parts = line.split()
        if len(parts) < 2:
            continue
        pkg = parts[0]
        nodes = parts[1:]
        available_nodes.extend(nodes)
        if node_name in nodes:
            package_name = pkg
            break

    
    if not package_name:
        click.secho(f"Error: Node '{node_name}' not found in any package.", fg="red")
        click.echo("Please ensure you have built your workspace and the node name is correct.")
        if available_nodes:
            click.echo("\nAvailable nodes are:")
            for node in sorted(available_nodes):
                click.echo(f"  - {node}")
        sys.exit(1)

    click.echo(f"Found node '{node_name}' in package '{package_name}'. Starting node...")

    # 5. Construct and run the final command, processing remapping args.
    run_command_parts = ["run", package_name, node_name]
    
    ros_args_to_add = []
    other_args_to_add = []
    
    i = 0
    while i < len(args):
        arg = args[i]
        if arg == '--remap':
            if i + 1 < len(args):
                ros_args_to_add.extend(['-r', args[i+1]])
                i += 2
            else: # ignore dangling --remap
                i += 1
        elif arg.startswith('--remap='):
            ros_args_to_add.extend(['-r', arg.split('=', 1)[1]])
            i += 1
        else:
            other_args_to_add.append(arg)
            i += 1
            
    if ros_args_to_add:
        run_command_parts.append('--ros-args')
        run_command_parts.extend(ros_args_to_add)
        
    run_command_parts.extend(other_args_to_add)

    try:
        run_sourcing_command("ros2 " + " ".join(run_command_parts), interactive=True)
    except KeyboardInterrupt:
        click.echo("\nNode execution interrupted by user.") # Handled in helper
