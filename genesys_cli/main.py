import click

from genesys_cli.commands.build import build
from genesys_cli.commands.doctor import doctor
from genesys_cli.commands.launch import launch
from genesys_cli.commands.make import make
from genesys_cli.commands.new import new
from genesys_cli.commands.ros import node, topic, service, action, param
from genesys_cli.commands.rqt import rqt
from genesys_cli.commands.run import run
from genesys_cli.commands.sim import sim

@click.group()
def cli():
    """Genesys CLI for ROS 2 workspace management."""
    pass


# Add commands to the main CLI group
cli.add_command(build)
cli.add_command(doctor)
cli.add_command(launch)
cli.add_command(make)
cli.add_command(new)
cli.add_command(run)
cli.add_command(sim)

# Add command groups
cli.add_command(node)
cli.add_command(topic)
cli.add_command(service)
cli.add_command(action)
cli.add_command(param)
cli.add_command(rqt)

if __name__ == '__main__':
    cli()