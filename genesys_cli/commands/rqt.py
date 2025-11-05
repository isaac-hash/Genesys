import click
import subprocess

@click.group()
def rqt():
    """Launch RQT tools."""
    pass

@rqt.command("console")
def rqt_console():
    """Launch RQT Console."""
    click.echo("Launching RQT Console...")
    subprocess.Popen(["rqt_console"])

@rqt.command("graph")
def rqt_graph():
    """Launch RQT Graph."""
    click.echo("Launching RQT Graph...")
    subprocess.Popen(["rqt_graph"])
