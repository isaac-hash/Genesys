import click
from genesys_cli.utils import run_sourcing_command

@click.group()
def node():
    """Commands for interacting with ROS 2 nodes."""
    pass

@node.command("list")
def node_list():
    """List all running nodes."""
    run_sourcing_command("ros2 node list", interactive=False)

@node.command("info")
@click.argument("node_name")
def node_info(node_name):
    """Get information about a specific node."""
    run_sourcing_command(f"ros2 node info {node_name}", interactive=False)

@click.group()
def topic():
    """Commands for interacting with ROS 2 topics."""
    pass

@topic.command("list")
def topic_list():
    """List all active topics."""
    run_sourcing_command("ros2 topic list", interactive=False)

@topic.command("info")
@click.argument("topic_name")
def topic_info(topic_name):
    """Get information about a specific topic."""
    run_sourcing_command(f"ros2 topic info {topic_name}", interactive=False)

@topic.command("echo")
@click.argument("topic_name")
def topic_echo(topic_name):
    """Echo messages from a topic."""
    run_sourcing_command(f"ros2 topic echo {topic_name}", interactive=True)

@topic.command("pub", context_settings=dict(ignore_unknown_options=True))
@click.argument("topic_name")
@click.argument("msg_type")
@click.argument("args", nargs=-1, type=click.UNPROCESSED)
def topic_pub(topic_name, msg_type, args):
    """Publish a message to a topic."""
    command = f"ros2 topic pub {topic_name} {msg_type} {' '.join(args)}"
    run_sourcing_command(command, interactive=True)

@topic.command("bw")
@click.argument("topic_name")
def topic_bw(topic_name):
    """Display bandwidth used by a topic."""
    run_sourcing_command(f"ros2 topic bw {topic_name}", interactive=True)

@topic.command("find")
@click.argument("msg_type")
def topic_find(msg_type):
    """Find topics by message type."""
    run_sourcing_command(f"ros2 topic find {msg_type}", interactive=False)

@topic.command("record")
@click.argument("topics", nargs=-1)
def topic_record(topics):
    """Record topics to a bag file (ros2 bag record)."""
    cmd = "ros2 bag record"
    if not topics:
        click.echo("Recording all topics.")
        cmd += " -a"
    else:
        click.echo(f"Recording topics: {', '.join(topics)}")
        cmd += " " + " ".join(topics)
    run_sourcing_command(cmd, interactive=True)

@topic.command("replay")
@click.argument("bag_file")
def topic_replay(bag_file):
    """Play back a bag file (ros2 bag play)."""
    run_sourcing_command(f"ros2 bag play {bag_file}", interactive=True)

@click.group()
def service():
    """Commands for interacting with ROS 2 services."""
    pass

@service.command("list")
def service_list():
    """List all active services."""
    run_sourcing_command("ros2 service list", interactive=False)

@service.command("type")
@click.argument("srv_name")
def service_type(srv_name):
    """Get the type of a service."""
    run_sourcing_command(f"ros2 service type {srv_name}", interactive=False)

@service.command("info")
@click.argument("srv_name")
def service_info(srv_name):
    """Get information about a specific service (shows type)."""
    click.echo(f"Note: 'ros2 service info' does not exist. Showing type for '{srv_name}' instead.")
    run_sourcing_command(f"ros2 service type {srv_name}", interactive=False)

@service.command("find")
@click.argument("srv_type")
def service_find(srv_type):
    """Find services by service type."""
    run_sourcing_command(f"ros2 service find {srv_type}", interactive=False)

@service.command("call", context_settings=dict(ignore_unknown_options=True))
@click.argument("srv_name")
@click.argument("srv_type")
@click.argument("args", nargs=-1, type=click.UNPROCESSED)
def service_call(srv_name, srv_type, args):
    """Call a service with the given arguments."""
    command = f"ros2 service call {srv_name} {srv_type} {' '.join(args)}"
    run_sourcing_command(command, interactive=True)

@service.command("echo")
@click.argument("srv_name")
def service_echo(srv_name):
    """Echo service requests (Not a standard ROS 2 command)."""
    click.secho(f"Feature 'service echo' is not implemented. It requires a custom node to intercept and print requests.", err=True, fg="yellow")

@click.group()
def action():
    """Commands for interacting with ROS 2 actions."""
    pass

@action.command("list")
def action_list():
    """List all active actions."""
    run_sourcing_command("ros2 action list", interactive=False)

@action.command("type")
@click.argument("action_name")
def action_type(action_name):
    """Get the type of an action."""
    click.echo(f"Note: 'ros2 action type' does not exist. Use 'ros2 action list -t' to see all types.")
    run_sourcing_command(f"ros2 action info {action_name}", interactive=False)

@action.command("info")
@click.argument("action_name")
def action_info(action_name):
    """Get information about a specific action."""
    run_sourcing_command(f"ros2 action info {action_name}", interactive=False)

@action.command("send_goal", context_settings=dict(ignore_unknown_options=True))
@click.argument("action_name")
@click.argument("action_type")
@click.argument("args", nargs=-1, type=click.UNPROCESSED)
def action_send_goal(action_name, action_type, args):
    """Send a goal to an action server."""
    command = f"ros2 action send_goal {action_name} {action_type} {' '.join(args)}"
    run_sourcing_command(command, interactive=True)

@action.command("echo")
@click.argument("action_name")
def action_echo(action_name):
    """Echo action feedback and results (Not a standard ROS 2 command)."""
    click.secho(f"Feature 'action echo' is not implemented. It requires a custom node to monitor the action.", err=True, fg="yellow")

@click.group()
def param():
    """Commands for interacting with ROS 2 parameters."""
    pass

@param.command("list")
@click.argument("node_name", required=False)
def param_list(node_name):
    """List all parameters of all nodes, or one specific node."""
    command = "ros2 param list"
    if node_name:
        command += f" {node_name}"
    run_sourcing_command(command, interactive=False)

@param.command("get")
@click.argument("node_name")
@click.argument("param_name")
def param_get(node_name, param_name):
    """Get a parameter from a node."""
    run_sourcing_command(f"ros2 param get {node_name} {param_name}", interactive=False)

@param.command("set")
@click.argument("node_name")
@click.argument("param_name")
@click.argument("value")
def param_set(node_name, param_name, value):
    """Set a parameter on a node."""
    run_sourcing_command(f"ros2 param set {node_name} {param_name} {value}", interactive=True)

@param.command("dump")
@click.argument("node_name")
def param_dump(node_name):
    """Dump all parameters for a node to a file."""
    run_sourcing_command(f"ros2 param dump {node_name}", interactive=False)

@param.command("load")
@click.argument("node_name")
@click.argument("file_path")
def param_load(node_name, file_path):
    """Load parameters for a node from a file."""
    run_sourcing_command(f"ros2 param load {node_name} {file_path}", interactive=True)
