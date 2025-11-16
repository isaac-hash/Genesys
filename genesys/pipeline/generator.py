from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import click # Added for warning messages

def generate_launch_description_from_pipeline(pipeline_data):
    """
    Generates a launch description from a pipeline dictionary, supporting
    both regular nodes and composable nodes, with remapping and parameters.
    """
    launch_description_entities = []
    composable_nodes = []
    regular_nodes = []

    pipeline_name = pipeline_data['pipeline']['name']

    # Define a launch argument for namespace, useful for remapping
    launch_description_entities.append(
        DeclareLaunchArgument(
            'namespace',
            default_value=pipeline_name,
            description='Namespace for the pipeline nodes.'
        )
    )
    pipeline_namespace = LaunchConfiguration('namespace')

    for node_config in pipeline_data['pipeline']['nodes']:
        node_id = node_config['id']
        package = node_config['package']
        name = node_config.get('name', node_id) # Use id as name if not specified

        # Handle remapping
        remappings = []
        if 'remap' in node_config:
            for old_topic, new_topic in node_config['remap'].items():
                remappings.append((old_topic, new_topic))
        
        # Handle parameters
        parameters = node_config.get('parameters', [])

        if 'plugin' in node_config:
            # This is a composable node
            plugin = node_config['plugin']
            composable_nodes.append(
                ComposableNode(
                    package=package,
                    plugin=plugin,
                    name=name,
                    namespace=pipeline_namespace, # Apply pipeline namespace
                    remappings=remappings,
                    parameters=parameters
                )
            )
        elif 'executable' in node_config:
            # This is a regular node
            executable = node_config['executable']
            regular_nodes.append(
                Node(
                    package=package,
                    executable=executable,
                    name=name,
                    namespace=pipeline_namespace, # Apply pipeline namespace
                    remappings=remappings,
                    parameters=parameters,
                    output='screen',
                    emulate_tty=True # Good for seeing logs in console
                )
            )
        else:
            click.secho(f"Warning: Node '{node_id}' in pipeline '{pipeline_name}' has neither 'plugin' nor 'executable'. Skipping.", fg="yellow")
            continue

    # Add regular nodes directly to the launch description
    launch_description_entities.extend(regular_nodes)

    # If there are composable nodes, create a container for them
    if composable_nodes:
        container = ComposableNodeContainer(
            name=f"{pipeline_name}_container",
            namespace=pipeline_namespace, # Apply pipeline namespace
            package='rclpy_components',
            executable='component_container',
            composable_node_descriptions=composable_nodes,
            output='screen',
        )
        launch_description_entities.append(container)

    return LaunchDescription(launch_description_entities)