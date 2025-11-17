# Components in Genesys

This document describes how to use components in the Genesys framework.

## What are components?

Components are reusable ROS 2 nodes that can be loaded into a container process. This is more efficient than running each node in a separate process, as it allows for intra-process communication and reduced overhead.

## Prerequisites

Before creating components, ensure you have a Genesys workspace set up. If you need to create a new ROS 2 package, you can do so using the `genesys make pkg` command:

```bash
genesys make pkg my_package --dependencies rclpy
```
Replace `my_package` with your desired package name and add any necessary ROS 2 dependencies.

## Creating a Component

To create a component node within an existing Genesys package, use the `genesys make component` command:

```bash
genesys make component MyComponentName --pkg my_package
```

-   `MyComponentName`: The desired name for your component (e.g., `image_processor`, `sensor_driver`). This will be used to generate the Python file (`my_component_name.py`) and the class name (`MyComponentName`).
-   `--pkg my_package`: The name of the existing Python package where the component should be created.

This command will:
1.  Generate a Python file (e.g., `my_package/my_package/my_component_name.py`) containing a basic component structure.
2.  The generated class will be decorated with `@component`, marking it as a Genesys component.
3.  Automatically add an `rclpy_components` entry point to your package's `setup.py`, allowing the component to be discovered by ROS 2.

## Building Your Workspace

After creating a new component or modifying existing code, you need to build your ROS 2 workspace:

```bash
colcon build --packages-select my_package
```
Replace `my_package` with the name of your package. If you want to build all packages in your workspace, you can omit `--packages-select`.

## Launching a Component

Genesys components are designed to be launched within a `ComposableNodeContainer`. You have two primary ways to launch your components:

### 1. Using a Mixed Launch File

You can create a custom Python launch file that includes a `ComposableNodeContainer` and your `ComposableNode`s.

Genesys can scaffold a mixed launch file for you:

```bash
genesys make launch --pkg my_package --name my_mixed_launch
```

Then, you can edit `src/my_package/launch/my_mixed_launch.py` to include your component:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define a namespace for your pipeline
    pipeline_namespace = LaunchConfiguration('my_pipeline_namespace', default='my_pipeline')

    my_component_node = ComposableNode(
        package='my_package',
        plugin='MyComponentName', # This should match the name used in genesys make component
        name='my_component_instance',
        namespace=pipeline_namespace,
        parameters=[
            {'example_param': 'value'}
        ],
        remappings=[
            ('/input_topic', '/my_pipeline/input_topic_remapped')
        ]
    )

    container = ComposableNodeContainer(
        name='my_component_container',
        namespace=pipeline_namespace,
        package='rclpy_components',
        executable='component_container',
        composable_node_descriptions=[my_component_node],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'my_pipeline_namespace',
            default_value='my_pipeline',
            description='Namespace for the component container and nodes.'
        ),
        container
    ])
```

To run this launch file:

```bash
ros2 launch my_package my_mixed_launch.py
```

### 2. Using the Genesys Pipeline System

For more complex component graphs and automatic wiring, you can define your components in a Genesys pipeline YAML file and use the `genesys pipeline` commands.

First, create a pipeline YAML file:

```bash
genesys pipeline create my_vision_pipeline
```

Edit the generated `my_vision_pipeline.yaml` to define your components. For example:

```yaml
pipeline:
  name: my_vision_pipeline
  nodes:
    - id: my_component_instance
      package: my_package
      plugin: MyComponentName # This should match the name used in genesys make component
      parameters:
        - name: example_param
          value: "another_value"
      remap:
        /input_topic: /my_vision_pipeline/input_topic_remapped
```

Then, run your pipeline:

```bash
genesys pipeline run my_vision_pipeline.yaml
```

You can also use `genesys pipeline watch my_vision_pipeline.yaml` for development, which will automatically reload your pipeline when the YAML file changes.

## Further Reading

-   `docs/pipeline.md`: Learn more about defining and managing Genesys pipelines.
-   `genesys_cli/commands/templates/python/component.py.j2`: Examine the template used for component generation.
-   `genesys/decorators.py`: Understand the `@component` decorator implementation.