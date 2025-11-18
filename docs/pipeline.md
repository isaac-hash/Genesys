# Pipelines in Genesys

This document describes how to use pipelines in the Genesys framework.

## What are pipelines?

Pipelines are a way to define a graph of connected ROS 2 nodes and components in a single YAML file. The Genesys CLI can then parse this file, generate a ROS 2 launch file in memory, and execute it. This is ideal for managing complex applications with many parts.

Pipelines support both regular nodes (executables) and composable nodes (components/plugins), along with parameters and topic remapping.

## Pipeline YAML Structure

A pipeline is defined by a top-level `pipeline` key.

```yaml
pipeline:
  name: my_vision_pipeline
  nodes:
    - id: camera_node
      package: image_publisher
      executable: image_publisher_node
      parameters:
        - name: frequency
          value: 30.0

    - id: image_processor
      package: my_package
      plugin: MyImageProcessor
      parameters:
        - name: brightness
          value: 1.2
      remap:
        /input_topic: /camera/image_raw
        /output_topic: /processed_image
```

### Top-Level Keys
- **`name`**: A descriptive name for your pipeline. This is used as the default namespace for all nodes in the pipeline.
- **`nodes`**: A list of node definitions.

### Node Definition Keys
- **`id`**: A unique identifier for the node within the pipeline.
- **`package`**: The name of the ROS 2 package containing the node.
- **`plugin`** (for Components): The name of the component plugin to load (e.g., the class name).
- **`executable`** (for Regular Nodes): The name of the node executable.
- **`name`** (optional): The ROS 2 node name. If omitted, the `id` is used.
- **`parameters`** (optional): A list of parameters to set on the node. Each parameter is a dictionary with `name` and `value`.
- **`remap`** (optional): A dictionary of topic remappings, where the key is the original topic name and the value is the new topic name.

## CLI Usage

The `genesys pipeline` command is used to interact with these files.

- **`genesys pipeline create <pipeline_name>`**: Creates a new boilerplate `pipeline_name.yaml` file.
- **`genesys pipeline run <pipeline.yaml>`**: Parses the YAML file and launches the defined graph of nodes.
- **`genesys pipeline watch <pipeline.yaml>`**: Runs the pipeline and automatically reloads it whenever the YAML file is saved, allowing for rapid iteration.

