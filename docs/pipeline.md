# Genesys Pipelines

This document provides a comprehensive guide to the Genesys Pipeline YAML format.

## What are Pipelines?

Pipelines are a way to define and configure a complete graph of connected ROS 2 nodes in a single YAML file. The `genesys pipeline` CLI can then parse this file, generate a ROS 2 launch description in memory, and execute it. This is the ideal way to manage and run complex applications with many parts.

## How It Works: In-Memory Launch

When you run `genesys pipeline run` or `watch`, the tool does not create a physical `.launch.py` file. Instead, it:
1.  **Parses** the YAML file into a Python dictionary.
2.  **Generates** a `LaunchDescription` object in memory based on your definitions.
3.  **Automatically creates** a `ComposableNodeContainer` if any components (`plugin` nodes) are defined.
4.  **Automatically namespaces** all nodes using the pipeline's `name`.
5.  **Executes** this `LaunchDescription` directly using the `launch.LaunchService` API.

---

## Pipeline YAML Structure

A pipeline is defined by a top-level `pipeline` key.

```yaml
#
# Example: A simple vision pipeline
#
pipeline:
  name: vision_system # This will be the ROS namespace for all nodes
  nodes:
    # A component (plugin) that runs in the auto-generated container
    - id: camera_producer
      package: image_publisher
      plugin: ImagePublisherNode
      name: camera_node # Overrides the ROS node name
      parameters:
        - name: frequency
          value: 30.0
    
    # A regular executable node
    - id: image_processor
      package: my_image_pkg
      executable: processor_node
      remap:
        # Manually remap the input topic to the camera's output
        /input_image: /vision_system/camera_node/image_raw

    # Another component
    - id: view_finder
      package: my_ui_pkg
      plugin: ImageViewNode
      remap:
        /input_image: /vision_system/image_processor/processed_image
```

### Top-Level Keys

- **`name`** (required, string): A descriptive name for your pipeline. This value is automatically used as the ROS 2 **namespace for all nodes** in the pipeline. In the example above, all nodes will be launched under the `/vision_system` namespace.
- **`nodes`** (required, list): A list of node definitions that make up the graph.

### Node Definition Keys

Each entry in the `nodes` list is an object that defines one node.

| Key | Required? | Type | Description |
| :--- | :--- | :--- | :--- |
| `id` | **Yes** | string | A unique identifier for the node within the pipeline. This is used for internal tracking. |
| `package` | **Yes** | string | The name of the ROS 2 package containing the node. |
| `plugin` | **Conditional** | string | The name of the component plugin to load (e.g., the class name). Use this for **components**. You must specify either `plugin` or `executable`. |
| `executable` | **Conditional** | string | The name of the node executable. Use this for **regular, standalone nodes**. You must specify either `plugin` or `executable`. |
| `name` | No | string | The final ROS 2 node name. If omitted, the node's `id` is used as its name. |
| `parameters` | No | list | A list of parameters to set on the node. Each item in the list is an object with `name` (string) and `value` keys. The value can be a string, number, or boolean. |
| `remap` | No | object | A dictionary of topic, service, or action remappings. The key is the original name and the value is the new name. |

---

## Important Concepts

### Component Container

You do **not** need to define a container in your pipeline YAML. If the generator detects one or more nodes with the `plugin` key, it will **automatically create a single `ComposableNodeContainer`** for the entire pipeline. All nodes defined with `plugin` will be loaded into this container. The container will be named `<pipeline_name>_container`.

Regular nodes defined with `executable` will be launched as separate processes, outside the container.

### Namespacing

All nodes, and the component container itself, are automatically namespaced with the top-level `name` of the pipeline. This is a powerful feature for isolating systems and avoiding topic name collisions.

### A Note on `publish` and `subscribe`

Some early examples or schemas may show `publish` and `subscribe` keys used for automatically connecting topics between nodes based on their `id`.

**This feature is aspirational and is NOT currently implemented in the pipeline generator.**

The generator will ignore these keys. To connect nodes, you **must** use the `remap` key to manually specify the topic connections, as shown in the example above. You should include the pipeline namespace in your remapping paths.
