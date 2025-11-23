# CLI Command: `genesys pipeline`

The `genesys pipeline` command group provides tools for managing and running declarative component graphs defined in YAML files. This allows you to define an entire multi-node system in one place.

See the main [Pipelines Documentation](./pipeline.md) for details on the YAML format.

---

## `genesys pipeline create`

Creates a new boilerplate pipeline YAML file.

### Usage
```bash
genesys pipeline create <pipeline_name>
```
- **`<pipeline_name>`**: The desired name for your pipeline.

### Description
This command creates a new file named `<pipeline_name>.yaml` in the current directory with a basic structure, ready for you to add nodes.

**Example:**
```bash
genesys pipeline create vision_system
```
This creates `vision_system.yaml` with the following content:
```yaml
pipeline:
  name: vision_system
  nodes: []
```

---

## `genesys pipeline run`

Parses a pipeline YAML file and launches the defined graph of nodes and components.

### Usage
```bash
genesys pipeline run <path_to_pipeline.yaml>
```

### How It Works
The `run` command provides a more direct way to launch a system compared to `genesys launch`.
1.  **Parse**: It reads and validates the pipeline YAML file.
2.  **Generate**: It converts the YAML definition into a ROS 2 `LaunchDescription` object **in memory**.
3.  **Execute**: Instead of calling `ros2 launch` as a subprocess, it uses the `launch.LaunchService` API from ROS 2 to run the generated launch description directly within the same process. This gives it more control, which is essential for the `watch` feature.

**Example:**
```bash
genesys pipeline run vision_system.yaml
```

---

## `genesys pipeline watch`

Runs a pipeline and automatically reloads it when the YAML file is changed. This provides a "hot-reload" capability for your system architecture.

### Usage
```bash
genesys pipeline watch <path_to_pipeline.yaml>
```

### How It Works
This is a powerful command for rapid development and iteration.
1.  **Initial Launch**: It starts by calling `genesys pipeline run` to launch the initial version of your pipeline.
2.  **File Monitoring**: It then uses the `watchdog` library to monitor your pipeline YAML file for any modifications.
3.  **Hot Reload**: When you save a change to the file, `watchdog` detects the event. The `watch` command then:
    a.  Gracefully **shuts down** the currently running `LaunchService`.
    b.  **Re-runs** the `run` command to parse the modified YAML and start a new `LaunchService` with the updated configuration.

This allows you to add/remove nodes, change parameters, or update topic remappings and see the changes take effect live within seconds, just by saving the file.

**Example:**
```bash
genesys pipeline watch vision_system.yaml
```
Now, every time you edit and save `vision_system.yaml`, the changes will be applied automatically.
