# CLI Command: `genesys pipeline`

The `genesys pipeline` command group provides tools for managing and running declarative component graphs defined in YAML files.

See the main [Pipelines Documentation](./pipeline.md) for details on the YAML format.

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
This creates `vision_system.yaml`.

---

## `genesys pipeline run`

Parses a pipeline YAML file and launches the defined graph of nodes and components.

### Usage
```bash
genesys pipeline run <path_to_pipeline.yaml>
```

### Description
This command reads the YAML file, validates it, generates a ROS 2 launch description in memory, and executes it. This is the primary way to run a pipeline.

**Example:**
```bash
genesys pipeline run vision_system.yaml
```

---

## `genesys pipeline watch`

Runs a pipeline and automatically reloads it when the YAML file is changed.

### Usage
```bash
genesys pipeline watch <path_to_pipeline.yaml>
```

### Description
This is an extremely useful command for development. It starts the pipeline using `run`, but also watches the source YAML file for any modifications. When you save a change to the file, the command will automatically shut down the running pipeline and launch a new one with the updated configuration. This allows for very fast iteration on your system's architecture.

**Example:**
```bash
genesys pipeline watch vision_system.yaml
```
Now, every time you edit and save `vision_system.yaml`, the changes will be applied live.
