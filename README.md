# Genesys: An Opinionated ROS 2 Framework

Genesys is a developer-friendly, opinionated framework for ROS 2 designed to reduce boilerplate, streamline common workflows, and provide a "happy path" for robotics development. It wraps the powerful but sometimes verbose ROS 2 toolchain in a single, intuitive CLI, allowing you to focus on logic, not setup.

## Core Philosophy

The goal of Genesys is not to replace ROS 2, but to enhance it. It addresses common pain points for both beginners and experienced developers:

-   **Complex Build Systems:** Automates package creation, dependency management, and the `colcon` build process.
-   **Verbose Boilerplate:** Uses decorators (Python) and macros (C++) to simplify node, publisher, and subscriber creation.
-   **Manual Configuration:** Auto-generates and registers launch files, configuration, and executables.
-   **Fragmented Tooling:** Provides a single, unified CLI (`genesys`) for creating, building, running, and simulating your projects.

**Key Principle:** Every Genesys project remains a 100% valid ROS 2 project. You can always fall back to `colcon build` and `ros2 run` at any time.

## Features

-   **Unified CLI:** A single entry point (`genesys`) for all your development tasks.
-   **Project Scaffolding:** Create a standardized workspace structure with `genesys new`.
-   **Interactive Code Generation:** Use `genesys make:pkg` and `genesys make:node` to interactively build packages and nodes with zero boilerplate.
-   **Automated Build & Sourcing:** `genesys build` handles `colcon` and environment sourcing automatically.
-   **Simplified Execution:** Run nodes by name with `genesys run <node_name>` or launch entire packages with `genesys launch <pkg_name>`.
-   **One-Command Simulation:** Launch Gazebo with your world and robot model using `genesys sim <world_file>`.
-   **Decorator-Based API:** A clean, declarative way to define ROS 2 components in Python.
-   **Environment Doctor:** A simple command (`genesys doctor`) to check if your environment is configured correctly.

## Installation

1.  **Prerequisites:**
    -   An installed ROS 2 distribution (e.g., Humble, Iron).
    -   The `ROS_DISTRO` environment variable must be set (e.g., `export ROS_DISTRO=humble`).

2.  **Install the CLI:**
    Clone this repository and run the following command from the project root (`Genesys/`):
    ```bash
    pip install -e .
    ```
    This installs the `genesys` command in "editable" mode, so any changes you make to the source code are immediately reflected.

3.  **Verify Installation:**
    Open a **new terminal** and run the environment checker:
    ```bash
    genesys doctor
    ```
    If all checks pass, you're ready to go!

## Quickstart: Your First Project

This workflow demonstrates the "happy path" for creating a new project from scratch.

1.  **Create a new workspace:**
    ```bash
    genesys new my_robot_ws
    cd my_robot_ws
    ```
    This creates a standard directory structure (`src/`, `launch/`, `config/`, etc.).

2.  **Create a package with a node:**
    The interactive wizard will guide you through the process.
    ```bash
    genesys make:pkg demo_pkg --with-node
    ```
    This generates `src/demo_pkg`, including `package.xml`, `setup.py`, a node file `demo_pkg/demo_pkg_node.py`, and auto-generates a corresponding launch file.

3.  **Build the project:**
    ```bash
    genesys build
    ```
    This runs `colcon build --symlink-install` and sources the environment for you. The `demo_pkg_node` is now a runnable executable.

4.  **Run your node:**
    ```bash
    genesys run demo_pkg_node
    ```
    Genesys finds which package the node belongs to and executes `ros2 run demo_pkg demo_pkg_node` under the hood.

## Command Reference

| Command | Description |
| ----------------------------------------- | -------------------------------------------------------------------------------------- |
| `genesys new <project_name>` | Creates a new, structured ROS 2 workspace. |
| `genesys make:pkg <pkg_name>` | Interactively creates a new Python or C++ package in `src/`. |
| `genesys make:node <node_name> --pkg <pkg>` | Creates a new node and registers it within an existing package. |
| `genesys build` | Builds the entire workspace using `colcon` and sources the environment. |
| `genesys run <node_name>` | Runs a node by its executable name without needing the package name. |
| `genesys launch <pkg>[:<file>]` | Launches a package's default launch file or a specific one. |
| `genesys launch --all` | Launches the `default.launch.py` from all packages in the workspace. |
| `genesys sim <world_file>` | Starts a Gazebo simulation with the specified world and a robot model from `sim/models`. |
| `genesys doctor` | Checks for common environment and configuration issues. |

## The Genesys Way: Decorators & Auto-generation

Genesys dramatically reduces boilerplate by using Python decorators to define ROS 2 constructs. When you create a node with `make:node`, it comes pre-filled with a working example.

#### Example: A Simple Publisher Node

```python
from framework_core.decorators import node, timer, publisher
from framework_core.helpers import spin_node
from std_msgs.msg import String

@node("my_talker_node")
class MyTalker:
    def __init__(self):
        self.counter = 0

    @timer(period_sec=1.0)
    @publisher(topic="chatter", msg_type=String)
    def publish_message(self):
        """
        This method runs every second. The String it returns is
        automatically published to the 'chatter' topic.
        """
        msg = String()
        msg.data = f"Hello from Genesys! Message #{self.counter}"
        self.logger.info(f'Publishing: "{msg.data}"') # logger is auto-injected
        self.counter += 1
        return msg

def main(args=None):
    spin_node(MyTalker, args)

if __name__ == '__main__':
    main()
```

When you run `genesys make:node` or `genesys build`, the framework:

1.  **Scans** for these decorators.
2.  **Auto-registers** `my_talker_node` as an executable in `setup.py`.
3.  **Auto-generates/updates** a launch file (`launch/<pkg_name>_launch.py`) to include this node.

This means your node is ready to run immediately without manually editing any build or launch files.

#### Example: A Simple Subscriber Node

```python
from framework_core.decorators import node, subscriber
from framework_core.helpers import spin_node
from std_msgs.msg import String

@node("my_listener_node")
class MyListener:
    def __init__(self):
        # The logger is automatically injected by the @node decorator.
        self.logger.info("Listener node has been initialized.")

    @subscriber(topic="chatter", msg_type=String)
    def message_callback(self, msg):
        """
        This method is called whenever a message is received on the 'chatter' topic.
        The message is automatically passed as an argument.
        """
        self.logger.info(f'I heard: "{msg.data}"')

def main(args=None):
    spin_node(MyListener, args)

if __name__ == '__main__':
    main()
