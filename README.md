# Genesys: An Opinionated ROS 2 Framework

Genesys is a developer-friendly, opinionated framework for ROS 2 designed to reduce boilerplate, streamline common workflows, and provide a "happy path" for robotics development. It wraps the powerful but sometimes verbose ROS 2 toolchain in a single, intuitive CLI, allowing you to focus on logic, not setup.

## Core Philosophy

The goal of Genesys is not to replace ROS 2, but to enhance it. It addresses common pain points for both beginners and experienced developers:

-   **Complex Build Systems:** Automates package creation, dependency management, and the `colcon` build process.
-   **Verbose Boilerplate:** Uses decorators (Python) and macros (C++) to simplify node, publisher, and subscriber creation.
-   **Manual Configuration:** Auto-generates and registers launch files, configuration, and executables.
-   **Fragmented Tooling:** Provides a single, unified CLI (`genesys`) for creating, building, running, and simulating your projects.

**Key Principle:** Every Genesys project remains a 100% valid ROS 2 project. You can always fall back to `colcon build` and `ros2 run` at any time.

---

## Features

-   **Unified CLI:** A single entry point (`genesys`) for all your development tasks.
-   **Project Scaffolding:** Create a standardized workspace structure with `genesys new`.
-   **Interactive Code Generation:** Use `genesys make pkg` and `genesys make node` to interactively build packages and nodes with zero boilerplate, now including **interactive selection for node types** like **publishers, subscribers, services, actions, and lifecycle nodes**.
-   **Automated Build & Sourcing:** `genesys build` handles `colcon` and environment sourcing automatically.
-   **Simplified Execution:** Run nodes by name with `genesys run <node_name>` or launch entire packages with `genesys launch <pkg_name>`.
-   **One-Command Simulation:** Launch Gazebo with your world and robot model using `genesys sim <world_file>`.
-   **Decorator-Based API:** A clean, declarative way to define ROS 2 components in Python.
-   **Expanded Tooling:** Provides a suite of commands for common ROS 2 tasks, from listing nodes and topics to managing parameters and debugging.
-   **Environment Doctor:** A simple command (`genesys doctor`) to check if your environment is configured correctly, including a check for missing dependencies via `rosdep`.

---

## Installation

1.  **Prerequisites:**
    -   An installed ROS 2 distribution (e.g., Humble, Iron).
    -   The `ROS_DISTRO` environment variable must be set (e.g., `export ROS_DISTRO=humble`).

2.  **Install the CLI:**
    Clone this repository and run the following command from the project root (`Genesys/`):
    ```bash
    pip install -e .
    ```
    OR
    Install without cloning the repo
    ```bash
    pip install genesys-framework-cli
    ```
    
    This installs the `genesys` command 

    # Make the command available immediately
    export PATH="$(python3 -m site --user-base)/bin:$PATH"

    # (Optional) Add permanently so you don't repeat this step
    echo 'export PATH="$(python3 -m site --user-base)/bin:$PATH"' >> ~/.bashrc


3.  **Verify Installation:**
    Open a **new terminal** and run the environment checker:
    ```bash
    genesys doctor
    ```
    If all checks pass, you're ready to go!

---

## Quickstart: Your First Project

This workflow demonstrates the "happy path" for creating a new project from scratch.

1.  **Create a new workspace:**
    ```bash
    genesys new my_robot_ws
    cd my_robot_ws
    ```
    This creates a standard directory structure (`src/`, `launch/`, `config/`, etc.).

2.  **Create a package with a node:**
    The interactive wizard will guide you through the process, prompting you to select the node type (e.g., Publisher, Subscriber, etc.).
    ```bash
    genesys make pkg demo_pkg --with-node
    ```
    This generates `src/demo_pkg`, including `package.xml`, `setup.py`, a node file `demo_pkg/demo_pkg_node.py`, and auto-generates a corresponding launch file.

3.  **Build the project:**
    ```bash
    genesys build
    ```
    This runs `colcon build --symlink-install --cmake-clean-first` and sources the environment for you, with an optional '--persist' flag that ensures the sourcing is permanent so you can open a new terminal without sourcing again or running the build command again. The `demo_pkg_node` is now a runnable executable.

4.  **Run your node:**
    ```bash
    genesys run demo_pkg_node
    ```
    Genesys finds which package the node belongs to and executes `ros2 run demo_pkg demo_pkg_node` under the hood.

---

## Command Reference

### Workspace & Project Management
| Command | Description |
| ----------------------------------------- | -------------------------------------------------------------------------------------- |
| `genesys new <project_name>` | Creates a new, structured ROS 2 workspace. |
| `genesys make pkg <pkg_name>` | Interactively creates a new Python or C++ package in `src/`. |
| `genesys make node <node_name> --pkg <pkg>` | Creates a new node with interactive type selection and registers it within an existing package. |
| `genesys make interface` | Scaffolds custom message, service, and action files. |
| `genesys build` | Builds the entire workspace using `colcon` and sources the environment. |

### Execution & Simulation
| Command | Description |
| ----------------------------------------- | -------------------------------------------------------------------------------------- |
| `genesys run <node_name>` | Runs a node by its executable name without needing the package name. |
| `genesys launch <pkg>[:<file>]` | Launches a package's default launch file or a specific one. |
| `genesys launch --all` | Launches the `default.launch.py` from all packages in the workspace. |
| `genesys sim <world_file>` | Starts a Gazebo simulation with the specified world and a robot model. |

### ROS 2 Tooling
| Command | Description |
| ----------------------------------------- | -------------------------------------------------------------------------------------- |
| `genesys node list` | Lists all active nodes. (Equivalent to `ros2 node list`) |
| `genesys node info <node>` | Displays information about a specific node. (Equivalent to `ros2 node info`) |
| `genesys topic list` | Lists all active topics. |
| `genesys topic info <topic>` | Displays information about a topic. |
| `genesys topic echo <topic>` | Prints messages from a topic to the console. |
| `genesys topic pub <topic> <msg_type> <args>` | Publishes a message to a topic from the CLI. |
| `genesys service list` | Lists all active services. |
| `genesys service call <srv_name> <srv_type> <args>` | Calls a service from the CLI. |
| `genesys param list` | Lists parameters for a node. |
| `genesys param get <node> <param>` | Gets a parameter value from a node. |
| `genesys param set <node> <param> <value>` | Sets a parameter value on a node. |
| `genesys param dump` | Dumps all parameters from a node to a file. |
| `genesys debug record <topics>` | Records data from specified topics. |
| `genesys debug replay <file>` | Replays recorded data from a `.db3` file. |
| `genesys doctor` | Checks for common environment and configuration issues. |

---

## The Genesys Way: Decorators & Auto-generation

Genesys dramatically reduces boilerplate by using Python decorators to define ROS 2 constructs. When you create a node with `make node`, it comes pre-filled with a working example.

#### Example: A Simple Publisher Node

```python
from genesys.decorators import node, timer, publisher
from genesys.helpers import spin_node
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
````

When you run `genesys make node` or `genesys build`, the framework:

1.  **Scans** for these decorators.
2.  **Auto-registers** `my_talker_node` as an executable in `setup.py`.
3.  **Auto-generates/updates** a launch file (`launch/<pkg_name>_launch.py`) to include this node.

This means your node is ready to run immediately without manually editing any build or launch files.

#### Example: A Simple Subscriber Node

```python
from genesys.decorators import node, subscriber
from genesys.helpers import spin_node
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
```

### New Decorators

In addition to the ones above, Genesys introduces new decorators for advanced ROS 2 concepts, further reducing boilerplate:

  - `@lifecycle_node`: Defines a ROS 2 Lifecycle Node and handles the state callbacks (`on_configure`, `on_activate`, etc.).
  - `@service`: Registers a service server and automatically handles requests and responses.
  - `@action_server`: Registers an action server with support for feedback and result generation.
  - `@parameter`: Defines a node parameter, automatically loading values from a YAML config file and injecting them.

<!-- end list -->
