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
    OR
    Install without cloning the repo
    ```bash
    pip install genesys-framework-cli==0.1.0
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
    genesys make pkg demo_pkg --with-node
    ```
    This generates `src/demo_pkg`, including `package.xml`, `setup.py`, a node file `demo_pkg/demo_pkg_node.py`, and auto-generates a corresponding launch file.

3.  **Build--- a/c:\Users\HP\Documents\genesys\Genesys\README.md
+++ b/c:\Users\HP\Documents\genesys\Genesys\README.md
@@ -1,118 +1,154 @@
-# Genesys: An Opinionated ROS 2 Framework
+# Genesys: A Developer-Friendly Framework for ROS 2
 
-Genesys is a developer-friendly, opinionated framework for ROS 2 designed to reduce boilerplate, streamline common workflows, and provide a "happy path" for robotics development. It wraps the powerful but sometimes verbose ROS 2 toolchain in a single, intuitive CLI, allowing you to focus on logic, not setup.
+Genesys is an opinionated, decorator-based framework designed to streamline ROS 2 development in Python. It provides a powerful Command-Line Interface (CLI) and a set of intuitive decorators that reduce boilerplate and accelerate the creation of robust, readable, and maintainable robotics applications.
 
-## Core Philosophy
+## Core Concepts
 
-The goal of Genesys is not to replace ROS 2, but to enhance it. It addresses common pain points for both beginners and experienced developers:
+*   **Declarative Node Creation:** Use simple Python decorators (`@node`, `@publisher`, `@subscriber`, etc.) to define your node's entire communication architecture. The framework handles the complex `rclpy` setup for you.
+*   **Intelligent Scaffolding:** Quickly generate packages and nodes with sensible defaults and functional boilerplate for various communication patterns (Publisher, Subscriber, Service, Action, Lifecycle).
+*   **Simplified CLI Tooling:** A single `genesys` command provides a unified interface for workspace management, building, running nodes, and introspection, acting as a smart wrapper around `colcon` and `ros2`.
+*   **Opinionated Workspace:** A standardized project structure (`new` command) ensures consistency and makes it easy to navigate any Genesys project.
 
--   **Complex Build Systems:** Automates package creation, dependency management, and the `colcon` build process.
--   **Verbose Boilerplate:** Uses decorators (Python) and macros (C++) to simplify node, publisher, and subscriber creation.
--   **Manual Configuration:** Auto-generates and registers launch files, configuration, and executables.
--   **Fragmented Tooling:** Provides a single, unified CLI (`genesys`) for creating, building, running, and simulating your projects.
+---
 
 **Key Principle:** Every Genesys project remains a 100% valid ROS 2 project. You can always fall back to `colcon build` and `ros2 run` at any time.
 
-## Features
-
--   **Unified CLI:** A single entry point (`genesys`) for all your development tasks.
--   **Project Scaffolding:** Create a standardized workspace structure with `genesys new`.
--   **Interactive Code Generation:** Use `genesys make:pkg` and `genesys make:node` to interactively build packages and nodes with zero boilerplate.
--   **Automated Build & Sourcing:** `genesys build` handles `colcon` and environment sourcing automatically.
--   **Simplified Execution:** Run nodes by name with `genesys run <node_name>` or launch entire packages with `genesys launch <pkg_name>`.
--   **One-Command Simulation:** Launch Gazebo with your world and robot model using `genesys sim <world_file>`.
--   **Decorator-Based API:** A clean, declarative way to define ROS 2 components in Python.
--   **Environment Doctor:** A simple command (`genesys doctor`) to check if your environment is configured correctly.
-
 ## Installation
 
 1.  **Prerequisites:**
-    -   An installed ROS 2 distribution (e.g., Humble, Iron).
-    -   The `ROS_DISTRO` environment variable must be set (e.g., `export ROS_DISTRO=humble`).
+    -   Ensure you have a working ROS 2 installation (e.g., Humble, Iron) and have it sourced.
 
-2.  **Install the CLI:**
-    Clone this repository and run the following command from the project root (`Genesys/`):
+2.  **Install Genesys:**
+    Clone this repository and install it using `pip`. This makes the `genesys` command available in your terminal.
     ```bash
-    pip install -e .
-    ```
-    OR
-    Install without cloning the repo
-    ```bash
-    pip install genesys-framework-cli==0.1.0
+    git clone https://your-repo-url/genesys.git
+    cd Genesys
+    pip install .
     ```
-    
-    This installs the `genesys` command 
-
-    # Make the command available immediately
-    export PATH="$(python3 -m site --user-base)/bin:"
-
-    # (Optional) Add permanently so you don't repeat this step
-    echo 'export PATH="$(python3 -m site --user-base)/bin:"' >> ~/.bashrc
-
 
 3.  **Verify Installation:**
-    Open a **new terminal** and run the environment checker:
+    Check that the CLI is working and your environment is set up correctly.
     ```bash
     genesys doctor
     ```
-    If all checks pass, you're ready to go!
 
-## Quickstart: Your First Project
+---
 
-This workflow demonstrates the "happy path" for creating a new project from scratch.
+## Getting Started: A 5-Minute Tutorial
 
 1.  **Create a new workspace:**
     ```bash
-    genesys new my_robot_ws
-    cd my_robot_ws
+    genesys new my_robot_project
+    cd my_robot_project
     ```
-    This creates a standard directory structure (`src/`, `launch/`, `config/`, etc.).
 
-2.  **Create a package with a node:**
-    The interactive wizard will guide you through the process.
+2.  **Create a Python package:**
     ```bash
-    genesys make pkg demo_pkg --with-node
+    genesys make:pkg demo_pkg
     ```
-    This generates `src/demo_pkg`, including `package.xml`, `setup.py`, a node file `demo_pkg/demo_pkg_node.py`, and auto-generates a corresponding launch file.
 
-3.  **Build the project:**
+3.  **Scaffold a new node:**
+    ```bash
+    genesys make:node talker_node --pkg demo_pkg
+    ```
+    The CLI will prompt you to select a node type. Choose `Publisher`. This generates `src/demo_pkg/demo_pkg/talker_node.py` with functional, ready-to-run code.
+
+4.  **Build the workspace:**
     ```bash
     genesys build
     ```
-    This runs `colcon build --symlink-install` and sources the environment for you. The `demo_pkg_node` is now a runnable executable.
 
-4.  **Run your node:**
+5.  **Run your node:**
+    Open a new terminal, `cd` into your project, and run:
     ```bash
-    genesys run demo_pkg_node
+    # The `run` command automatically finds which package the node belongs to.
+    genesys run talker_node
     ```
-    Genesys finds which package the node belongs to and executes `ros2 run demo_pkg demo_pkg_node` under the hood.
 
-## Command Reference
+6.  **Inspect with the CLI:**
+    In another terminal, you can now use the Genesys CLI to see your node in action.
+    ```bash
+    # See the topic being published
+    genesys topic list
+
+    # Listen to the messages
+    genesys topic echo chatter
+    ```
+
+---
+
+## Declarative Node Development (Decorators)
+
+Genesys uses decorators to define ROS 2 entities.
 
 | Command | Description |
-| ----------------------------------------- | -------------------------------------------------------------------------------------- |
-| `genesys new <project_name>` | Creates a new, structured ROS 2 workspace. |
-| `genesys make:pkg <pkg_name>` | Interactively creates a new Python or C++ package in `src/`. |
-| `genesys make:node <node_name> --pkg <pkg>` | Creates a new node and registers it within an existing package. |
-| `genesys build` | Builds the entire workspace using `colcon` and sources the environment. |
-| `genesys run <node_name>` | Runs a node by its executable name without needing the package name. |
-| `genesys launch <pkg>[:<file>]` | Launches a package's default launch file or a specific one. |
-| `genesys launch --all` | Launches the `default.launch.py` from all packages in the workspace. |
-| `genesys sim <world_file>` | Starts a Gazebo simulation with the specified world and a robot model from `sim/models`. |
-| `genesys doctor` | Checks for common environment and configuration issues. |
+| :--- | :--- |
+| `@node("node_name")` | **(Required)** Class decorator that turns a Python class into a ROS Node. |
+| `@timer(period_sec)` | Method decorator. Runs the decorated method on a loop. |
+| `@publisher(topic, type)` | Method decorator. Publishes the method's return value. |
+| `@subscriber(topic, type)` | Method decorator. The method becomes the callback for a subscription. |
+| `@service(name, type)` | Method decorator. The method becomes a service callback. |
+| `@action_server(name, type)` | Method decorator for an `async` method to handle action goals. |
+| `@lifecycle_node` | Class decorator to explicitly mark a class as a Lifecycle Node. |
+| `@parameter("name", val)` | Class attribute. Declares a ROS parameter with a default value. |
+| `action_client(name, type)` | Class attribute. Injects a ready-to-use `ActionClient`. |
 
-## The Genesys Way: Decorators & Auto-generation
+### Example: A Multi-Function Node
 
-Genesys dramatically reduces boilerplate by using Python decorators to define ROS 2 constructs. When you create a node with `make:node`, it comes pre-filled with a working example.
+```python
+from framework_core.decorators import node, parameter, subscriber, service, timer
+from std_msgs.msg import String
+from example_interfaces.srv import AddTwoInts
 
-#### Example: A Simple Publisher Node
+@node("my_complex_node")
+class MyComplexNode:
+    # A configurable parameter, accessible via `self.prefix`
+    prefix: str = parameter("message_prefix", default_value="LOG")
 
-```python
-from framework_core.decorators import node, timer, publisher
-from framework_core.helpers import spin_node
-from std_msgs.msg import String
+    def __init__(self):
+        self.logger.info("Complex node is ready!")
 
-@node("my_talker_node")
-class MyTalker:
-    def __init__(self):
-        self.counter = 0
+    @subscriber(topic="input_topic", msg_type=String)
+    def input_callback(self, msg):
+        # self.prefix is dynamically updated if changed via `ros2 param set`
+        self.logger.info(f'[{self.prefix}] I heard: "{msg.data}"')
 
-    @timer(period_sec=1.0)
-    @publisher(topic="chatter", msg_type=String)
-    def publish_message(self):
-        """
-        This method runs every second. The String it returns is
-        automatically published to the 'chatter' topic.
-        """
-        msg = String()
-        msg.data = f"Hello from Genesys! Message #{self.counter}"
-        self.logger.info(f'Publishing: "{msg.data}"') # logger is auto-injected
-        self.counter += 1
-        return msg
+    @service(service_name="add_two_ints", service_type=AddTwoInts)
+    def add_callback(self, request, response):
+        response.sum = request.a + request.b
+        return response
+```
 
-def main(args=None):
-    spin_node(MyTalker, args)
+---
 
-if __name__ == '__main__':
-    main()
-```
+## Command-Line Interface (CLI) Reference
 
-When you run `genesys make:node` or `genesys build`, the framework:
+### Workspace & Build
 
-1.  **Scans** for these decorators.
-2.  **Auto-registers** `my_talker_node` as an executable in `setup.py`.
-3.  **Auto-generates/updates** a launch file (`launch/<pkg_name>_launch.py`) to include this node.
+| Command | Description |
+| :--- | :--- |
+| `genesys new <project_name>` | Creates a new, structured ROS 2 workspace. |
+| `genesys build` | Builds the entire workspace (`colcon build`). |
+| `genesys doctor` | Checks the environment for common configuration issues. |
 
-This means your node is ready to run immediately without manually editing any build or launch files.
+### Scaffolding (`make`)
 
-#### Example: A Simple Subscriber Node
+| Command | Description |
+| :--- | :--- |
+| `genesys make:pkg <pkg_name>` | Creates a new C++ or Python package in `src/`. |
+| `genesys make:node <node_name> --pkg <pkg>` | Creates a new node from a boilerplate template. |
+| `genesys make:interface <name> --pkg <pkg>` | (Placeholder) Guides creating custom interfaces. |
 
-```python
-from framework_core.decorators import node, subscriber
-from framework_core.helpers import spin_node
-from std_msgs.msg import String
+### Running & Launching
 
-@node("my_listener_node")
-class MyListener:
-    def __init__(self):
-        # The logger is automatically injected by the @node decorator.
-        self.logger.info("Listener node has been initialized.")
+| Command | Description |
+| :--- | :--- |
+| `genesys run <node_name>` | Finds and runs a node executable from any package. |
+| `genesys launch <pkg>[:<file.py>]` | Launches a specific launch file. |
+| `genesys launch --all` | Finds and launches `default.launch.py` in all packages. |
 
-    @subscriber(topic="chatter", msg_type=String)
-    def message_callback(self, msg):
-        """
-        This method is called whenever a message is received on the 'chatter' topic.
-        The message is automatically passed as an argument.
-        """
-        self.logger.info(f'I heard: "{msg.data}"')
+**Simplified Remapping:** Both `run` and `launch` support a cleaner syntax for remapping.
 
-def main(args=None):
-    spin_node(MyListener, args)
+```bash
+# Remap the 'chatter' topic to '/new_chatter'
+genesys run talker_node --remap chatter:=/new_chatter
+```
 
-if __name__ == '__main__':
-    main()
-```
+### Introspection & Debugging (ROS 2 Wrappers)
+
+Genesys provides direct, sourced wrappers for most common `ros2` commands.
+
+*   **`genesys node list | info <node_name>`**
+*   **`genesys topic list | info | echo | pub | bw | find`**
+*   **`genesys service list | type | info | find | call`**
+*   **`genesys action list | info | send_goal`**
+*   **`genesys param list | get | set | dump | load`**
+
+### Tooling
+
+| Command | Description |
+| :--- | :--- |
+| `genesys topic:record [topics...]` | Records specified topics to a bag file. Records all if none are given. |
+| `genesys topic:replay <bag_file>` | Plays back a bag file. |
+| `genesys rqt console` | Launches the RQT Console GUI. |
+| `genesys rqt graph` | Launches the RQT Graph GUI. |
+| `genesys sim <world_file>` | (Experimental) Launches Gazebo with a world and robot model. |

 the project:**
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
