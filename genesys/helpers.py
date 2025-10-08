import rclpy
import inspect
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.lifecycle import LifecycleNode

def spin_node(node_class, args=None):
    """
    A helper to initialize, spin, and shut down a Genesys-decorated node class.

    This function automatically handles both standard and lifecycle nodes,
    as well as graceful shutdown via KeyboardInterrupt.

    Args:
        node_class: The class decorated with @node.
        args: Command-line arguments to pass to rclpy.init().
    """
    if not inspect.isclass(node_class):
        raise TypeError("spin_node must be called with a class.")

    rclpy.init(args=args)
    node_instance = None
    try:
        node_instance = node_class()

        if isinstance(node_instance, LifecycleNode):
            # Lifecycle nodes require an executor to handle state transitions
            executor = MultiThreadedExecutor()
            executor.add_node(node_instance)
            executor.spin()
        else:
            # Standard nodes can be spun directly
            rclpy.spin(node_instance)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # finally:
    #     if node_instance and node_instance.context.is_valid():
    #         node_instance.destroy_node()
    #     if rclpy.ok():
    #         rclpy.shutdown()
    # finally:
    #     if node_instance is not None:
    #         node_instance.destroy_node()
    #     if rclpy.ok():
    #         rclpy.shutdown()
    finally:
        if node_instance is not None:
            try:
                node_instance.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
