import functools
import inspect
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionServer, ActionClient
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


# --- Quality of Service Profiles ---
# Making QoS profiles more accessible as per guardrail improvements.
QOS_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

QOS_SENSOR_DATA = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

QOS_SYSTEM_DEFAULT = rclpy.qos.qos_profile_system_default
QOS_SERVICES_DEFAULT = rclpy.qos.qos_profile_services_default

# --- Helper Functions ---
def _import_type_from_string(type_string: str):
    """Dynamically imports a ROS message type from a string like 'std_msgs.msg.String'."""
    try:
        parts = type_string.split('.')
        module_name = '.'.join(parts[:-1])
        class_name = parts[-1]
        module = __import__(module_name, fromlist=[class_name])
        return getattr(module, class_name)
    except (ImportError, AttributeError, IndexError) as e:
        raise ImportError(f"Could not import message type '{type_string}': {e}")

# --- Placeholder objects ---
class Parameter:
    """
    A placeholder for a ROS 2 parameter that will be declared by the @node decorator.
    """
    def __init__(self, name, default_value=None):
        self.name = name
        self.default_value = default_value
        self.attr_name = None  # This will be set by __set_name__

    def __set_name__(self, owner, name):
        """
        Captures the attribute name this Parameter instance is assigned to.
        e.g., in `scan_topic = parameter(...)`, name is 'scan_topic'.
        """
        self.attr_name = name

class ActionClientPlaceholder:
    """
    A placeholder for a ROS 2 action client that will be created by the @node decorator.
    """
    def __init__(self, name, action_type):
        self.name = name
        self.action_type = action_type
        self.attr_name = None  # This will be set by __set_name__

    def __set_name__(self, owner, name):
        """Captures the attribute name this instance is assigned to."""
        self.attr_name = name

def action_client(name, action_type):
    """Defines a ROS 2 action client as a class attribute."""
    # This is not a decorator, but a function that returns a placeholder
    return ActionClientPlaceholder(name, action_type)

def parameter(name, default_value=None):
    """
    Decorator-like function to define a ROS 2 parameter as a class attribute.

    Args:
        name (str): The ROS 2 parameter name.
        default_value: The default value of the parameter.

    Returns:
        Parameter: A placeholder object processed by the @node decorator.
    """
    return Parameter(name, default_value)


# --- Metadata decorators ---
def publisher(topic, msg_type, qos: QoSProfile = QOS_DEFAULT):
    """
    Decorator to mark a method as a source for a ROS 2 publisher.

    The decorated method's return value will be published.
    Stack this decorator to publish to multiple topics from one method.
    """
    def decorator(func):
        if not hasattr(func, '_ros_publishers'):
            func._ros_publishers = []
        # The 'topic' can be a string literal or a Parameter object.
        func._ros_publishers.append({'topic': topic, 'msg_type': msg_type, 'qos': qos})
        return func
    return decorator


def subscriber(topic, msg_type, qos: QoSProfile = QOS_DEFAULT, debug_log: bool = False):
    """
    Decorator to register a method as a ROS 2 subscriber callback.

    Stack this decorator to subscribe to multiple topics with the same callback.
    """
    def decorator(func):
        if not hasattr(func, '_ros_subscribers'):
            func._ros_subscribers = []
        func._ros_subscribers.append({
            'topic': topic,
            'msg_type': msg_type,
            'qos': qos,
            'debug_log': debug_log
        })
        return func
    return decorator


def timer(period_sec):
    """Decorator to run a method on a recurring timer."""
    def decorator(func):
        if not hasattr(func, '_ros_timers'):
            func._ros_timers = []
        func._ros_timers.append({'period': period_sec})
        return func
    return decorator

def service(service_name, service_type):
    """
    Decorator to register a method as a ROS 2 service server callback.
    """
    def decorator(func):
        if not hasattr(func, '_ros_services'):
            func._ros_services = []
        func._ros_services.append({'name': service_name, 'type': service_type})
        return func
    return decorator


def action_server(action_name, action_type):
    """
    Decorator to register a method as a ROS 2 action server execute callback.
    The decorated method must be a coroutine (async def).
    """
    def decorator(func):
        if not hasattr(func, '_ros_action_servers'):
            func._ros_action_servers = []
        func._ros_action_servers.append({'name': action_name, 'type': action_type})
        return func
    return decorator

def lifecycle_node(cls):
    """
    Class decorator to explicitly mark a class as a Lifecycle Node.
    This is an alternative to the automatic detection based on lifecycle hooks.
    """
    cls._is_lifecycle_node = True
    return cls

# --- The main orchestrator decorator ---
def node(node_name):
    """
    Transforms a plain Python class into a complete ROS 2 Node.
    If the user class defines lifecycle hooks (e.g., on_configure), it is
    automatically promoted to a LifecycleNode.
    """
    def decorator(user_cls):

        # This mixin contains all the shared logic for both node types
        class GenesysNodeLogicMixin:
            def _init_logic(self, user_cls_arg):
                # This is called from the wrapper's __init__
                # Create an instance of the user's class without calling __init__
                self.user_instance = user_cls_arg.__new__(user_cls_arg)

                # Inject the logger before calling the user's constructor
                self.user_instance.logger = self.get_logger()

                # Inject other common Node methods
                self.user_instance.get_clock = self.get_clock

                # Now, call the user's constructor
                self.user_instance.__init__()
                self._param_to_attr_map = {}
                self._managed_publishers = []
                self._managed_subscribers = []
                self._managed_timers = []
                self._managed_services = []
                self._managed_action_servers = []
                self._timer_definitions = []

            def _resolve_topic_or_service_name(self, name_arg):
                """Resolves an argument that could be a string or a Parameter placeholder."""
                if isinstance(name_arg, str):
                    return name_arg
                if isinstance(name_arg, Parameter):
                    if name_arg.attr_name is None:
                        raise ValueError(f"Parameter for ROS name '{name_arg.name}' was not correctly assigned to a class attribute.")
                    return getattr(self.user_instance, name_arg.attr_name)
                raise TypeError(f"Invalid type for topic/service name: {type(name_arg)}. Must be str or parameter().")

            def _on_parameter_event(self, params):
                """Callback for when parameters are changed externally (e.g., `ros2 param set`)."""
                for param in params:
                    if param.name in self._param_to_attr_map:
                        attr_name = self._param_to_attr_map[param.name]
                        setattr(self.user_instance, attr_name, param.value)
                        self.get_logger().info(
                            f"Parameter '{param.name}' updated dynamically. "
                            f"Set '{attr_name}' to: {param.value}"
                        )
                return SetParametersResult(successful=True)

            def _initialize_parameters(self):
                self.get_logger().info("  -> Declaring parameters and setting up dynamic reconfigure...")
                type_hints = inspect.get_annotations(user_cls)

                for attr_name, placeholder in inspect.getmembers(user_cls):
                    if isinstance(placeholder, Parameter):
                        self._param_to_attr_map[placeholder.name] = attr_name
                        param_type = type_hints.get(attr_name)
                        ros_param_type = {
                            str: ParameterType.STRING,
                            int: ParameterType.INTEGER,
                            float: ParameterType.DOUBLE,
                            bool: ParameterType.BOOL,
                            list: ParameterType.STRING_ARRAY
                        }.get(param_type, ParameterType.NOT_SET)

                        descriptor = ParameterDescriptor(name=placeholder.name, type=ros_param_type)
                        self.declare_parameter(
                            placeholder.name, placeholder.default_value, descriptor
                        )
                        param_value = self.get_parameter(placeholder.name).value
                        setattr(self.user_instance, attr_name, param_value)
                        self.get_logger().info(f"     - '{placeholder.name}' ({attr_name}) = {param_value}")

                self.add_on_set_parameters_callback(self._on_parameter_event)

            def _initialize_action_clients(self):
                self.get_logger().info("  -> Setting up action clients...")
                for attr_name, placeholder in inspect.getmembers(user_cls):
                    if isinstance(placeholder, ActionClientPlaceholder):
                        resolved_name = self._resolve_topic_or_service_name(placeholder.name)
                        action_type_class = placeholder.action_type
                        if isinstance(action_type_class, str):
                            action_type_class = _import_type_from_string(action_type_class)

                        self.get_logger().info(f"     - Creating action client for '{attr_name}' on action '{resolved_name}'")
                        client = ActionClient(self, action_type_class, resolved_name)
                        setattr(self.user_instance, attr_name, client)

            def _initialize_communications(self):
                self.get_logger().info("  -> Setting up communications...")
                for name, method in inspect.getmembers(self.user_instance, predicate=inspect.ismethod):
                    original_func = method.__func__
                    timer_infos = getattr(original_func, '_ros_timers', None)
                    pub_infos = getattr(original_func, '_ros_publishers', None)
                    sub_infos = getattr(original_func, '_ros_subscribers', None)
                    srv_infos = getattr(original_func, '_ros_services', None)
                    act_srv_infos = getattr(original_func, '_ros_action_servers', None)

                    if not any([timer_infos, pub_infos, sub_infos, srv_infos, act_srv_infos]):
                        continue
                    callback_to_use = method
                    if pub_infos:
                        publishers = []
                        for pub_info in pub_infos:
                            resolved_topic = self._resolve_topic_or_service_name(pub_info['topic'])
                            msg_type_class = pub_info['msg_type']
                            if isinstance(msg_type_class, str):
                                msg_type_class = _import_type_from_string(msg_type_class)
                            self.get_logger().info(f"     - Creating publisher for '{name}' on topic '{resolved_topic}'")
                            p = self.create_publisher(msg_type_class, resolved_topic, pub_info['qos'])
                            publishers.append(p)
                            self._managed_publishers.append(p)
                        def create_publisher_wrapper(user_method, pubs):
                            @functools.wraps(user_method)
                            def wrapper(*a, **kw):
                                result = user_method(*a, **kw)
                                if result is not None:
                                    for pub in pubs:
                                        pub.publish(result)
                                return result
                            return wrapper
                        callback_to_use = create_publisher_wrapper(method, publishers)

                    if srv_infos:
                        for srv_info in srv_infos:
                            resolved_name = self._resolve_topic_or_service_name(srv_info['name'])
                            srv_type_class = srv_info['type']
                            if isinstance(srv_type_class, str):
                                srv_type_class = _import_type_from_string(srv_type_class)

                            self.get_logger().info(f"     - Creating service for '{name}' on '{resolved_name}'")
                            srv = self.create_service(srv_type_class, resolved_name, method)
                            self._managed_services.append(srv)

                    if act_srv_infos:
                        for act_srv_info in act_srv_infos:
                            resolved_name = self._resolve_topic_or_service_name(act_srv_info['name'])
                            act_type_class = act_srv_info['type']
                            if isinstance(act_type_class, str):
                                act_type_class = _import_type_from_string(act_type_class)

                            self.get_logger().info(f"     - Creating action server for '{name}' on '{resolved_name}'")
                            act_srv = ActionServer(self, act_type_class, resolved_name, method)
                            self._managed_action_servers.append(act_srv)

                    if sub_infos:
                        for sub_info in sub_infos:
                            resolved_topic = self._resolve_topic_or_service_name(sub_info['topic'])
                            msg_type_class = sub_info['msg_type']
                            if isinstance(msg_type_class, str):
                                msg_type_class = _import_type_from_string(msg_type_class)
                            callback_for_sub = method
                            if sub_info.get('debug_log', False):
                                def create_debug_wrapper(user_callback, topic, msg_type_str):
                                    @functools.wraps(user_callback)
                                    def debug_wrapper(msg):
                                        self.get_logger().debug(f"Received message on topic '{topic}' (type: {msg_type_str})")
                                        return user_callback(msg)
                                    return debug_wrapper
                                msg_type_name = msg_type_class.__name__
                                callback_for_sub = create_debug_wrapper(method, resolved_topic, msg_type_name)
                            self.get_logger().info(f"     - Creating subscriber for '{name}' on topic '{resolved_topic}'")
                            s = self.create_subscription(
                                msg_type_class, resolved_topic, callback_for_sub, sub_info['qos']
                            )
                            self._managed_subscribers.append(s)
                    if timer_infos:
                        if sub_infos:
                            self.get_logger().warn(f"Method '{name}' has both @subscriber and @timer. The @timer will be ignored.")
                            continue
                        
                        is_lifecycle_node = isinstance(self, LifecycleNode)
                        for timer_info in timer_infos:
                            if is_lifecycle_node:
                                self.get_logger().info(f"     - Deferring timer creation for '{name}' (Lifecycle Node)")
                                self._timer_definitions.append({'period': timer_info['period'], 'callback': callback_to_use})
                            else:
                                self.get_logger().info(f"     - Creating timer for '{name}' with period {timer_info['period']}s")
                                t = self.create_timer(timer_info['period'], callback_to_use)
                                self._managed_timers.append(t)

            def _cleanup_ros_entities(self):
                """Destroy all publishers, subscribers, and timers created by the framework."""
                self.get_logger().info("Cleaning up framework-managed ROS entities...")
                for pub in self._managed_publishers:
                    self.destroy_publisher(pub)
                self._managed_publishers.clear()
                for sub in self._managed_subscribers:
                    self.destroy_subscription(sub)
                self._managed_subscribers.clear()
                for timer in self._managed_timers:
                    self.destroy_timer(timer)
                self._managed_timers.clear()
                for srv in self._managed_services:
                    self.destroy_service(srv)
                self._managed_services.clear()
                # Action servers are managed by the node and don't need explicit destruction
                self._managed_action_servers.clear()
                self.get_logger().info("Cleanup complete.")

            def _call_user_hook(self, hook_name, *args):
                """Helper to safely call a hook on the user's class instance."""
                user_hook = getattr(self.user_instance, hook_name, None) 
                if callable(user_hook):
                    self.get_logger().info(f"Executing user hook: {hook_name}")
                    try:
                        result = user_hook(*args)
                        if result is False or result == TransitionCallbackReturn.FAILURE:
                            return TransitionCallbackReturn.FAILURE
                    except Exception as e:
                        self.get_logger().error(f"Error in user hook '{hook_name}': {e}", exc_info=True)
                        return TransitionCallbackReturn.ERROR
                return TransitionCallbackReturn.SUCCESS

        # --- Detect if this should be a lifecycle node ---
        is_lifecycle = getattr(user_cls, '_is_lifecycle_node', False) or any(hasattr(user_cls, hook) for hook in [
            'on_configure', 'on_activate', 'on_deactivate', 'on_cleanup', 'on_shutdown', 'on_error'
        ])

        if is_lifecycle:
            class GenesysLifecycleNodeWrapper(GenesysNodeLogicMixin, LifecycleNode):
                def __init__(self):
                    super().__init__(node_name)
                    self._init_logic(user_cls)
                    self.get_logger().info(f"'{node_name}' detected as a Lifecycle Node. Awaiting configuration.")

                def on_configure(self, state):
                    self.get_logger().info("on_configure() is called.")
                    self._initialize_parameters()
                    self._initialize_action_clients()
                    self._initialize_communications()
                    if self._call_user_hook('on_configure', state) != TransitionCallbackReturn.SUCCESS:
                        self.get_logger().error("User's on_configure hook failed. Configuration aborted.")
                        self._cleanup_ros_entities()
                        return TransitionCallbackReturn.FAILURE
                    return TransitionCallbackReturn.SUCCESS

                def on_activate(self, state):
                    self.get_logger().info("on_activate() is called.")
                    # Activate timers
                    self.get_logger().info("  -> Activating timers...")
                    for timer_def in self._timer_definitions:
                        t = self.create_timer(timer_def['period'], timer_def['callback'])
                        self._managed_timers.append(t)

                    # Call user hook
                    if self._call_user_hook('on_activate', state) != TransitionCallbackReturn.SUCCESS:
                        self.get_logger().error("User's on_activate hook failed. Deactivating timers.")
                        for timer in self._managed_timers:
                            self.destroy_timer(timer)
                        self._managed_timers.clear()
                        return TransitionCallbackReturn.FAILURE

                    # Finally, call super() to activate publishers.
                    return super().on_activate(state)

                def on_deactivate(self, state):
                    self.get_logger().info("on_deactivate() is called.")
                    # Call user hook first
                    if self._call_user_hook('on_deactivate', state) != TransitionCallbackReturn.SUCCESS:
                        return TransitionCallbackReturn.FAILURE

                    # Deactivate timers
                    self.get_logger().info("  -> Deactivating timers...")
                    for timer in self._managed_timers:
                        self.destroy_timer(timer)
                    self._managed_timers.clear()

                    # Finally, call super() to deactivate publishers.
                    return super().on_deactivate(state)

                def on_cleanup(self, state):
                    self.get_logger().info("on_cleanup() is called.")
                    self._cleanup_ros_entities()
                    return self._call_user_hook('on_cleanup', state)

                def on_shutdown(self, state):
                    self.get_logger().info("on_shutdown() is called.")
                    self._cleanup_ros_entities()
                    return self._call_user_hook('on_shutdown', state)

                def on_error(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
                    self.get_logger().error(f"on_error() is called from state {state.label}.")
                    return self._call_user_hook('on_error', state)

            return GenesysLifecycleNodeWrapper
        else:
            class GenesysNodeWrapper(GenesysNodeLogicMixin, Node):
                def __init__(self):
                    super().__init__(node_name)
                    self._init_logic(user_cls)
                    self.get_logger().info(f"Initializing Genesys node '{node_name}' for class '{user_cls.__name__}'...")
                    self._initialize_parameters()
                    self._initialize_action_clients()
                    self._initialize_communications()
                    self.get_logger().info("Node initialization complete.")
            return GenesysNodeWrapper

    return decorator