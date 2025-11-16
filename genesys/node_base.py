import functools
import inspect
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy
)
from rclpy.action import ActionServer, ActionClient
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

# Re-define or import necessary helpers from decorators.py
# For now, I'll copy them to avoid circular imports or complex refactoring at this stage.
# A better long-term solution would be to have a common 'utils' module.

QOS_REGISTRY = {
    "default": QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    ),
    "sensor": QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    ),
    "system": rclpy.qos.qos_profile_system_default,
    "services": rclpy.qos.qos_profile_services_default,
}

def resolve_qos(qos):
    """Allows QoS to be: QoSProfile OR string alias."""
    if isinstance(qos, QoSProfile):
        return qos
    if isinstance(qos, str):
        if qos not in QOS_REGISTRY:
            raise ValueError(f"Unknown QoS alias '{qos}'. Allowed: {list(QOS_REGISTRY.keys())}")
        return QOS_REGISTRY[qos]
    raise TypeError("QoS must be QoSProfile or string alias.")

def _import_type_from_string(type_string: str):
    parts = type_string.split('.')
    module_name = '.'.join(parts[:-1])
    class_name = parts[-1]
    module = __import__(module_name, fromlist=[class_name])
    return getattr(module, class_name)

class Parameter:
    def __init__(self, name, default_value=None):
        self.name = name
        self.default_value = default_value
        self.attr_name = None

    def __set_name__(self, owner, name):
        self.attr_name = name

class ActionClientPlaceholder:
    def __init__(self, name, action_type):
        self.name = name
        self.action_type = action_type
        self.attr_name = None

    def __set_name__(self, owner, name):
        self.attr_name = name


class NodeBase(Node):
    """
    Base class for Genesys nodes, handling automatic wiring of publishers,
    subscribers, timers, services, and action clients based on decorators.
    """
    def __init__(self, node_name: str, user_cls):
        super().__init__(node_name)
        self.user_instance = user_cls.__new__(user_cls)
        self.user_instance.logger = self.get_logger()
        self.user_instance.get_clock = self.get_clock
        self.user_instance.__init__() # Call the user's __init__
        self._param_to_attr_map = {}
        self._managed_publishers = []
        self._managed_subscribers = []
        self._managed_timers = []
        self._managed_services = []
        self._managed_action_servers = []
        self._timer_definitions = [] # This seems unused in the mixin, keeping for now.

        self._initialize_parameters(user_cls)
        self._initialize_action_clients(user_cls)
        self._initialize_communications(user_cls)

    def _resolve_topic_or_service_name(self, name_arg):
        if isinstance(name_arg, str):
            return name_arg
        if isinstance(name_arg, Parameter):
            return getattr(self.user_instance, name_arg.attr_name)
        raise TypeError

    def _on_parameter_event(self, params):
        for param in params:
            if param.name in self._param_to_attr_map:
                attr = self._param_to_attr_map[param.name]
                setattr(self.user_instance, attr, param.value)
        return SetParametersResult(successful=True)

    def _initialize_parameters(self, user_cls):
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
                self.declare_parameter(placeholder.name, placeholder.default_value, descriptor)
                val = self.get_parameter(placeholder.name).value
                setattr(self.user_instance, attr_name, val)

        self.add_on_set_parameters_callback(self._on_parameter_event)

    def _initialize_action_clients(self, user_cls):
        for attr_name, placeholder in inspect.getmembers(user_cls):
            if isinstance(placeholder, ActionClientPlaceholder):
                resolved = self._resolve_topic_or_service_name(placeholder.name)
                action_type = placeholder.action_type
                if isinstance(action_type, str):
                    action_type = _import_type_from_string(action_type)
                client = ActionClient(self, action_type, resolved)
                setattr(self.user_instance, attr_name, client)

    def _initialize_communications(self, user_cls):
        for name, method in inspect.getmembers(self.user_instance, predicate=inspect.ismethod):
            func = method.__func__

            pub_infos = getattr(func, '_ros_publishers', None)
            sub_infos = getattr(func, '_ros_subscribers', None)
            timer_infos = getattr(func, '_ros_timers', None)
            srv_infos = getattr(func, '_ros_services', None)
            act_srv_infos = getattr(func, '_ros_action_servers', None)

            callback = method

            # Publishers
            if pub_infos:
                pubs = []
                for info in pub_infos:
                    resolved = self._resolve_topic_or_service_name(info["topic"])
                    msg_type = info['msg_type']
                    if isinstance(msg_type, str):
                        msg_type = _import_type_from_string(msg_type)

                    p = self.create_publisher(msg_type, resolved, resolve_qos(info['qos']))
                    pubs.append(p)
                    self._managed_publishers.append(p)

                def wrapper(user_method, pubs_list):
                    @functools.wraps(user_method)
                    def _wrap(*a, **kw):
                        result = user_method(*a, **kw)
                        if result is not None:
                            for pub in pubs_list:
                                pub.publish(result)
                        return result
                    return _wrap

                callback = wrapper(method, pubs)

            # Subscribers
            if sub_infos:
                for info in sub_infos:
                    resolved = self._resolve_topic_or_service_name(info["topic"])
                    msg_type = info['msg_type']
                    if isinstance(msg_type, str):
                        msg_type = _import_type_from_string(msg_type)

                    cb = callback
                    if info['debug_log']:
                        def dbg_wrapper(user_cb):
                            @functools.wraps(user_cb)
                            def _wrap(msg):
                                self.get_logger().debug(f"[DEBUG] Sub recv: {resolved}")
                                return user_cb(msg)
                            return _wrap
                        cb = dbg_wrapper(callback)

                    s = self.create_subscription(msg_type, resolved, cb, resolve_qos(info['qos']))
                    self._managed_subscribers.append(s)

            # Services
            if srv_infos:
                for info in srv_infos:
                    resolved = self._resolve_topic_or_service_name(info["name"])
                    srv_type = info['type']
                    if isinstance(srv_type, str):
                        srv_type = _import_type_from_string(srv_type)
                    srv = self.create_service(srv_type, resolved, callback)
                    self._managed_services.append(srv)

            # Action Servers
            if act_srv_infos:
                for info in act_srv_infos:
                    resolved = self._resolve_topic_or_service_name(info["name"])
                    act_type = info['type']
                    if isinstance(act_type, str):
                        act_type = _import_type_from_string(act_type)
                    srv = ActionServer(self, act_type, resolved, callback)
                    self._managed_action_servers.append(srv)

            # Timers
            if timer_infos:
                for t in timer_infos:
                    period = t["period"]
                    timer = self.create_timer(period, callback)
                    self._managed_timers.append(timer)
