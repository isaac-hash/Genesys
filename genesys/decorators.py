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

from .node_base import NodeBase


# ============================================================
#   Q O S   R E G I S T R Y  (SCALABLE + PLUGGABLE)
# ============================================================

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



# ============================================================
#   H E L P E R S   /   P L A C E H O L D E R S
# ============================================================

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


def action_client(name, action_type):
    return ActionClientPlaceholder(name, action_type)


def parameter(name, default_value=None):
    return Parameter(name, default_value)




# ============================================================
#   D E C O R A T O R S
# ============================================================

def publisher(topic, msg_type, qos="default"):
    qos = resolve_qos(qos)

    def decorator(func):
        if not hasattr(func, '_ros_publishers'):
            func._ros_publishers = []
        func._ros_publishers.append({
            'topic': topic,
            'msg_type': msg_type,
            'qos': qos
        })
        return func

    return decorator


def subscriber(topic, msg_type, qos="default", debug_log=False):
    qos = resolve_qos(qos)

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
    def decorator(func):
        if not hasattr(func, '_ros_timers'):
            func._ros_timers = []
        func._ros_timers.append({'period': period_sec})
        return func

    return decorator



def service(service_name, service_type):
    def decorator(func):
        if not hasattr(func, '_ros_services'):
            func._ros_services = []
        func._ros_services.append({'name': service_name, 'type': service_type})
        return func
    return decorator



def action_server(action_name, action_type):
    def decorator(func):
        if not hasattr(func, '_ros_action_servers'):
            func._ros_action_servers = []
        func._ros_action_servers.append({'name': action_name, 'type': action_type})
        return func
    return decorator


def component(name=None, **meta):
    def wrapper(cls):
        comp_name = name or getattr(cls, '__name__').lower()
        cls.__genesys_is_component__ = True
        cls.__genesys_component_name__ = comp_name

        def get_node_factory():
            from rclpy_components import NodeFactory
            return NodeFactory(cls)

        cls.get_node_factory = staticmethod(get_node_factory)
        # preserve existing genesys attributes
        return cls
    return wrapper


def lifecycle_node(cls):
    cls._is_lifecycle_node = True
    return cls




# ===================================================================
#   N O D E   /   L I F E C Y C L E   W R A P P E R S
#   (unchanged except QoS handling)
# ===================================================================

def node(node_name):

    def decorator(user_cls):



        # =============== LIFECYCLE OR NORMAL NODE =================

        is_lifecycle = getattr(user_cls, "_is_lifecycle_node", False)

        if is_lifecycle:
            class Wrapper(NodeBase, LifecycleNode):
                def __init__(self):
                    super().__init__(node_name, user_cls)
            return Wrapper

        else:
            class Wrapper(NodeBase, Node):
                def __init__(self):
                    super().__init__(node_name, user_cls)
            return Wrapper

    return decorator