# Migration Guide

This document describes how to migrate an existing Genesys project to use components.

## Migrating a node to a component

To migrate an existing node to a component, you need to:

1.  Replace the `@node` decorator with the `@component` decorator.
2.  Make sure your class inherits from `rclpy.node.Node`.
3.  Remove the `main` function and the `if __name__ == '__main__':` block.
4.  Add a `get_node_factory` function.
5.  Add the `rclpy_components` entry point to your `setup.py` file.

See the `component.py.j2` template for an example.
