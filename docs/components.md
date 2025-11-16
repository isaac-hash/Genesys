# Components in Genesys

This document describes how to use components in the Genesys framework.

## What are components?

Components are reusable ROS 2 nodes that can be loaded into a container process. This is more efficient than running each node in a separate process.

## Creating a component

To create a component, use the `genesys make:component` command:

```bash
genesys make:component my_component --pkg my_package
```

This will create a new file `my_component.py` in your package, with a class `MyComponent` decorated with `@component`.

## Using components in a launch file

To use a component in a launch file, you need to create a `ComposableNodeContainer` and add your component to it as a `ComposableNode`.

See the `mixed_launch.py.j2` template for an example.
