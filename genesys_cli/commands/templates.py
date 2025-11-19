import os
from jinja2 import Environment, FileSystemLoader

def get_python_node_template(node_type, node_name, class_name):
    """Returns the boilerplate for a Python node of a given type."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'python')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template(f"{node_type}.py.j2")
    node_content = template.render(node_name=node_name, class_name=class_name)

    main_template = env.get_template('python_main.py.j2')
    main_content = main_template.render(class_name=class_name)

    return f"{node_content}\n{main_content}"

def get_python_component_template(component_type, component_name, class_name):
    """Returns the boilerplate for a Python component of a given type."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'python')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    # Always load the generic component.py.j2 template
    template = env.get_template("component.py.j2")
    
    # Pass component_type as a variable to the template
    return template.render(node_name=component_name, class_name=class_name, component_type=component_type)

def get_mixed_launch_template():
    """Returns the boilerplate for a mixed launch file."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'launch')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template("mixed_launch.py.j2")
    return template.render()

def get_cpp_node_template(node_name, class_name):
    """Returns the boilerplate for a C++ node."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'cpp')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template('publisher.cpp.j2')
    return template.render(node_name=node_name, class_name=class_name)

def get_cmakelists_template(package_name):
    """Returns the boilerplate for a CMakeLists.txt file."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'cpp')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template('cmakelists.txt.j2')
    return template.render(package_name=package_name)

def get_cpp_component_cmakelists_template(context):
    """Returns the boilerplate for a C++ component CMakeLists.txt file."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'cpp')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template('cmakelists_component.txt.j2')
    return template.render(**context)
