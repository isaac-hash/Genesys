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

def get_cpp_node_template(node_name, class_name):
    """Returns the boilerplate for a C++ node."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'cpp')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template('publisher.cpp.j2')
    return template.render(node_name=node_name, class_name=class_name)