import os
from jinja2 import Environment, FileSystemLoader

def get_template_path():
    """
    Get the templates directory path.
    Priority:
    1. GENESYS_TEMPLATES_PATH environment variable
    2. Default path relative to this file.
    Returns:
        str: Path to the templates directory
    """
    # Check environment variable first
    templates_path = os.environ.get('GENESYS_TEMPLATES_PATH')
    if templates_path and os.path.isdir(templates_path):
        return templates_path

    # Fallback to default path
    return os.path.join(os.path.dirname(__file__), 'templates')

def get_template_env(subfolder=''):
    """
    Returns a Jinja2 environment configured for the templates directory.
    An optional subfolder can be specified (e.g., 'python', 'cpp').
    """
    path = os.path.join(get_template_path(), subfolder)
    return Environment(
        loader=FileSystemLoader(path),
        trim_blocks=True,
        lstrip_blocks=True
    )

def get_python_node_template(node_type, node_name, class_name):
    """Returns the boilerplate for a Python node of a given type."""
    env = get_template_env('python')

    template = env.get_template(f"{node_type}.py.j2")
    node_content = template.render(node_name=node_name, class_name=class_name)

    main_template = env.get_template('python_main.py.j2')
    main_content = main_template.render(class_name=class_name)

    return f"{node_content}\n{main_content}"

def get_python_component_template(component_type, component_name, class_name):
    """Returns the boilerplate for a Python component of a given type."""
    env = get_template_env('python')

    # Always load the generic component.py.j2 template
    template = env.get_template("component.py.j2")
    
    # Pass component_type as a variable to the template
    return template.render(node_name=component_name, class_name=class_name, component_type=component_type)

def get_mixed_launch_template():
    """Returns the boilerplate for a mixed launch file."""
    env = get_template_env('launch')

    template = env.get_template("mixed_launch.py.j2")
    return template.render()

def get_cpp_node_template(node_name, class_name):
    """Returns the boilerplate for a C++ node."""
    env = get_template_env('cpp')

    template = env.get_template('publisher.cpp.j2')
    return template.render(node_name=node_name, class_name=class_name)

def get_cmakelists_template(package_name):
    """Returns the boilerplate for a CMakeLists.txt file."""
    env = get_template_env('cpp')

    template = env.get_template('cmakelists.txt.j2')
    return template.render(package_name=package_name)

def get_cpp_component_templates(component_type, pkg_name, class_name, description):
    """Renders all necessary C++ component templates."""
    env = get_template_env() # This will look in the root of templates dir

    hpp_template = env.get_template(f'cpp/{component_type}.hpp.j2')
    cpp_template = env.get_template(f'cpp/{component_type}.cpp.j2')
    register_template = env.get_template('cpp/register_components.cpp.j2')
    plugin_template = env.get_template('cpp/plugin.xml.j2')

    context = {
        "package_name": pkg_name,
        "class_name": class_name,
        "description": description
    }

    hpp_content = hpp_template.render(**context)
    cpp_content = cpp_template.render(**context)
    register_content = register_template.render(**context)
    plugin_content = plugin_template.render(**context)
    
    return hpp_content, cpp_content, register_content, plugin_content

def get_cpp_component_cmakelists_template(context):
    """Returns the boilerplate for a C++ component CMakeLists.txt file."""
    env = get_template_env('cpp')

    template = env.get_template('cmakelists_component.txt.j2')
    return template.render(**context)