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

def get_cpp_component_templates(component_type, pkg_name, class_name, component_description):
    """
    Returns the boilerplate for C++ component header, source,
    initial registration file content, and initial plugin XML file content.
    """
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'cpp')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    # Load full templates for .hpp and .cpp
    hpp_template = env.get_template('component.hpp.j2')
    hpp_content = hpp_template.render(package_name=pkg_name, class_name=class_name)

    cpp_template = env.get_template('component.cpp.j2')
    cpp_content = cpp_template.render(
        package_name=pkg_name,
        class_name=class_name,
        component_name=class_name.lower(), # Assuming component_name is lowercased class_name
        component_type=component_type
    )

    # Load initial full file content for register_components.cpp and plugin.xml
    # These are for the *first* component in the package.
    register_components_template = env.get_template('register_components.cpp.j2')
    initial_register_content = register_components_template.render(package_name=pkg_name, class_name=class_name)

    plugin_xml_template = env.get_template('plugin.xml.j2')
    initial_plugin_content = plugin_xml_template.render(
        package_name=pkg_name,
        class_name=class_name,
        component_description=component_description
    )

    return hpp_content, cpp_content, initial_register_content, initial_plugin_content

def get_cpp_component_cmakelists_template(context):
    """Returns the boilerplate for a C++ component CMakeLists.txt file."""
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates', 'cpp')),
        trim_blocks=True,
        lstrip_blocks=True
    )

    template = env.get_template('cmakelists_component.txt.j2')
    return template.render(**context)
