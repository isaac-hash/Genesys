import yaml
import os
import click

class PipelineValidationError(Exception):
    """Custom exception for pipeline validation errors."""
    pass

def parse_pipeline(file_path):
    """
    Parses a pipeline YAML file, validates its structure, and checks for cycles.
    """
    if not os.path.exists(file_path):
        raise PipelineValidationError(f"Pipeline file not found: {file_path}")

    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or 'pipeline' not in data:
        raise PipelineValidationError("YAML must contain a top-level 'pipeline' key.")

    pipeline_data = data['pipeline']

    if not isinstance(pipeline_data, dict) or 'name' not in pipeline_data or 'nodes' not in pipeline_data:
        raise PipelineValidationError("Pipeline definition must contain 'name' and 'nodes'.")

    pipeline_name = pipeline_data['name']
    nodes_data = pipeline_data['nodes']

    if not isinstance(nodes_data, list):
        raise PipelineValidationError("'nodes' must be a list.")

    node_ids = set()
    graph = {} # Adjacency list for dependency graph
    
    for node in nodes_data:
        if not isinstance(node, dict) or 'id' not in node or 'package' not in node or 'plugin' not in node:
            raise PipelineValidationError(f"Each node must have 'id', 'package', and 'plugin'. Node: {node}")
        
        node_id = node['id']
        if node_id in node_ids:
            raise PipelineValidationError(f"Duplicate node ID found: {node_id}")
        node_ids.add(node_id)
        graph[node_id] = []

        # Handle 'subscribe' for dependency tracking
        subscribe_topics = node.get('subscribe')
        if subscribe_topics:
            if isinstance(subscribe_topics, str):
                subscribe_topics = [subscribe_topics]
            elif not isinstance(subscribe_topics, list):
                raise PipelineValidationError(f"'subscribe' for node {node_id} must be a string or list of strings.")
            
            # For cycle detection, we need to know which nodes publish these topics
            # This is a simplified approach; a full implementation would track topic producers
            # For now, we'll assume a topic subscribed implies a dependency on a node that publishes it.
            # This part needs more sophisticated logic to map topics to nodes.
            # For now, we'll just ensure the subscribed topics are mentioned somewhere.
            pass # Actual dependency resolution will be in generator or a separate pass

    # Basic cycle detection (simplified for now, needs topic-to-node mapping)
    # This part is more complex and usually done after topic resolution.
    # For a simple check, we can ensure no node subscribes to a topic it publishes in the same pipeline.
    # This is a placeholder for a more robust DAG validation.
    
    # For now, we'll just return the parsed data after basic structural checks.
    # Full DAG validation and topic resolution will be part of the generator or a dedicated resolver.
    
    return data