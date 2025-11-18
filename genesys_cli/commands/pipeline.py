import click
import os
import subprocess
import time
import yaml
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from genesys.pipeline.parser import parse_pipeline
from genesys.pipeline.generator import generate_launch_description_from_pipeline
from launch import LaunchService

# Global variable to hold the current LaunchService instance
current_launch_service = None

class PipelineEventHandler(FileSystemEventHandler):
    def __init__(self, pipeline_file_path, ctx):
        super().__init__()
        self.pipeline_file_path = pipeline_file_path
        self.ctx = ctx
        self.last_modified = os.path.getmtime(pipeline_file_path)
        click.secho(f"Watching for changes in: {pipeline_file_path}", fg="cyan")

    def on_modified(self, event):
        # Debounce to avoid multiple events for a single save
        if time.time() - self.last_modified < 1.0:
            return
        
        if event.src_path == self.pipeline_file_path:
            click.secho(f"\nDetected change in {event.src_path}. Reloading pipeline...", fg="yellow")
            self.last_modified = time.time()
            self.reload_pipeline()

    def reload_pipeline(self):
        global current_launch_service
        if current_launch_service:
            click.secho("Shutting down previous pipeline...", fg="yellow")
            current_launch_service.shutdown()
            current_launch_service = None
            time.sleep(1) # Give some time for shutdown

        click.secho("Starting new pipeline...", fg="green")
        try:
            # Re-run the run_pipeline command
            self.ctx.invoke(run_pipeline, pipeline_file=self.pipeline_file_path)
        except Exception as e:
            click.secho(f"Error reloading pipeline: {e}", fg="red")


@click.group("pipeline")
def pipeline():
    """Manage and run Genesys pipelines."""
    pass

@pipeline.command("create")
@click.argument("pipeline_name")
def create_pipeline(pipeline_name):
    """Creates a new pipeline YAML file."""
    pipeline_file = f"{pipeline_name}.yaml"
    if os.path.exists(pipeline_file):
        click.secho(f"Error: Pipeline file '{pipeline_file}' already exists.", fg="red")
        return

    default_pipeline = {
        'pipeline': {
            'name': pipeline_name,
            'nodes': []
        }
    }

    with open(pipeline_file, 'w') as f:
        yaml.dump(default_pipeline, f, default_flow_style=False)

    click.secho(f"✓ Created pipeline file: {pipeline_file}", fg="green")

@pipeline.command("run")
@click.argument("pipeline_file")
@click.pass_context
def run_pipeline(ctx, pipeline_file):
    """Runs a pipeline from a YAML file."""
    global current_launch_service
    if not os.path.exists(pipeline_file):
        click.secho(f"Error: Pipeline file '{pipeline_file}' not found.", fg="red")
        return

    pipeline_dict = parse_pipeline(pipeline_file)
    launch_description = generate_launch_description_from_pipeline(pipeline_dict)
    
    current_launch_service = LaunchService()
    current_launch_service.include_launch_description(launch_description)
    
    click.secho(f"Running pipeline from {pipeline_file}...", fg="green")
    current_launch_service.run()
    click.secho(f"Pipeline from {pipeline_file} finished.", fg="green")


@pipeline.command("watch")
@click.argument("pipeline_file")
@click.pass_context
def watch_pipeline(ctx, pipeline_file):
    """Monitors a pipeline file and automatically reloads the pipeline on changes."""
    if not os.path.exists(pipeline_file):
        click.secho(f"Error: Pipeline file '{pipeline_file}' not found.", fg="red")
        return

    # Initial run
    ctx.invoke(run_pipeline, pipeline_file=pipeline_file)

    event_handler = PipelineEventHandler(os.path.abspath(pipeline_file), ctx)
    observer = Observer()
    observer.schedule(event_handler, path=os.path.dirname(os.path.abspath(pipeline_file)), recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
    
    click.secho("Pipeline watch stopped.", fg="green")
    global current_launch_service
    if current_launch_service:
        current_launch_service.shutdown()
        current_launch_service = None
