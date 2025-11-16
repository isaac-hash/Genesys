# Pipelines in Genesys

This document describes how to use pipelines in the Genesys framework.

## What are pipelines?

Pipelines are a way to define a series of connected components. The Genesys CLI can then generate a launch file to run the pipeline.

## Creating a pipeline

To create a pipeline, use the `genesys pipeline create` command:

```bash
genesys pipeline create my_pipeline
```

This will create a new file `my_pipeline.yaml` in your current directory.

## Running a pipeline

To run a pipeline, use the `genesys pipeline run` command:

```bash
genesys pipeline run my_pipeline.yaml
```
