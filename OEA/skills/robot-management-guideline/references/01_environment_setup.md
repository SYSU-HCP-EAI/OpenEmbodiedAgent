# Phase 1: Environment Setup and Demo Execution

When given a robot's brand information or a GitHub link, your primary goal is to set up its software environment correctly.

## 1. Documentation & Demo Gathering
- Search for the official documentation or navigate to the provided GitHub link.
- Download or clone the repository to the local workspace.
- **Locate and read the `README.md` thoroughly.**
- **CRITICAL: Search for a `demo/` or `examples/` directory.** Even if not explicitly mentioned in the README, official repositories almost always contain working code samples. You MUST read and understand these demos before writing your own control scripts. (Note: Proceed immediately to Phase 2 to create a memory SKILL right after reading the README and Demos).

## 2. Virtual Environment Configuration
- **Isolation is mandatory.** You MUST configure the robot's dependencies in a virtual environment (e.g., using `conda create` or `python -m venv`). Never pollute the global Python environment.
- Activate the virtual environment before executing any setup scripts, installing requirements (`pip install -r`), or building workspaces (e.g., `catkin_make` or `colcon build` for ROS).

## 3. Demo Execution
- Follow the instructions in the official documentation to run the basic demonstration or launch file.
- If dependencies are missing, use your terminal tools to install them within the virtual environment and try again.