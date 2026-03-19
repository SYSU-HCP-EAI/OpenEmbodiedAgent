# Robot Control & Environment Configuration Skill

## 1. Introduction
This SKILL equips you with the capability to configure environments for physical robots from scratch, run demonstration code, and program trajectory controls. 
*(Note: Hardware is assumed to be pre-configured by the user. The Rekep vision-control module will be handled by a separate SKILL in the future.)*

## 2. Core Workflows
When you are tasked with configuring a new robot or controlling an existing one, you MUST follow these specialized workflows. Due to the complexity of robot control, the detailed instructions are modularized. **You MUST read the corresponding reference files before proceeding:**

1. **Environment Setup:** Searching docs, setting up isolated virtual environments, and running the README demo. 
   👉 **Read:** `references/01_environment_setup.md`
2. **Skill Generation & Memory Management (CRITICAL):** Using your native tools to create and maintain a sub-SKILL for this specific robot brand as long-term memory.
   👉 **Read:** `references/02_skill_generation_and_memory.md`
3. **Trajectory & Movement Execution:** Writing scripts (ROS/SDK) to control the robot's physical movement.
   👉 **Read:** `references/03_trajectory_and_execution.md`
4. **Safety Guidelines:** Rules for interacting with the physical world safely without being overly sensitive.
   👉 **Read:** `references/04_safety_guidelines.md`
5. **Workspace Management & Security:** Rules for organizing scripts, maintaining a clean workspace, and preventing data leaks.
   👉 **Read:** `references/05_workspace_and_security.md`

## 3. General Rules
- **Use Native Tools:** Do not reinvent the wheel. Rely heavily on your native OpenClaw/OEA tools (e.g., terminal execution, file editing, web browsing, and importantly, the **SKILL creation tool**).
- **Check References:** Always check the `references/` directory. Do not try to guess the workflow.
- **Expand References:** As you encounter new robot projects, you are encouraged to create new reference files (e.g., `references/project_A.md`, `references/interface_specs.md`, `references/pitfalls.md`) to manage the growing knowledge base.
