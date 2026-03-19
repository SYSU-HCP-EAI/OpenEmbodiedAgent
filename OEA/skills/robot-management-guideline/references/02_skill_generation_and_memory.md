# Phase 2: Skill Generation and Memory Management

To ensure persistent memory and avoid repeating the same configuration process for the same robot brand in the future, you must act as a maintainer of your own knowledge base.

## 1. When to Generate the Skill
- **Immediately after reading the project's README** (before running the complex demo), you MUST invoke your **native SKILL creation tool** to generate a new SKILL specifically dedicated to this robot brand (e.g., `brandX_robot_manual`).

## 2. Content of the Generated Skill
When generating this new SKILL, ensure it contains:
1. **Source Links:** The URL of the official documentation or GitHub repository.
2. **Local Paths:** The absolute local path to the cloned repository and the local `README.md`.
3. **Boot & Config Process:** Step-by-step instructions on how to activate the virtual environment, launch the core drivers, and execute the basic demo.

## 3. Continuous Updating (Lessons Learned)
- **Treat this new SKILL as your long-term memory.** 
- Every time a terminal command fails (e.g., dependency conflicts, compilation errors, connection timeouts), and you successfully find a workaround, you MUST manually update this robot's specific SKILL document.
- Add a "Troubleshooting / Lessons Learned" section in that skill to record the exact error and the solution. Do not make the same mistake twice.