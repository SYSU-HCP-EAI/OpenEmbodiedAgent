# Phase 5: Workspace Management and Script Organization

As an autonomous agent, you must maintain a clean, organized, and secure workspace. Do not leave temporary scripts scattered in the root directory.

## 1. Workspace Directory Management
- **Create a `WORKSPACE.md`:** Maintain a `WORKSPACE.md` file in the root of your workspace. This file acts as a directory index.
- **Document Projects:** Every time you start working on a new robot or project, add an entry to `WORKSPACE.md` describing the project, its directory path, and its purpose.
- **Clean Up:** Always clean up temporary files. Move related scripts into their respective project folders or skill directories.

## 2. Script Organization and Reusability
- **Differentiate Scripts:** Clearly separate successful, reusable scripts from failed or experimental ones.
- **Utilize the `scripts/` Directory:** When you create a successful, reusable script for a specific robot, move it into the `scripts/` directory of that robot's SKILL (e.g., `skills/agilex-piper-robot/scripts/`).
- **Document in SKILL:** Update the robot's `SKILL.md` to document the purpose, usage, and running context of every script placed in the `scripts/` directory.
- **Handle Failed Scripts:** Failed scripts should NOT be placed in the SKILL's `scripts/` directory. They can be stored in a separate `archive/` or `failed_experiments/` folder within the project directory for future reference, or simply deleted if they have no reference value.

## 3. Security and Privacy (CRITICAL)
- **Prevent Data Leaks:** Before saving any script to a SKILL directory or uploading it anywhere, you MUST review the code for sensitive information.
- **Scrub Sensitive Data:** Ensure the script does NOT contain:
  - Host IP addresses (use `localhost`, `127.0.0.1`, or placeholders like `<ROBOT_IP>`).
  - Passwords, API keys, or authentication tokens.
  - Personal user data or specific local absolute paths that reveal user information (use relative paths or environment variables where possible).
