# Phase 3: Trajectory Generation and Execution

Once the environment is healthy and drivers are running, you will be tasked with controlling the robot's physical movement.

## 1. Interface Identification
- Determine if the robot is controlled via **ROS** (e.g., publishing to standard geometry topics, Action Clients, MoveIt!) or a specific **Python/C++ SDK**.

## 2. Code Generation
- **Align with Official Demos:** Your code MUST align with the patterns found in the official `demo/` or `examples/` directory (e.g., initialization sequence, heartbeat requirements, control frequency).
- Write a clean, well-commented script defining the robot's trajectory (e.g., waypoints, joint angles, or Cartesian coordinates).
- Ensure the script correctly imports the necessary SDKs or ROS libraries.
- For initial tests, explicitly set velocity and acceleration limits to a low, safe threshold (e.g., 10%-20% of max speed) within the code.

## 3. Execution Execution
- Save the script to the workspace.
- Execute the script using your terminal tool **strictly within the activated virtual environment**.
- **Verify Physical Movement:** Do NOT rely solely on the script's terminal output (e.g., "Command sent successfully") to confirm the robot is actually moving. You MUST design logic to verify physical execution.
  - **Primary Method:** Monitor the robot's real-time pose/joint states via the SDK or ROS topics (e.g., `GetArmStatus()`, `/joint_states`). Compare the current state with the target state over time.
  - **Secondary Methods:** If pose feedback is unavailable or unreliable, consider using other sensors if available (e.g., odometry, vision/camera feedback) to confirm movement.
- Monitor the terminal output for success signals or collision warnings. Update the code if it fails.