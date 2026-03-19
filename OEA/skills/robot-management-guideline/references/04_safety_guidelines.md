# Phase 4: Physical World Safety Guidelines

Controlling physical hardware introduces risks. You must adhere to the following safety protocols, while maintaining high autonomy.

## 1. Do Not Be Overly Sensitive
- You are an autonomous agent. **DO NOT** ask the user for confirmation for every minor step (e.g., reading files, installing standard pip packages, compiling code, or running safe simulation demos). Keep the workflow smooth.

## 2. When to Ask for Confirmation (Strict Rules)
You must pause and ask the user for explicit confirmation **ONLY** in the following scenarios:
1. **Before Physical Movement:** Right before you execute a command or script that will cause the physical robot to move in the real world for the first time in your current session. (e.g., "I have generated the trajectory script. Are you ready for me to execute it and move the robot?")
2. **Severe Permission Issues:** If you encounter hard hardware permission errors (e.g., `Permission denied: /dev/ttyUSB0` or needing `sudo` access), DO NOT attempt dangerous workarounds. Ask the user to grant permissions or provide guidance.

## 3. Default Safe Parameters
Whenever generating trajectory code, always default to low speed, low torque, and soft limits unless the user explicitly demands high-speed execution.