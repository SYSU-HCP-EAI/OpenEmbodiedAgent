# Migration Notes

API changes since last Noetic Release

- Servo::getLatestJointState was removed.  To get the latest joint positions of the group servo is working with use the CSM in the PSM.  Here is an example of how to get the latest joint positions:
        planning_scene_monitor_->getStateMonitor()->getCurrentState()->copyJointGroupPositions(move_group_name, positions);
