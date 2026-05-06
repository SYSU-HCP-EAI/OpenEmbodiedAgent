# Trajectory generation

## Overview sequence processing
The following diagram shows the steps to process and execute
a list of commands given as sequence. The diagram also shows the resulting data
structures of each processing step.
![OverviewProcessing](sequence_processing_overview.png)

## Overview classes PlanningContext
The following diagram shows the different classes responsible for the loading
of the different planning contexts (e.g. Ptp, Lin, Circ) and the
relationship between them.
![DiagClassPlanningContext](diag_class_planning_context.png)

## Relationship MoveGroupCapabilities and ComandListManager
The following diagram shows the relationship between the MoveGroupCapabilities
and the CommandListManager.
![DiagClassCapabilities](diag_class_capabilities.png)

## Blending algorithm description
A description of the used blending algorithm can be found
[here](MotionBlendAlgorithmDescription.pdf).
