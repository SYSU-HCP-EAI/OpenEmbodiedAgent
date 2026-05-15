# PhyAgentOS Perception Runtime

This directory contains the Target-Configured Perception Runtime. It lets a runtime target declare external sensor and perception YAML files, runs strict preflight before a session starts, executes dependency-lazy perception plugins, and writes compact environment state to `ENVIRONMENT.md`.

The current implementation is a dependency-free MVP. Built-in dummy plugins work without physical sensors or model SDKs. Heavy integrations such as SAM, YOLO, Open3D, ROS, RealSense, and LiDAR SDKs must be added as lazy-loaded plugins and must not be imported at module import time.

## Files And Workspaces

Runtime protocol files live in the runtime workspace:

```text
TARGETS.md
SKILLS.md
SESSIONS.md
LOG.md
configs/runtime/sensors/<target_id>.sensors.yaml
configs/runtime/perception/<target_id>.perception.yaml
artifacts/runtime/<session_id>/
```

Perception writes environment state to the agent/shared workspace:

```text
ENVIRONMENT.md
artifacts/perception/<target_id>/<session_id>/
```

For a single-workspace setup, the runtime workspace and agent workspace can be the same directory.

For a fleet or shared-agent setup, start the runtime watchdog with an explicit environment workspace:

```bash
python scripts/run_runtime_watchdog.py \
  --workspace /path/to/robot_runtime_workspace \
  --environment-workspace /path/to/shared_agent_workspace
```

`--workspace` is used for `TARGETS.md`, `SKILLS.md`, `SESSIONS.md`, config YAML files, runtime artifacts, and `LOG.md`.

`--environment-workspace` is where perception writes `ENVIRONMENT.md`. This should be the workspace read by the upper-level Agent.

## Runtime Protocol Files

### TARGETS.md

`TARGETS.md` remains a compact target registry. Do not inline verbose sensor lists, model lists, or pipeline definitions here. Put those in external YAML files.

Example:

```yaml
version: runtime_target_registry_v1
targets:
  - id: franka_lab_a
    type: real_robot
    enabled: true
    workspace: workspaces/franka_lab_a
    supported_skills:
      - rekep_grasp
      - openpi_pick_place
    adapter: franka_real_adapter

    perception:
      enabled: true
      strict_preflight: true
      sensor_config_ref: configs/runtime/sensors/franka_lab_a.sensors.yaml
      perception_config_ref: configs/runtime/perception/franka_lab_a.perception.yaml
      artifact_dir: artifacts/perception/franka_lab_a
      config_version: franka_lab_a_perception_v1
```

Important rules:

- `perception.enabled` must be `true` when a skill declares required sensors or environment outputs.
- `strict_preflight` is currently required to be `true`.
- `sensor_config_ref` is required for any skill with `requires.sensors` or `requires.environment_outputs`.
- `perception_config_ref` is required when `requires.environment_outputs` is non-empty.
- Relative config paths are resolved relative to the runtime workspace.

### SKILLS.md

Skills declare what they need, not which plugin or model should satisfy it.

Example:

```yaml
version: runtime_skill_registry_v1
skills:
  - id: rekep_grasp
    category: builtin
    runtime: ReKepRuntime
    supported_target_types: [real_robot, sim]
    requires:
      sensors: [front_rgb, front_depth]
      environment_outputs: [objects_3d, scene_graph]
      object_queries_from_task: true
      min_confidence: 0.55
      require_metric_geometry: true
      strict_environment_contract: true
```

Behavior:

- `requires.sensors: []` and `requires.environment_outputs: []` means perception is not used.
- If `requires.sensors` is non-empty but `environment_outputs` is empty, runtime validates sensor config and target observation channels, but does not write `ENVIRONMENT.md`.
- If `environment_outputs` is non-empty, runtime loads the perception YAML, selects a pipeline that declares full coverage for those outputs, runs the plugin pipeline, verifies that the actual `EnvironmentDelta.generated_outputs` covers every requested output, and only then writes `ENVIRONMENT.md`.

### SESSIONS.md

Sessions select a target and a skill. They do not select perception plugins and cannot weaken strict preflight.

Example:

```yaml
version: runtime_sessions_v1
sessions:
  - session_id: sess_pick_apple_001
    target_ref: target://franka_lab_a
    skill_ref: skill://rekep_grasp
    task_description: "pick up the red apple on the table"
    status: pending
    routing:
      policy_endpoint: dummy://local
      adapter: franka_real_adapter
```

## Sensor Config YAML

Sensor config files use `runtime_sensor_config_v1`.

Example:

```yaml
version: runtime_sensor_config_v1
target_id: franka_lab_a
updated_at: "2026-05-14T00:00:00Z"

sensors:
  - id: front_rgb
    modality: rgb
    role: primary_scene_camera
    source: realsense_front/color
    observation_key: image.front_rgb
    frame_id: camera_front_color_optical_frame
    resolution: [1280, 720]
    hz: 30
    enabled: true
    calibration:
      intrinsics: calibrations/franka_lab_a/front_rgb_intrinsics.yaml
      extrinsics: calibrations/franka_lab_a/front_to_base.yaml
    required_fields:
      - source
      - observation_key
      - frame_id
      - resolution
      - calibration.intrinsics
      - calibration.extrinsics

  - id: front_depth
    modality: depth
    role: aligned_depth
    source: realsense_front/aligned_depth
    observation_key: depth.front_depth
    frame_id: camera_front_color_optical_frame
    resolution: [1280, 720]
    hz: 30
    enabled: true
    calibration:
      aligned_to: front_rgb
      depth_scale: 0.001
      depth_unit: meter
    required_fields:
      - source
      - observation_key
      - frame_id
      - resolution
      - calibration.aligned_to
      - calibration.depth_scale
      - calibration.depth_unit

observation_schema:
  image.front_rgb:
    dtype: uint8
    shape: [720, 1280, 3]
  depth.front_depth:
    dtype: float32
    shape: [720, 1280]
```

Preflight checks:

- `target_id` must match `TARGETS.md`.
- Every skill-required sensor and every selected plugin `requires_sensors` entry must exist and be enabled.
- Every required field listed in `required_fields` must be configured.
- Calibration files referenced by `calibration.intrinsics` or `calibration.extrinsics` must exist.
- `observation_schema` must include each required sensor's `observation_key`.
- Target observations must contain every required `observation_key`, with compatible dtype and shape.
- The runtime builds `SensorFrame` from the union of skill-required sensors and selected plugin-required sensors.

Observation keys support both direct and dotted lookup. For example, `image.front_rgb` can be read from:

```python
{"image": {"front_rgb": array}}
```

or directly from:

```python
{"image.front_rgb": array}
```

## Perception Config YAML

Perception config files use `runtime_perception_config_v1`.

Example:

```yaml
version: runtime_perception_config_v1
target_id: franka_lab_a
strict_preflight: true
default_outputs:
  - objects_3d
  - scene_graph

models:
  - id: rgbd_builder_builtin
    type: rgbd_object_builder
    provider: builtin_dummy
    install_hint: "No install needed for the dependency-free dummy plugin."

plugin_candidates:
  - id: dummy_segmenter
    type: segmenter_2d
    model_ref: rgbd_builder_builtin
    requires_sensors: [front_rgb]
    requires_modalities: [rgb]
    produces: [boxes_2d, labels, confidence]
    config:
      object_id: obj_red_apple
      label: red apple
      confidence: 0.95

  - id: rgbd_object_builder
    type: object_3d_builder
    model_ref: rgbd_builder_builtin
    requires_plugins: [dummy_segmenter]
    requires_outputs: [boxes_2d]
    requires_sensors: [front_rgb, front_depth]
    produces: [objects_3d, scene_graph]
    config:
      output_frame: base_link

pipelines:
  - id: rgbd_object_environment
    required_outputs: [objects_3d, scene_graph]
    use_plugins: [dummy_segmenter, rgbd_object_builder]
```

Pipeline selection is strict:

- Runtime chooses a pipeline whose `required_outputs` fully covers `skill.requires.environment_outputs`.
- Runtime does not downgrade outputs. `objects_3d` will not be replaced by `boxes_2d`.
- Runtime executes `pipelines[].use_plugins` in order.
- Selected plugin `requires_sensors` entries are merged into the required sensor set and checked during preflight.
- Plugin `requires_plugins`, `requires_sensors`, `model_ref`, and importable plugin module references are checked during preflight.
- `plugin_candidates[].requires_outputs` is currently documentation for plugin authors; the runtime does not yet validate per-plugin intermediate outputs.
- A pipeline declaration is not enough to succeed. After execution, the merged `EnvironmentDelta.generated_outputs` must cover every requested output or the session is rejected before environment state is written.

Supported model providers:

- `builtin_dummy`: dependency-free test/provider placeholder.
- `python_module`: preflight checks that `module` is importable.
- `local_checkpoint`: preflight checks that `checkpoint_path` exists.
- `local_worker` or `remote_endpoint`: preflight checks `healthcheck` or `endpoint` with a short HTTP request.

## Plugin Installation And Development

Plugins live under:

```text
PhyAgentOS/plugins/perception_plugins/
```

Built-in MVP plugins:

- `dummy_segmenter`: creates a compact 2D object from config.
- `rgbd_object_builder`: converts prior plugin objects into dependency-free test 3D objects.
- `sim_oracle`: debug-only plugin that writes configured oracle objects.
- `sam3_open_vocab`, `yolo_seg`: placeholders that show the lazy-loading pattern.

To add a plugin:

1. Create a module under `PhyAgentOS/plugins/perception_plugins/` or an importable external package.
2. Subclass `BasePerceptionPlugin`.
3. Keep module import light. Put heavy imports inside `run()` for the current MVP; `BasePerceptionPlugin.preflight()` exists as an extension hook but is not automatically called yet.
4. Return an `EnvironmentDelta`.
5. Reference the plugin in `plugin_candidates`.

Minimal plugin:

```python
from PhyAgentOS.plugins.perception_plugins.base import BasePerceptionPlugin
from PhyAgentOS.runtime.schemas.perception import (
    EnvironmentDelta,
    EnvironmentObject,
    EnvironmentObjectSource,
)


class MyDetector(BasePerceptionPlugin):
    def preflight(self) -> None:
        # Extension hook for future plugin-local preflight; not called automatically in the current MVP.
        pass

    def run(self, frame, context):
        obj = EnvironmentObject(
            label="box",
            confidence=0.9,
            source=EnvironmentObjectSource(
                target_id=frame.target_id,
                session_id=frame.session_id,
                source_plugin_id=self.plugin_id,
            ),
            pose={
                "frame_id": "map",
                "position_m": [1.0, 2.0, 0.0],
            },
        )
        return EnvironmentDelta(
            objects={"local_box_1": obj},
            generated_outputs=["objects_3d"],
            refresh_scope=["local_box_1"],
        )
```

External module declaration:

```yaml
plugin_candidates:
  - id: my_detector
    type: object_detector
    model_ref: my_model
    module: my_package.my_detector
    class_name: MyDetector
    requires_sensors: [front_rgb]
    produces: [objects_3d]
```

Install optional dependencies in the environment that runs the watchdog. For example:

```bash
pip install your-plugin-package
```

Then make sure the module can be imported by that Python environment:

```bash
python -c "import my_package.my_detector"
```

## ENVIRONMENT.md Output

Perception writes `PhyAgentOS.environment.v2` to the environment workspace.

Top-level fields are:

```yaml
schema_version: PhyAgentOS.environment.v2
updated_at: ...
targets: {}
objects: {}
scene_graph:
  relations: []
perception:
  runs: {}
map: {}
tf: {}
```

Legacy fields such as `runtime` and `robots` are not preserved by the v2 perception writer. Runtime session history is written to `LOG.md` in the runtime workspace instead.

### Object IDs And Global Anchoring

Plugins output local object ids in `EnvironmentDelta.objects`, such as:

```python
objects={"local_box_1": obj}
```

The environment writer converts local ids to global ids:

1. If the object has `pose.frame_id` and `pose.position_m`, writer searches existing objects with the same label and frame.
2. If distance is within `position_threshold_m`, default `0.25`, writer reuses that existing global id.
3. Otherwise writer creates a namespaced id:

```text
<target_id>::<local_object_id>
```

Each object records identity metadata:

```yaml
identity:
  global_id: target_a::local_box_1
  local_object_id: local_box_1
  anchor:
    frame_id: map
    position_m: [1.0, 2.0, 0.0]
  position_threshold_m: 0.25
```

This lets multiple targets share one `ENVIRONMENT.md` while reducing duplicate objects by global position.

### Refresh Scope

`refresh_scope` declares which local object ids are authoritative for the current perception refresh.

Current default behavior:

- If `EnvironmentDelta.scope` is provided, writer uses it.
- Else writer builds a scope from `EnvironmentDelta.refresh_scope`.
- Else writer scopes to all object ids in the delta.

Default scope:

```yaml
target_id: <target_id>
sensor_ids: []
frame_id: null
mode: authoritative_object_ids
object_ids: [...]
position_threshold_m: 0.25
```

Deletion rule:

- Only objects from the same `target_id` are considered.
- Only objects in the resolved refresh scope are considered.
- If a scoped object is not produced in the current delta, it is deleted.
- Other targets' objects are not deleted.

Relations are rewritten from local ids to global ids when possible. Relations referencing deleted or missing objects are removed.

## LOG.md

Runtime session summaries are written to:

```text
<runtime_workspace>/LOG.md
```

Format:

```yaml
version: runtime_session_history_v1
updated_at: ...
last_session_id: ...
last_target_id: ...
last_status: ...
last_success: ...
last_artifact_dir: ...
sessions:
  sess_001:
    session_id: sess_001
    target_id: dummy_sim
    status: succeeded
    success: true
    artifact_dir: artifacts/runtime/sess_001
    num_steps: 3
    return_value: 1.0
    updated_at: ...
```

`LOG.md` is a temporary runtime history file. It is intentionally separate from `ENVIRONMENT.md`.

## Strict Preflight Failure Behavior

If perception preflight fails:

- Session is marked `rejected`.
- Target execution does not enter `running`.
- Policy inference is not called.
- Skill runtime is not started.
- No perception objects are written to `ENVIRONMENT.md`.

The same failure behavior applies if a pipeline runs but does not actually generate all requested outputs.

Common rejection causes:

- Missing `sensor_config_ref`.
- Missing `perception_config_ref` when environment outputs are required.
- Required sensor missing or disabled.
- Required observation channel missing.
- Dtype or shape mismatch.
- Calibration file missing.
- Pipeline does not cover requested outputs.
- Pipeline declares but does not generate requested outputs.
- Plugin references missing sensors/plugins/models.
- Python module, checkpoint, or model endpoint unavailable.

## Smoke Setup

Initialize a runtime workspace:

```bash
python scripts/init_runtime_workspace.py --workspace /tmp/paos_runtime
```

The template includes dependency-free dummy perception configs:

```text
configs/runtime/sensors/dummy_sim.sensors.yaml
configs/runtime/perception/dummy_sim.perception.yaml
```

By default `dummy_sim` has `perception.enabled: false` and the default skill has no perception requirements. To exercise perception:

1. Set `TARGETS.md` `targets[dummy_sim].perception.enabled: true`.
2. Add required sensors and outputs to `SKILLS.md`, for example:

```yaml
requires:
  sensors: [front_rgb, proprio]
  environment_outputs: [objects_3d, scene_graph]
  strict_environment_contract: true
```

3. Run the watchdog:

```bash
python scripts/run_runtime_watchdog.py \
  --workspace /tmp/paos_runtime \
  --environment-workspace /tmp/paos_shared \
  --once
```

Expected outputs:

```text
/tmp/paos_runtime/LOG.md
/tmp/paos_runtime/artifacts/runtime/<session_id>/episode.json
/tmp/paos_shared/ENVIRONMENT.md
```
