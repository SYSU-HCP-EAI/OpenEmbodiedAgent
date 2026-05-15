# Runtime Targets

```yaml
version: runtime_target_registry_v1
targets:
  - id: dummy_sim
    type: sim
    backend: dummy
    enabled: true
    workspace: workspaces/dummy_sim
    supported_skills:
      - openpi_sim_vla
    adapter: dummy_openpi_adapter
    perception:
      enabled: false
      strict_preflight: true
      sensor_config_ref: configs/runtime/sensors/dummy_sim.sensors.yaml
      perception_config_ref: configs/runtime/perception/dummy_sim.perception.yaml
      artifact_dir: artifacts/perception/dummy_sim
    config:
      success_after_steps: 3
      observation:
        image_size: 16
        state_dim: 8
      action:
        action_dim: 7
        chunk_size: 4
```
