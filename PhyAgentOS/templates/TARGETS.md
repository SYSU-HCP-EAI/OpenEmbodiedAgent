# Runtime Targets

```yaml
version: runtime_target_registry_v1
targets:
  - id: dummy_sim
    target_class: local
    target_kind: simulation
    enabled: true
    workspace: workspaces/dummy_sim
    supported_skills:
      - openpi_sim_vla
    runtime:
      target_runtime: DummySimTargetRuntime
      target_endpoint: null
      target_adapter: target_adapter://dummy_sim_adapter
      runtime_contract_ref: configs/runtime/contracts/dummy_sim.runtime.yaml
    observation:
      observation_type: multimodal
      empty_observation_allowed: false
    perception:
      enabled: true
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
        id: dummy_delta_eef_gripper_v1
        action_dim: 7
        chunk_size: 4
        max_chunk_size: 4
```
