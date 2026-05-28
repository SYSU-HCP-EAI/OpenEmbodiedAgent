# Runtime Skills

```yaml
version: runtime_skill_registry_v1
skills:
  - id: openpi_sim_vla
    runtime: OpenPISkillRuntime
    runtime_kind: policy
    loop_mode: policy_closed_loop
    agent_exposure: none
    supported_target_kinds:
      - simulation
    policy:
      policy_client: dummy
      policy_adapter: policy_adapter://dummy_openpi_adapter
      supports_chunk: true
    observation_contract:
      observation_type: multimodal
      empty_observation_allowed: false
    supports_chunk: true
    default_replan_every: 4
    requires:
      sensors:
        - front_rgb
        - wrist_rgb
        - proprio
      environment_outputs: []
      strict_environment_contract: true
    input_contract:
      images:
        - observation/image
        - observation/wrist_image
      state: observation/state
      prompt: prompt
    output_contract:
      action:
        action_space_id: dummy_policy_delta_eef_gripper_v1
        tensor_key: actions
        shape:
          - T
          - 7
        dtype: float32
        normalized: false
        representation: delta_eef_pose_gripper
        frame: base
        chunk:
          variable_T: true
          default_T: 4
          policy_hz: 20
    adapter_requirements:
      allowed_bridges:
        - bridge://safety_clamp
      forbidden:
        - implicit_shape_truncation
        - implicit_representation_cast
```
