# Runtime Skills

```yaml
version: runtime_skill_registry_v1
skills:
  - id: openpi_sim_vla
    category: vla
    runtime: OpenPISimSkillRuntime
    supported_target_types:
      - sim
    policy_client: dummy
    supports_chunk: true
    default_replan_every: 4
    requires:
      sensors: []
      environment_outputs: []
      strict_environment_contract: true
    input_contract:
      images:
        - observation/image
        - observation/wrist_image
      state: observation/state
      prompt: prompt
    output_contract:
      actions: actions
      shape:
        - T
        - A
```
