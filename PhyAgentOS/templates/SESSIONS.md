# Runtime Sessions

```yaml
version: runtime_sessions_v1
sessions:
  - session_id: sess_dummy_smoke
    goal_id: goal_runtime_smoke
    target_ref: target://dummy_sim
    skill_ref: skill://openpi_sim_vla
    task_description: move the object to the target
    status: pending
    priority: normal
    timeouts:
      queue_timeout_s: 30
      preflight_timeout_s: 20
      execute_timeout_s: 120
      policy_timeout_s: 5
    retry:
      max_retries: 0
      attempted: 0
    routing:
      target_endpoint: null
      policy_endpoint: dummy://local
      adapter_resolution: strict_auto
      adapter_overrides: null
    execution:
      max_steps: 10
      replan_every_steps: 4
      action_chunk_mode: chunk_buffer
      chunk_switch_mode: hard_switch
    runtime_hints:
      perception_queries: []
      force_environment_refresh: false
      preferred_replan_every_steps: 4
    safety_profile:
      profile: default_simulation
      workspace_bounds: default
      stop_on_policy_timeout: true
    result: {}
```
