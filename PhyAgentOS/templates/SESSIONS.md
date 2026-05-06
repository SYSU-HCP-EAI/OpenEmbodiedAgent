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
      execute_timeout_s: 120
      policy_timeout_s: 5
    retry:
      max_retries: 0
      attempted: 0
    routing:
      policy_endpoint: dummy://local
      adapter: dummy_openpi_adapter
    execution:
      max_steps: 10
      replan_every: 4
      action_chunk_mode: open_loop
    result: {}
```
