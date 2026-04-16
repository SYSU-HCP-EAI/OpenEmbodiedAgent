# Franka Research 3 驱动配置说明

> Franka Research 3 Driver Configuration Reference

配置文件：`examples/franka_research3.driver.json`

---

## 参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ip` | string | `"172.16.0.2"` | Control Box IP 地址 |
| `robot_id` | string | `"franka_research3_001"` | 机器人逻辑标识符 |
| `control_rate` | float | `250.0` | 控制频率 (Hz), 范围 1~1000 |
| `realtime_mode` | boolean | `false` | 是否启用实时模式 (1kHz) |
| `default_gripper_force` | float | `20.0` | 默认夹爪力 (N), 范围 0~70 |
| `safe_max_linear_m_s` | float | `0.5` | 最大线速度 (m/s) |
| `safe_max_angular_deg_s` | float | `30.0` | 最大角速度 (deg/s) |
| `reconnect_policy` | string | `"auto"` | 重连策略: `"auto"` 或 `"manual"` |
| `auto_discover` | boolean | `true` | 是否自动发现机器人 IP |
| `backend_priority` | array | `["franky", "pylibfranka"]` | 后端优先级列表 |
| `force_backend` | string/null | `null` | 强制使用特定后端 |

---

## 详细说明

### 网络配置

**ip**
- Control Box 的 Shop Floor 接口 IP
- 不是机器人本身的 IP
- 示例: `"172.16.0.2"`

**auto_discover**
- 为 `true` 时，如果未指定 `ip` 或 `ip` 无效，会尝试以下已知 IP:
  - `172.16.0.2` (用户配置)
  - `192.168.1.1` (Franka 默认)
  - `172.16.0.1` (备用)
  - `192.168.0.1` (机器人网络)

### 机器人标识

**robot_id**
- 用于运行时状态和日志标识
- 多机器人时需要唯一

### 控制参数

**control_rate**
- 控制命令发送频率
- 默认 250Hz
- 可配置到 1000Hz (需要实时内核)

**realtime_mode**
- `false`: kIgnore 模式，开发友好
- `true`: kEnforce 模式，1kHz 实时控制，需要 PREEMPT_RT 内核

### 安全限制

**safe_max_linear_m_s**
- 末端执行器最大线速度
- 默认 0.5 m/s

**safe_max_angular_deg_s**
- 末端执行器最大角速度
- 默认 30 deg/s

**default_gripper_force**
- 抓取动作的默认力
- 范围 0~70N

### 连接管理

**reconnect_policy**
- `"auto"`: 连接断开后自动重连
- `"manual"`: 需要手动调用 connect_robot

### 后端协商 (franka_multi 驱动)

**backend_priority**
- 连接时尝试后端的顺序
- 可选值:
  - `"franky"`: franky-control 高层库，更宽松的兼容性
  - `"pylibfranka"`: 官方 pylibfranka，完整功能

**force_backend**
- `null`: 自动协商
- `"franky"`: 强制使用 franky-control
- `"pylibfranka"`: 强制使用 pylibfranka

---

## 环境变量

以下参数可以通过环境变量覆盖配置文件：

| 环境变量 | 对应参数 |
|----------|----------|
| `PAOS_FRANKA_IP` | `ip` |
| `PAOS_FRANKA_CONTROL_RATE` | `control_rate` |
| `PAOS_FRANKA_ROBOT_ID` | `robot_id` |
| `PAOS_FRANKA_GRIPPER_FORCE` | `default_gripper_force` |
| `PAOS_FRANKA_SAFE_MAX_LINEAR_M_S` | `safe_max_linear_m_s` |
| `PAOS_FRANKA_SAFE_MAX_ANGULAR_DEG_S` | `safe_max_angular_deg_s` |
| `PAOS_FRANKA_BACKEND_PRIORITY` | `backend_priority` (逗号分隔) |
| `PAOS_FRANKA_FORCE_BACKEND` | `force_backend` |

---

## 使用示例

```bash
# 基本使用
python hal/hal_watchdog.py --driver franka_multi --driver-config examples/franka_research3.driver.json

# 强制使用 pylibfranka 后端
python hal/hal_watchdog.py --driver franka_multi --driver-config examples/franka_research3.driver.json

# 通过环境变量覆盖
export PAOS_FRANKA_IP=192.168.1.50
export PAOS_FRANKA_BACKEND_PRIORITY=pylibfranka,franky
python hal/hal_watchdog.py --driver franka_multi
```

---

## 碰撞阈值 (高级选项)

如需自定义碰撞检测阈值，可以在配置中添加：

```json
{
  "collision_thresholds": {
    "lower_torque": [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
    "upper_torque": [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
    "lower_force": [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    "upper_force": [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
  }
}
```

⚠️ **注意**: 错误配置可能导致安全问题，建议使用默认值。