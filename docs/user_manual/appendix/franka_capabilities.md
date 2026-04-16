# Franka 后端能力支持表

> Franka Backend Capabilities Matrix

本文档详细列出 `FrankaMultiBackendDriver` 各后端的能力支持情况。

详细信息请参考官方文档与网站

---

## 1. 后端能力总览

| 能力分类 | pylibfranka | franky-control | ROS2 (预留) |
|:---------|:-----------:|:--------------:|:-----------:|
| **基础运动** | | | |
| 关节位置控制 | ✅ | ✅ | ✅ |
| 笛卡尔位置控制 | ✅ | ✅ | ✅ |
| 关节速度控制 | ✅ | ⚠️ 有限 | ✅ |
| 笛卡尔速度控制 | ✅ | ⚠️ 有限 | ✅ |
| **高级运动** | | | |
| 力控 / Force Control | ✅ | ❌ | ⚠️ |
| 阻抗控制 / Impedance Control | ✅ | ❌ | ⚠️ |
| 柔顺控制 / Compliance Control | ✅ | ❌ | ⚠️ |
| **实时控制** | | | |
| 实时 1kHz | ✅ | ❌ | ❌ |
| 实时 250Hz | ✅ | ✅ | ⚠️ |
| **夹爪控制** | | | |
| 夹爪开/关 | ✅ | ✅ | ✅ |
| 力控抓取 | ✅ | ⚠️ 有限 | ✅ |
| 位置控制 | ✅ | ✅ | ✅ |
| **安全功能** | | | |
| 碰撞检测 | ✅ | ✅ | ✅ |
| 碰撞响应 | ✅ | ✅ | ⚠️ |
| 自碰撞检测 | ✅ | ⚠️ | ⚠️ |
| 速度限制 | ✅ | ✅ | ✅ |
| 力矩限制 | ✅ | ⚠️ | ⚠️ |
| **连接管理** | | | |
| 自动重连 | ✅ | ✅ | ⚠️ |
| 心跳检测 | ✅ | ⚠️ | ⚠️ |
| 状态监控 | ✅ | ✅ | ✅ |
| **运动规划** | | | |
| 梯形轨迹 | ⚠️ 手动 | ✅ | ⚠️ |
| 最小jerk轨迹 | ⚠️ 手动 | ✅ | ⚠️ |
| 梯形速度曲线 | ⚠️ 手动 | ✅ | ⚠️ |
| 在线轨迹修改 | ⚠️ 高级 | ⚠️ | ⚠️ |

**图例**:
- ✅ 完全支持
- ⚠️ 部分支持 / 有限支持
- ❌ 不支持

---

## 2. 能力详细说明

### 2.1 运动控制能力

#### 关节位置控制 (Joint Position Control)

控制每个关节的角度位置。

```python
# 所有后端都支持
driver.execute_action("move_joints", {
    "joints": {"q1": 0.0, "q2": -0.5, "q3": 0.0, "q4": -1.0, "q5": 0.0, "q6": 1.0, "q7": 0.0}
})
```

| 后端 | 精度 | 速度控制 | 平滑过渡 |
|:-----|:----:|:--------:|:---------:|
| pylibfranka | ±0.1mm | ✅ | 手动实现 |
| franky | ±0.1mm | ✅ | 内置最小jerk |

#### 笛卡尔位置控制 (Cartesian Position Control)

控制末端执行器的位置和姿态。

```python
# 所有后端都支持
driver.execute_action("move_to", {
    "x": 0.3, "y": 0.2, "z": 0.5,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0
})
```

| 后端 | 位置精度 | 姿态控制 | 奇异性处理 |
|:-----|:--------:|:--------:|:----------:|
| pylibfranka | ±0.1mm | 完全 | 手动避免 |
| franky | ±0.1mm | 完全 | 自动处理 |

### 2.2 高级运动控制

#### 力控 (Force Control)

在指定方向上控制末端力的大小。

```python
# 仅 pylibfranka 支持
# franky 不支持力控模式
```

**pylibfranka 力控模式**:
- 阻抗模式
- 力矩控制模式
- 混合控制模式

#### 阻抗控制 (Impedance Control)

调整机械臂的柔顺性，模拟弹簧阻尼系统。

| 后端 | 关节阻抗 | 笛卡尔阻抗 | 可变刚度 |
|:-----|:--------:|:-----------:|:--------:|
| pylibfranka | ✅ | ✅ | ⚠️ 有限 |
| franky | ❌ | ❌ | ❌ |

### 2.3 实时控制

| 后端 | 1kHz (实时内核) | 250Hz (普通内核) | 100Hz |
|:-----|:---------------:|:----------------:|:------:|
| pylibfranka | ✅ (需RT内核) | ✅ | ✅ |
| franky | ❌ | ✅ | ✅ |

**注意**: 1kHz 实时控制需要安装 PREEMPT_RT 实时内核。

### 2.4 夹爪控制

| 功能 | pylibfranka | franky-control |
|:-----|:-----------:|:--------------:|
| `grasp` 抓取 | ✅ | ✅ |
| `move_gripper` 移动 | ✅ | ✅ |
| `homing` 回零 | ✅ | ✅ |
| 力反馈 | ✅ | ⚠️ 有限 |
| 速度控制 | ✅ | ✅ |

### 2.5 安全功能

| 安全功能 | pylibfranka | franky-control |
|:---------|:-----------:|:--------------:|
| 碰撞检测 | ✅ | ✅ |
| 碰撞后停止 | ✅ | ✅ |
| 碰撞后释放 | ✅ | ✅ |
| 关节限位 | ✅ | ✅ |
| 自碰撞检测 | ✅ | ⚠️ |
| 末端碰撞检测 | ✅ | ⚠️ |

---

## 3. 后端选择指南

### 根据需求选择

| 需求场景 | 推荐后端 | 原因 |
|:---------|:--------|:-----|
| 基础运动控制 | franky | API 简单，轨迹平滑 |
| 力控/阻抗控制 | pylibfranka | 完整功能支持 |
| 实时 1kHz 控制 | pylibfranka | 唯一支持 |
| 快速原型开发 | franky | 更简单的 API |
| Robot System 9 兼容性 | franky | 更宽松的版本要求 |

### 根据 Robot System Version 选择

| Robot System Version | 推荐后端 | 备选 |
|:--------------------|:---------|:-----|
| 5.2 ~ 5.5 | franky | pylibfranka (需版本匹配) |
| 5.5 ~ 5.7 | franky 或 pylibfranka | franky |
| 5.7 ~ 5.9 | franky 或 pylibfranka | franky (更宽松) |
| >= 5.9 | pylibfranka (最新) | franky |

### 根据经验水平选择

| 用户类型 | 推荐后端 | 学习曲线 |
|:---------|:--------|:---------|
| 初学者 | franky | 平缓 |
| 有 libfranka 经验 | pylibfranka | N/A |
| 需要精确控制 | pylibfranka | 陡峭 |
| 快速原型 | franky | 平缓 |

---

## 4. 性能对比

### 运动执行速度 (参考值)

| 操作 | pylibfranka | franky |
|:-----|:-----------:|:------:|
| 关节运动 90° | ~2-3s | ~2-3s |
| 笛卡尔直线运动 0.3m | ~2-3s | ~2-3s |
| 夹爪开/闭 | ~1s | ~1s |
| 状态读取 | ~4ms | ~4ms |

### 轨迹平滑度

| 后端 | 最小jerk | 梯形 | 手动 |
|:-----|:--------:|:----:|:----:|
| pylibfranka | ⚠️ 需手动 | ⚠️ 需手动 | ✅ |
| franky | ✅ | ✅ | N/A |

---

## 5. API 对比

### 连接

```python
# pylibfranka
driver = FrankaDriver(ip="172.16.0.2")
driver.connect()

# franky
driver = FrankaMultiBackendDriver(ip="172.16.0.2", backend_priority=["franky"])
driver.connect()  # 自动选择 franky 后端
```

### 关节运动

```python
# pylibfranka
driver.move_joints([0, -0.5, 0, -1.0, 0, 1.0, 0])

# franky
driver.move_joints({"q1": 0, "q2": -0.5, "q3": 0, "q4": -1.0, "q5": 0, "q6": 1.0, "q7": 0})
```

### 笛卡尔运动

```python
# pylibfranka
driver.move_to(x=0.3, y=0.2, z=0.5, roll=0, pitch=0, yaw=0)

# franky
driver.move_to(x=0.3, y=0.2, z=0.5, roll=0, pitch=0, yaw=0)
```

---

*最后更新: 2026-04-11*
