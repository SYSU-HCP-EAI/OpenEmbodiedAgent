# Franka 机器人兼容性表格

> Franka Robot Compatibility Matrix

本文档提供 Franka Research 3 (FR3) 与不同软件包的版本兼容性粗略信息。

详细信息请参考官方文档与网站。

---

## 1. Robot System Version 与 libfranka/pylibfranka 兼容性

根据 Franka 官方文档整理([参考资料](https://frankarobotics.github.io/docs/compatibility.html))：

| Robot System Version | libfranka 版本要求 | Robot/Gripper Server | Ubuntu | ROS2 Humble | ROS2 Jazzy |
|:-------------------:|:------------------:|:--------------------:|:------:|:-----------:|:----------:|
| **>= 5.9.0** | >= 0.18.0 | 10 / 3 | 22.04 | v2.0.2 ~ v3.0.0 | >= v3.0.0 |
| **>= 5.7.2** | >= 0.15.0 | **9 / 3** | 22.04 | v2.0.2 ~ v3.0.0 | >= v3.0.0 |
| **>= 5.7.0** | 0.14.1 ~ 0.15.0 | 8 / 3 | 22.04 | v0.1.15 ~ v2.0.0 | N/A |
| **>= 5.5.0** | 0.13.3 ~ 0.14.1 | 7 / 3 | 22.04 | v0.1.15 ~ v2.0.0 | N/A |
| **>= 5.2.0** | 0.10.0 ~ 0.13.3 | 6 / 3 | 22.04 | v0.1.0 ~ v0.1.8 | N/A |

### 如何查看 Robot System Version

1. **通过 Desk 界面**: 登录 Control Box Desk → 左侧菜单 → About
2. **通过机器人标签**: 机器人底座上的铭牌
3. **通过 API**: 连接后读取机器人固件信息

---

## 2. FrankaMultiBackendDriver 后端兼容性

`FrankaMultiBackendDriver` 支持多个后端实现，每个后端有不同的版本要求：

### 2.1 pylibfranka 后端

从 libfranka 0.20.2 以上才开始有 pylibfranka

**安装命令**:
```bash
pip install pylibfranka==0.21.1  # 指定版本
pip install pylibfranka           # 最新版本
```

### 2.2 franky-control 后端

`franky-control` (github.com/TimSchneider42/franky) 是一个基于 libfranka 的高层运动库。

**安装命令**:
```bash
# 从 GitHub 安装最新版
pip install git+https://github.com/TimSchneider42/franky.git

# 指定版本 (如果版本标签可用)
pip install git+https://github.com/TimSchneider42/franky.git@v1.1.3
```

---

## 3. 后端能力对比

| 能力 / Capability | pylibfranka | franky-control |
|:------------------|:-----------:|:--------------:|
| 关节位置控制 / Joint Position | ✅ | ✅ |
| 笛卡尔位置控制 / Cartesian Position | ✅ | ✅ |
| 夹爪控制 / Gripper Control | ✅ | ✅ |
| 力控 / Force Control | ✅ | ❌ |
| 阻抗控制 / Impedance Control | ✅ | ❌ |
| 实时控制 1kHz / Realtime 1kHz | ✅ | ❌ |
| 碰撞检测 / Collision Detection | ✅ | ✅ |
| 自动重连 / Auto Reconnect | ✅ | ✅ |
| 运动平滑 / Motion Smoothing | 手动实现 | 内置 |

---

## 4. 推荐的安装组合

### 场景 A: Robot System Version 9 (你的环境)

```bash
# 推荐组合
pip install pylibfranka==0.21.1  # 官方 Python 绑定
pip install franky-control        # 备选高层库 (如果有需求)
```

### 场景 B: Robot System Version 5.7.x ~ 5.9.x

```bash
# 使用兼容版本
pip install pylibfranka==0.18.0  # 兼容 Robot System 5.9
```

### 场景 C: 需要实时控制 (1kHz)

```bash
# 实时控制需要特定配置
pip install pylibfranka==0.21.1
# + 需要安装实时内核
```

---

## 5. 故障排除

### 5.1 "Incompatible library version" 错误

```
libfranka: Incompatible library version (server version: 6, library version: 5)
```

**解决方案**:
1. 升级 pylibfranka: `pip install --upgrade pylibfranka`
2. 或降级 pylibfranka 匹配机器人固件版本
3. 或使用 franky-control 作为替代

### 5.2 "FCI not activated" 错误

**解决方案**:
1. 登录 Control Box Desk 界面
2. 左侧菜单 → Activate FCI
3. 确认 FCI 状态为 "Active"

### 5.3 连接超时

**检查清单**:
- [ ] 物理网线连接 Control Box
- [ ] 控制 PC IP 设置为 172.16.0.x (同一网段)
- [ ] Control Box 已上电
- [ ] 防火墙允许 7512 端口

---

## 6. 获取帮助

- **Franka 官方文档**: https://frankaemika.github.io/docs/
- **libfranka 兼容性表**: https://frankaemika.github.io/docs/compatibility.html
- **franky-control GitHub**: https://github.com/TimSchneider42/franky
- **PhyAgentOS 文档**: /docs/user_manual/

---

*最后更新: 2026-04-11*
