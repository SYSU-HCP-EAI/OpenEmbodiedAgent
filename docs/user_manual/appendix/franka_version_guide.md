# Franka 版本升级/降级指南

> Franka Version Upgrade/Downgrade Guide

本文档提供 Franka 相关软件包的版本管理和故障排查粗略指南。

详细信息请参考官方文档以及网站。

---

## 1. 版本检查

### 1.1 检查 pylibfranka 版本

```bash
# 方法1: pip show
pip show pylibfranka

# 方法2: Python 代码
python3 -c "import pylibfranka; print(pylibfranka.__version__)"

# 方法3: 在 Python 中
python3 -c "import pylibfranka; print(dir(pylibfranka))"
```

### 1.2 检查 Robot System Version

```python
# 通过 FrankaDriver 连接后检查
from hal.drivers import load_driver

driver = load_driver("franka_research3", ip="172.16.0.2")
result = driver.execute_action("connect_robot", {})
print(result)

# 通过 get_robot_state 查看详细信息
state = driver.execute_action("get_robot_state", {})
print(state)
```

### 1.3 检查 franky-control 版本

```bash
# 方法1: pip show
pip show franky-control

# 方法2: 检查 Git 提交
pip show -f franky-control | grep -i commit
```

---

## 2. pylibfranka 版本管理

### 2.1 安装指定版本

```bash
# 安装特定版本
pip install pylibfranka==0.21.1

# 安装特定范围版本
pip install "pylibfranka>=0.18.0,<0.22.0"

# 从源码安装
git clone https://github.com/frankaemika/libfranka.git
cd libfranka
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
sudo ldconfig
pip install --force-reinstall pylibfranka
```

### 2.2 升级 pylibfranka

```bash
# 升级到最新版本
pip install --upgrade pylibfranka

# 升级到特定版本
pip install --upgrade pylibfranka==0.21.1
```

### 2.3 降级 pylibfranka

```bash
# 如果 Robot System Version 9 不兼容 0.21.x
# 降级到 0.19.x 或 0.18.x

pip install --force-reinstall pylibfranka==0.19.0

# 验证版本
pip show pylibfranka | grep Version
```

### 2.4 完全卸载重装

```bash
# 卸载
pip uninstall pylibfranka -y

# 清理缓存
pip cache purge

# 重新安装
pip install pylibfranka
```

---

## 3. franky-control 版本管理

### 3.1 安装 franky-control

```bash
# 从 GitHub 安装最新版
pip install git+https://github.com/TimSchneider42/franky.git

# 安装特定版本 (如果版本标签可用)
pip install git+https://github.com/TimSchneider42/franky.git@v1.1.3

# 安装到指定目录 (开发模式)
git clone https://github.com/TimSchneider42/franky.git
cd franky
pip install -e .
```

### 3.2 安装特定版本

可参照 `hal/drivers/franka_backends/setup_utils/franky_install.sh` 的安装脚本模板来编写安装对应特定版本 libfranka 的 franky-control。 详细文档见 [franky 官方文档](https://github.com/TimSchneider42/franky?tab=readme-ov-file#installing-franky)。

### 3.3 更新 franky-control

```bash
# 更新到最新版本
pip install --force-reinstall git+https://github.com/TimSchneider42/franky.git

# 或者 cd 到源码目录
cd franky
git pull
pip install -e .
```

### 3.4 卸载 franky-control

```bash
pip uninstall franky-control -y
```

---

## 4. 多后端环境配置

### 4.1 同时安装两个后端

```bash
# 安装 pylibfranka
pip install pylibfranka

# 安装 franky-control
pip install git+https://github.com/TimSchneider42/franky.git
```

### 4.2 配置后端优先级

通过环境变量:
```bash
# 设置后端优先级
export PAOS_FRANKA_BACKEND_PRIORITY="franky,pylibfranka"

# 或者在 Python 代码中
from hal.drivers import load_driver

driver = load_driver(
    "franka_multi",
    backend_priority=["franky", "pylibfranka"],
    force_backend=None  # 自动协商
)
```

### 4.3 强制使用特定后端

```bash
# 使用环境变量
export PAOS_FRANKA_FORCE_BACKEND="pylibfranka"

# 或在代码中
driver = load_driver(
    "franka_multi",
    force_backend="pylibfranka"  # 强制使用 pylibfranka
)
```

---

## 5. 常见错误与解决方案

### 5.1 "Incompatible library version" 错误

```
错误信息:
libfranka: Incompatible library version (server version: 6, library version: 5)
```

**原因**: pylibfranka 版本与机器人固件版本不匹配。

**解决方案**:

| Robot System Version | 解决方案 |
|:--------------------|:---------|
| 9 | 升级 pylibfranka 到 >= 0.19.0 |
| 8 | 降级到 pylibfranka 0.14.x ~ 0.15.x |
| 7 | 降级到 pylibfranka 0.13.x ~ 0.14.x |

**操作步骤**:

```bash
# 1. 确认 Robot System Version
# (见 1.2 节)

# 2. 根据版本选择合适的 pylibfranka
# Robot System 9:
pip install --force-reinstall pylibfranka==0.19.0

# Robot System 8:
pip install --force-reinstall "pylibfranka>=0.14.1,<0.15.0"

# 3. 验证安装
python3 -c "import pylibfranka; print(pylibfranka.__version__)"

# 4. 重新连接
```

### 5.2 "pylibfranka not installed" 错误

**原因**: pylibfranka 未安装或安装损坏。

**解决方案**:

```bash
# 重新安装 pylibfranka
pip uninstall pylibfranka -y
pip cache purge
pip install pylibfranka

# 验证导入
python3 -c "import pylibfranka; print('OK')"
```

### 5.3 "franky-control not installed" 错误

**原因**: franky-control 未安装。

**解决方案**:

```bash
# 安装 franky-control
pip install git+https://github.com/TimSchneider42/franky.git

# 验证安装
python3 -c "import franky; print('OK')"
```

### 5.4 连接超时错误

```
错误信息:
Connection timeout. Check network configuration.
```

**原因**: 网络连接问题。

**解决方案**:

1. **检查物理连接**:
   - 网线连接 Control Box
   - 网线连接控制 PC

2. **检查 IP 配置**:
   ```bash
   # 控制 PC 应该设置为 172.16.0.x
   ip addr show | grep 172.16.0

   # 例如设置静态 IP
   sudo ip addr add 172.16.0.1/24 dev eth0
   ```

3. **测试网络连通性**:
   ```bash
   ping 172.16.0.2  # Control Box IP

   # 测试端口
   nc -zv 172.16.0.2 7512
   ```

4. **检查防火墙**:
   ```bash
   sudo ufw allow 7512/tcp
   ```

### 5.5 "FCI not activated" 错误

**原因**: Franka Control Interface (FCI) 未在 Desk 界面激活。

**解决方案**:

1. 打开浏览器，访问 `https://172.16.0.2` (Control Box Desk)
2. 登录 (默认用户名: admin)
3. 左侧菜单 → **Activate FCI**
4. 确认 FCI 状态为 **Active** (绿色)

---

## 6. 环境隔离 (推荐)

### 6.1 使用 conda 环境

```bash
# 创建专用环境
conda create -n franka python=3.11
conda activate franka

# 安装依赖
pip install pylibfranka
pip install git+https://github.com/TimSchneider42/franky.git

# 验证
python3 -c "import pylibfranka; import franky; print('OK')"
```

### 6.2 使用虚拟环境

```bash
# 创建虚拟环境
python3 -m venv franka_env
source franka_env/bin/activate

# 安装
pip install pylibfranka
pip install git+https://github.com/TimSchneider42/franky.git
```

---

## 7. 版本兼容性快速参考表

| Robot System | pylibfranka | franky-control | 备注 |
|:------------:|:-----------:|:--------------:|:-----|
| 9 | >= 0.19.0 | 1.1.x | 推荐 franky 作为首选 |
| 8 | 0.14.1 ~ 0.15.0 | 1.0.x | franky 更宽松 |
| 7 | 0.13.3 ~ 0.14.1 | 1.0.x | franky 更宽松 |
| 6 | 0.10.0 ~ 0.13.3 | N/A | 较旧版本 |

---

## 8. 获取帮助

- **PhyAgentOS GitHub Issues**: https://github.com/your-repo/PhyAgentOS/issues
- **Franka 官方文档**: https://frankaemika.github.io/docs/
- **libfranka GitHub**: https://github.com/frankaemika/libfranka
- **franky GitHub**: https://github.com/TimSchneider42/franky

---

*最后更新: 2026-04-11*
