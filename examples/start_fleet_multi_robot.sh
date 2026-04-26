#!/bin/bash
#
# examples/start_fleet_multi_robot.sh
#
# 三机器人协同演示启动脚本 (PiperGo2 + Franka + G1)
# Fleet模式: 3个hal_watchdog进程 + 1个PAOS agent
#
# 使用方法:
#   bash examples/start_fleet_multi_robot.sh
#
# 前置条件:
#   1. conda 环境已激活 (paos_zhongqijun)
#   2. InternUtopia 已安装并配置 pythonpath
#   3. scene_asset_path 指向有效的 USD 文件
#

set -e

# 配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FLEET_CONFIG="$SCRIPT_DIR/fleet_multi_robot_demo.json"

PIPERGO2_DRIVER_CONFIG="$SCRIPT_DIR/pipergo2_manipulation_driver.json"
FRANKA_DRIVER_CONFIG="$SCRIPT_DIR/franka_simulation_driver.json"
G1_DRIVER_CONFIG="$SCRIPT_DIR/g1_simulation_driver.json"

# 工作目录
WORKSPACE_BASE="/tmp/paos_fleet_multi_robot"
PIPERGO2_WS="$WORKSPACE_BASE/pipergo2_001"
FRANKA_WS="$WORKSPACE_BASE/franka_001"
G1_WS="$WORKSPACE_BASE/g1_001"
AGENT_WS="$WORKSPACE_BASE/agent_workspace"
SHARED_WS="$WORKSPACE_BASE/shared"

echo "=========================================="
echo "PhyAgentOS 三机器人协同演示启动脚本"
echo "=========================================="
echo ""
echo "配置: $FLEET_CONFIG"
echo "Workspace: $WORKSPACE_BASE"
echo ""

# 创建工作目录
echo "[1/5] 创建工作目录..."
mkdir -p "$PIPERGO2_WS"
mkdir -p "$FRANKA_WS"
mkdir -p "$G1_WS"
mkdir -p "$AGENT_WS"
mkdir -p "$SHARED_WS"
echo "      完成."

# 清理旧的日志
echo "[2/5] 清理旧日志..."
rm -rf "$WORKSPACE_BASE/logs" 2>/dev/null || true
mkdir -p "$WORKSPACE_BASE/logs"
echo "      完成."

# 检查配置文件
echo "[3/5] 检查配置文件..."
if [[ ! -f "$PIPERGO2_DRIVER_CONFIG" ]]; then
    echo "      警告: $PIPERGO2_DRIVER_CONFIG 不存在，将使用默认配置"
fi
if [[ ! -f "$FRANKA_DRIVER_CONFIG" ]]; then
    echo "      警告: $FRANKA_DRIVER_CONFIG 不存在，将使用默认配置"
fi
if [[ ! -f "$G1_DRIVER_CONFIG" ]]; then
    echo "      警告: $G1_DRIVER_CONFIG 不存在，将使用默认配置"
fi
echo "      完成."

# 创建 ENVIRONMENT.md (共享)
echo "[4/5] 创建共享 ENVIRONMENT.md..."
cat > "$SHARED_WS/ENVIRONMENT.md" << 'EOF'
# Fleet Multi-Robot Environment

## Robots

| Robot | Type | Driver | Status |
|-------|------|--------|--------|
| pipergo2_001 | Mobile Manipulator | pipergo2_manipulation | - |
| franka_001 | Fixed Arm | franka_simulation | - |
| g1_001 | Humanoid Locomotion | g1_simulation | - |

## Scene

- **Location**: Isaac Sim (internutopia sim stack)
- **Scene Asset**: merom_scene_baked.usd

## Coordination

所有机器人共享此 ENVIRONMENT.md 进行协调。
每个机器人有独立的 ACTION.md 和状态文件。

## 任务说明

本演示为三机器人协同场景:
1. PiperGo2 移动到 staging 位置
2. Franka 执行抓取/放置任务
3. G1 人形机器人进行移动演示
EOF
echo "      完成."

# 启动说明
echo "[5/5] 启动说明..."
echo ""
echo "=========================================="
echo "请在三个独立的终端中分别执行以下命令:"
echo ""
echo "终端 1 - PiperGo2:"
echo "  cd $PROJECT_ROOT"
echo "  conda activate paos_zhongqijun"
echo "  python hal/hal_watchdog.py --driver pipergo2_manipulation --robot-id pipergo2_001 --workspace $PIPERGO2_WS --gui"
echo ""
echo "终端 2 - Franka:"
echo "  cd $PROJECT_ROOT"
echo "  conda activate paos_zhongqijun"
echo "  python hal/hal_watchdog.py --driver franka_simulation --robot-id franka_001 --workspace $FRANKA_WS --gui"
echo ""
echo "终端 3 - G1:"
echo "  cd $PROJECT_ROOT"
echo "  conda activate paos_zhongqijun"
echo "  python hal/hal_watchdog.py --driver g1_simulation --robot-id g1_001 --workspace $G1_WS --gui"
echo ""
echo "终端 4 - PAOS Agent:"
echo "  cd $PROJECT_ROOT"
echo "  conda activate paos_zhongqijun"
echo "  paos onboard --workspace $AGENT_WS --config $FLEET_CONFIG"
echo ""
echo "=========================================="
echo ""

# 询问是否立即启动
read -p "是否立即启动所有watchdog进程? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "启动 PiperGo2 watchdog..."
    cd "$PROJECT_ROOT"
    conda activate paos_zhongqijun
    python hal/hal_watchdog.py \
        --driver pipergo2_manipulation \
        --robot-id pipergo2_001 \
        --workspace "$PIPERGO2_WS" \
        --gui \
        --driver-config "$PIPERGO2_DRIVER_CONFIG" \
        > "$WORKSPACE_BASE/logs/pipergo2.log" 2>&1 &
    PIPERGO2_PID=$!
    echo "      PID: $PIPERGO2_PID"

    echo "启动 Franka watchdog..."
    python hal/hal_watchdog.py \
        --driver franka_simulation \
        --robot-id franka_001 \
        --workspace "$FRANKA_WS" \
        --gui \
        --driver-config "$FRANKA_DRIVER_CONFIG" \
        > "$WORKSPACE_BASE/logs/franka.log" 2>&1 &
    FRANKA_PID=$!
    echo "      PID: $FRANKA_PID"

    echo "启动 G1 watchdog..."
    python hal/hal_watchdog.py \
        --driver g1_simulation \
        --robot-id g1_001 \
        --workspace "$G1_WS" \
        --gui \
        --driver-config "$G1_DRIVER_CONFIG" \
        > "$WORKSPACE_BASE/logs/g1.log" 2>&1 &
    G1_PID=$!
    echo "      PID: $G1_PID"

    echo ""
    echo "所有 watchdog 进程已启动."
    echo "PiperGo2: $PIPERGO2_PID"
    echo "Franka: $FRANKA_PID"
    echo "G1: $G1_PID"
    echo ""
    echo "日志文件: $WORKSPACE_BASE/logs/"
    echo ""
    echo "按 Ctrl+C 停止所有进程"
    echo ""

    # 等待信号
    trap "echo '停止所有进程...'; kill $PIPERGO2_PID $FRANKA_PID $G1_PID 2>/dev/null; exit" INT TERM

    # 监控进程
    while true; do
        sleep 1
        if ! kill -0 $PIPERGO2_PID 2>/dev/null; then
            echo "PiperGo2 watchdog 已退出"
        fi
        if ! kill -0 $FRANKA_PID 2>/dev/null; then
            echo "Franka watchdog 已退出"
        fi
        if ! kill -0 $G1_PID 2>/dev/null; then
            echo "G1 watchdog 已退出"
        fi
    done
else
    echo "已取消. 请手动启动各进程."
fi