#!/bin/bash
OLD_REPO="/home/zyserver/work/PhyAgentOS"
NEW_REPO="/home/zyserver/work/PhyAgentOS-clean"
OLD_COMMIT="6176801"

# 进入新仓库
cd "$NEW_REPO"

# 1. 复制新增目录和文件
cp -r "$OLD_REPO/environment.yml" . 2>/dev/null
mkdir -p examples
cp "$OLD_REPO/examples"/{fleet_multi_robot_demo.json,franka_research3.driver.json,franka_simulation_driver.json,g1_simulation_driver.json,merom_scene_baked.usd,multi_robot_simulation_internutopia_driver.json,pipergo2_franka_same_scene_internutopia_config.json,single_driver_three_robot_full_config.json,start_fleet_multi_robot.sh,three_robot_same_scene_internutopia_driver.json} examples/ 2>/dev/null

cp -r "$OLD_REPO/hal/drivers/franka_backends" hal/drivers/
cp "$OLD_REPO/hal/drivers"/{franka_driver.py,franka_multi_backend_driver.py,franka_simulation_driver.py,g1_simulation_driver.py,multi_robot_simulation_driver.py,multi_robot_unified_isaac_driver.py} hal/drivers/ 2>/dev/null

cp "$OLD_REPO/hal/profiles"/{franka_research3.md,franka_simulation.md,g1_simulation.md,multi_robot_unified_isaac.md} hal/profiles/ 2>/dev/null

mkdir -p hal/simulation
cp "$OLD_REPO/hal/simulation/isaac_scene_bootstrap.py" hal/simulation/ 2>/dev/null

cp "$OLD_REPO/tests/test_embodied_start_bypass.py" tests/ 2>/dev/null

mkdir -p PhyAgentOS/templates
cp "$OLD_REPO/PhyAgentOS/templates/ACTION.md" PhyAgentOS/templates/ 2>/dev/null

# 2. 覆盖修改过的文件（从旧提交中提取干净版本）
FILES=(
    "bridge/src/index.ts"
    "examples/pipergo2_manipulation_driver.json"
    "hal/drivers/__init__.py"
    "hal/drivers/pipergo2_manipulation_driver.py"
    "PhyAgentOS/agent/context.py"
    "PhyAgentOS/agent/loop.py"
    "PhyAgentOS/agent/tools/embodied.py"
    "PhyAgentOS/channels/telegram.py"
    "PhyAgentOS/__init__.py"
    "PhyAgentOS/skills/README.md"
    "PhyAgentOS/templates/configs/pipergo2_manipulation_driver.json"
    "PhyAgentOS/templates/SOUL.md"
    "PhyAgentOS/templates/TOOLS.md"
    "PhyAgentOS/utils/helpers.py"
    "pyproject.toml"
    ".gitignore"
    "README.md"
    "README_zh.md"
)

for file in "${FILES[@]}"; do
    git --git-dir="$OLD_REPO/.git" show "$OLD_COMMIT:$file" > "$file"
done

echo "所有文件已复制/覆盖。请检查后执行 git add 和 git commit。"
