# 检查 ethtool 是否已安装
if ! dpkg -l | grep -q "ethtool"; then
    echo "\e[31m错误: 系统中未检测到 ethtool。\e[0m"
    echo "请使用以下命令安装 ethtool:"
    echo "sudo apt update && sudo apt install ethtool"
    exit 1
fi

# 检查 can-utils 是否已安装
if ! dpkg -l | grep -q "can-utils"; then
    echo "\e[31m错误: 系统中未检测到 can-utils。\e[0m"
    echo "请使用以下命令安装 can-utils:"
    echo "sudo apt update && sudo apt install can-utils"
    exit 1
fi

echo "ethtool 和 can-utils 均已安装。"

# 遍历所有 CAN 接口
for iface in $(ip -br link show type can | awk '{print $1}'); do
    # 使用 ethtool 获取 bus-info
    BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
    
    if [ -z "$BUS_INFO" ];then
        echo "错误: 无法获取接口 $iface 的 bus-info 信息。"
        continue
    fi
    
    echo "接口 $iface 插入在 USB 端口 $BUS_INFO"
done