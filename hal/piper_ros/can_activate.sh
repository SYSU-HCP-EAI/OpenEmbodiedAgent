#!/bin/bash

# 默认的 CAN 名称，用户可以通过命令行参数设定
DEFAULT_CAN_NAME="${1:-can0}"

# 单个 CAN 模块时的默认比特率，用户可以通过命令行参数设定
DEFAULT_BITRATE="${2:-1000000}"

# USB 硬件地址（可选参数）
USB_ADDRESS="${3}"
echo "-------------------START-----------------------"
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

# 获取当前系统中的 CAN 模块数量
CURRENT_CAN_COUNT=$(ip link show type can | grep -c "link/can")

# 检查当前系统中的 CAN 模块数量是否符合预期
if [ "$CURRENT_CAN_COUNT" -ne "1" ]; then
    if [ -z "$USB_ADDRESS" ]; then
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
        echo -e " \e[31m 错误: 系统检测到的 CAN 模块数量 ($CURRENT_CAN_COUNT) 与预期数量 (1) 不符。\e[0m"
        echo -e " \e[31m 请增加usb硬件地址参数，如: \e[0m"
        echo -e " bash can_activate.sh can0 1000000 1-2:1.0"
        echo "-------------------ERROR-----------------------"
        exit 1
    fi
fi

# 加载 gs_usb 模块
# sudo modprobe gs_usb
# if [ $? -ne 0 ]; then
#     echo "错误: 无法加载 gs_usb 模块。"
#     exit 1
# fi

if [ -n "$USB_ADDRESS" ]; then
    echo "检测到 USB 硬件地址参数: $USB_ADDRESS"
    
    # 使用 ethtool 查找与 USB 硬件地址对应的 CAN 接口
    INTERFACE_NAME=""
    for iface in $(ip -br link show type can | awk '{print $1}'); do
        BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
        if [ "$BUS_INFO" == "$USB_ADDRESS" ]; then
            INTERFACE_NAME="$iface"
            break
        fi
    done
    
    if [ -z "$INTERFACE_NAME" ]; then
        echo "错误: 无法找到与 USB 硬件地址 $USB_ADDRESS 对应的 CAN 接口。"
        exit 1
    else
        echo "找到与 USB 硬件地址 $USB_ADDRESS 对应的接口: $INTERFACE_NAME"
    fi
else
    # 获取唯一的 CAN 接口
    INTERFACE_NAME=$(ip -br link show type can | awk '{print $1}')
    
    # 检查是否获取到了接口名称
    if [ -z "$INTERFACE_NAME" ]; then
        echo "错误: 无法检测到 CAN 接口。"
        exit 1
    fi
    BUS_INFO=$(sudo ethtool -i "$INTERFACE_NAME" | grep "bus-info" | awk '{print $2}')
    echo "预期配置单个can模块，检测到接口 $INTERFACE_NAME，对应的usb地址为 $BUS_INFO"
fi

# 检查当前接口是否已经激活
IS_LINK_UP=$(ip link show "$INTERFACE_NAME" | grep -q "UP" && echo "yes" || echo "no")

# 获取当前接口的比特率
CURRENT_BITRATE=$(ip -details link show "$INTERFACE_NAME" | grep -oP 'bitrate \K\d+')

if [ "$IS_LINK_UP" == "yes" ] && [ "$CURRENT_BITRATE" -eq "$DEFAULT_BITRATE" ]; then
    echo "接口 $INTERFACE_NAME 已经激活，并且比特率为 $DEFAULT_BITRATE"
    
    # 检查接口名称是否与默认的名称匹配
    if [ "$INTERFACE_NAME" != "$DEFAULT_CAN_NAME" ]; then
        echo "将接口 $INTERFACE_NAME 重命名为 $DEFAULT_CAN_NAME"
        sudo ip link set "$INTERFACE_NAME" down
        sudo ip link set "$INTERFACE_NAME" name "$DEFAULT_CAN_NAME"
        sudo ip link set "$DEFAULT_CAN_NAME" up
        echo "接口已重命名为 $DEFAULT_CAN_NAME，并重新激活。"
    else
        echo "接口名称已经是 $DEFAULT_CAN_NAME"
    fi
else
    # 如果接口未激活或比特率不同，进行设置
    if [ "$IS_LINK_UP" == "yes" ]; then
        echo "接口 $INTERFACE_NAME 已经激活，但比特率为 $CURRENT_BITRATE，与设定的 $DEFAULT_BITRATE 不符。"
    else
        echo "接口 $INTERFACE_NAME 未激活或未设置比特率。"
    fi
    
    # 设置接口比特率并激活
    sudo ip link set "$INTERFACE_NAME" down
    sudo ip link set "$INTERFACE_NAME" type can bitrate $DEFAULT_BITRATE
    sudo ip link set "$INTERFACE_NAME" up
    echo "接口 $INTERFACE_NAME 已重新设置为比特率 $DEFAULT_BITRATE 并激活。"
    
    # 重命名接口为默认名称
    if [ "$INTERFACE_NAME" != "$DEFAULT_CAN_NAME" ]; then
        echo "将接口 $INTERFACE_NAME 重命名为 $DEFAULT_CAN_NAME"
        sudo ip link set "$INTERFACE_NAME" down
        sudo ip link set "$INTERFACE_NAME" name "$DEFAULT_CAN_NAME"
        sudo ip link set "$DEFAULT_CAN_NAME" up
        echo "接口已重命名为 $DEFAULT_CAN_NAME，并重新激活。"
    fi
fi

echo "-------------------OVER------------------------"
