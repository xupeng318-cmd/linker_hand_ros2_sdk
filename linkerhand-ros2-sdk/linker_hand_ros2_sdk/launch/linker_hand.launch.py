#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk',
            output='screen',
            parameters=[{
                'hand_type': 'right', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "L7", # O6\L6P\L6\L7\L10\L20\L21 字母为大写
                'is_touch': True,
                'can': 'can1', # 这里需要修改为实际的CAN总线名称 如果是win系统则 PCAN_USBBUS1
            }],
        ),
    ])
