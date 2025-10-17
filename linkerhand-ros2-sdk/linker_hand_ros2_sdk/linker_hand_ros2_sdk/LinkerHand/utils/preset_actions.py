#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class PresetActions:
    def __init__(self, hand_api):
        """初始化预设动作控制器"""
        self.hand = hand_api
        # L7型号需要7个关节位置值
        # 位置范围：0-255（0=闭合，255=张开）
        self.actions = {
            # 完全张开
            'open': {
                'position': [255, 255, 255, 255, 255, 255, 255],
                'speed': [255, 255, 255, 255, 255, 255, 255]
            },
            # GUI调整的握拳姿态
            'close': {
                'position': [0, 255, 30, 30, 30, 30, 225],
                'speed': [255, 255, 255, 255, 255, 255, 255]
            },
            # 自然状态
            'grasp': {
                'position': [0, 180, 220, 215, 210, 205, 255],
                'speed': [255, 255, 255, 255, 255, 255, 255]
            }
        }
    
    def execute(self, action_name: str) -> bool:
        """执行预设动作"""
        print(f"[PresetActions] 执行动作: {action_name}")
        
        if action_name not in self.actions:
            print(f"[PresetActions] 未找到: {action_name}")
            print(f"[PresetActions] 可用的动作: {list(self.actions.keys())}")
            return False
            
        try:
            action = self.actions[action_name]
            pos = action['position']
            spd = action['speed']
            
            print(f"[PresetActions] 发送 pos={pos} spd={spd}")
            self.hand.set_speed(spd)
            self.hand.finger_move(pos)
            print(f"[PresetActions] 动作 {action_name} 执行完成")
            return True
        except Exception as e:
            print(f"[PresetActions] 动作失败: {e}")
            return False
    
    def get_available_actions(self) -> list:
        """获取所有可用的预设动作列表"""
        return list(self.actions.keys())