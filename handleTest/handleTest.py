import pygame
import time

def main():
    # 初始化pygame和手柄模块
    pygame.init()
    pygame.joystick.init()
    
    # 检查连接的手柄数量
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("没有检测到手柄。请连接手柄后重试。")
        return
    
    print(f"检测到 {joystick_count} 个手柄")
    
    # 初始化第一个检测到的手柄
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    # 获取手柄信息
    joystick_name = joystick.get_name()
    axes = joystick.get_numaxes()
    buttons = joystick.get_numbuttons()
    hats = joystick.get_numhats()
    
    print(f"手柄名称: {joystick_name}")
    print(f"轴数量: {axes}")
    print(f"按钮数量: {buttons}")
    print(f"方向键数量: {hats}")
    
    print("开始监测手柄输入，按Ctrl+C退出...")
    
    # 添加按钮状态跟踪
    previous_button_states = [0] * buttons
    
    try:
        # 主循环
        while True:
            # 处理pygame事件
            pygame.event.pump()
            
            # 检测按钮状态
            button_states = []
            button_release_states = []
            for i in range(buttons):
                button = joystick.get_button(i)
                if button:
                    button_states.append(f"按钮 {i} 被按下")
                # 检测释放事件 - 如果按钮之前是按下的而现在不是
                if previous_button_states[i] == 1 and button == 0:
                    button_release_states.append(f"按钮 {i} 被释放")
                # 更新先前状态
                previous_button_states[i] = button
            
            # 检测轴状态
            axis_states = []
            for i in range(axes):
                axis = joystick.get_axis(i)
                if abs(axis) > 0.1:  # 添加一个阈值，避免显示微小的波动
                    axis_states.append(f"轴 {i}: {axis:.2f}")
            
            # 检测方向键状态
            hat_states = []
            for i in range(hats):
                hat = joystick.get_hat(i)
                if hat != (0, 0):
                    hat_states.append(f"方向键 {i}: {hat}")
            
            # 清屏并打印当前状态
            if button_states or axis_states or hat_states or button_release_states:
                print("\n" + "-" * 40)
                if button_states:
                    print("按钮状态:", ", ".join(button_states))
                if button_release_states:
                    print("按钮释放:", ", ".join(button_release_states))
                if axis_states:
                    print("轴状态:", ", ".join(axis_states))
                if hat_states:
                    print("方向键状态:", ", ".join(hat_states))
            
            # 短暂休眠以减少CPU占用
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n程序已退出")
    finally:
        # 清理
        pygame.joystick.quit()
        pygame.quit()

if __name__ == "__main__":
    main()
