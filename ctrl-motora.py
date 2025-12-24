import can
import struct
import time

# ==================== 配置参数 ====================
MOTOR_IDS = [1, 2, 3]         # 电机ID列表
CAN_BITRATE = 1000000         # 1Mbps
# =================================================

class MotorController:
    def __init__(self):
        self.bus = self.init_pcan()
        self.current_idx = 0
        self.motor_id = MOTOR_IDS[self.current_idx]
        
        # 缓存按键2设定的参数
        self.set_angle = 0.0     # 单位: 度 (float)
        self.set_dir = 0x00      # 0x00顺时针, 0x01逆时针
        self.set_speed = 30     # 单位: dps
        
    def init_pcan(self):
        try:
            bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=CAN_BITRATE)
            print(f"✓ PCAN初始化成功 [{CAN_BITRATE//1000}kbps]")
            return bus
        except Exception as e:
            print(f"✗ 初始化失败: {e}")
            return None

    def send_cmd(self, data, timeout=0.2):
        msg = can.Message(arbitration_id=0x140 + self.motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return self.bus.recv(timeout)
        except Exception as e:
            print(f"发送异常: {e}")
            return None

    # --- 功能 1: 切换电机ID ---
    def switch_id(self):
        self.current_idx = (self.current_idx + 1) % len(MOTOR_IDS)
        self.motor_id = MOTOR_IDS[self.current_idx]
        print(f"当前操控电机 ID: {self.motor_id}")

    # --- 功能 2: 设定参数 ---
    def set_params(self):
        try:
            self.set_angle = float(input("输入角度 (0-360.0): "))
            self.set_dir = int(input("输入方向 (0:顺时针 / 1:逆时针): "))
            self.set_speed = int(input("输入速度限制 (dps): "))
            print(f"✓ 参数已设: {self.set_angle}°, {'逆' if self.set_dir else '顺'}, {self.set_speed}dps")
        except ValueError:
            print("输入非法，请重新输入数字")

    # --- 功能 3: 单圈位置闭环控制2 (0xA6) ---
    def control_single_circle(self):
        # 转换物理量: 0.01degree/LSB
        angle_val = int(self.set_angle * 100)
        data = [0xA6, self.set_dir]
        data += list(struct.pack('<H', self.set_speed))  # 速度限制 (2字节)
        data += list(struct.pack('<I', angle_val))      # 位置控制 (4字节)
        print(f"执行单圈控制: {self.set_angle}°")
        self.send_cmd(data)

    # --- 功能 4: 设置当前位置为任意角度 (0x95) ---
    def set_zero_position(self):
        try:
            angle = int(float(input("设置当前点为多少度? (0.01°/LSB): ")) * 100)
            data = [0x95, 0x00, 0x00, 0x00] + list(struct.pack('<i', angle))
            reply = self.send_cmd(data)
            if reply: print("✓ 位置设置成功")
        except: print("设置失败")

    # --- 功能 5: 增量位置闭环控制2 (0xA8) ---
    def control_incremental(self):
        # 符号位决定方向
        inc_val = int(self.set_angle * 100)
        if self.set_dir == 1: inc_val = -inc_val
        
        data = [0xA8, 0x00]
        data += list(struct.pack('<H', self.set_speed)) # 这里的协议描述是uint32，但通常dps为uint16，此处按协议描述4字节偏移或2字节补位，通常2字节足够
        data += [0, 0] # 补足速度段的4字节
        data = [0xA8, 0x00] + list(struct.pack('<H', self.set_speed)) + list(struct.pack('<i', inc_val))
        # 修正: 0xA8 DATA[2-3]是速度, DATA[4-7]是增量
        print(f"执行增量控制: {inc_val/100.0}°")
        self.send_cmd(data)

    # --- 功能 6: 打印现在电机角度 (0x92 & 0x94) ---
    def print_current_angles(self):
        # 读取多圈
        rep92 = self.send_cmd([0x92, 0,0,0,0,0,0,0])
        if rep92:
            angle = struct.unpack('<q', rep92.data[1:8] + b'\x00')[0] # 7字节转int64
            print(f"多圈角度: {angle * 0.01:.2f}°")
        # 读取单圈
        rep94 = self.send_cmd([0x94, 0,0,0,0,0,0,0])
        if rep94:
            angle = struct.unpack('<I', rep94.data[4:8])[0]
            print(f"单圈角度: {angle * 0.01:.2f}°")

    # --- 功能 7: 打印电机状态1/2 ---
    def print_all_status(self):
        # 状态1 (0x9A)
        rep1 = self.send_cmd([0x9A, 0,0,0,0,0,0,0])
        if rep1:
            print(f"--- 状态1 --- 温度: {rep1.data[1]}°C, 电压: {struct.unpack('<h', rep1.data[2:4])[0]*0.01:.2f}V")
        # 状态2 (0x9C)
        rep2 = self.send_cmd([0x9C, 0,0,0,0,0,0,0])
        if rep2:
            speed = struct.unpack('<h', rep2.data[4:6])[0]
            print(f"--- 状态2 --- 转速: {speed} dps")

    # --- 功能 9: 切换电机电源 ---
    def toggle_power(self):
        # 先读状态判断开关
        rep = self.send_cmd([0x9A, 0,0,0,0,0,0,0])
        if rep:
            is_on = (rep.data[6] == 0x00)
            cmd = 0x80 if is_on else 0x88 # 0x80关闭，0x88运行
            self.send_cmd([cmd, 0,0,0,0,0,0,0])
            print(f"电机电源已 {'关闭' if is_on else '开启'}")


# ========== 主程序 ==========

def main():
    ctrl = MotorController()
    if not ctrl.bus: return
    
    ctrl.print_all_status()

    menu = """
    [1] 切换ID     [2] 设定参数 [3] 单圈闭环  
    [4] 设任意位置  [5] 增量控制 [6] 读当前角  
    [7] 读状态1&2  [9] 电源开关  
    [q] 退出
    """
    
    while True:
        print(menu)
        key = input(f"ID:{ctrl.motor_id} >>> ").strip().lower()
        if key == '1': ctrl.switch_id()
        elif key == '2': ctrl.set_params()
        elif key == '3': ctrl.control_single_circle()
        elif key == '4': ctrl.set_zero_position()
        elif key == '5': ctrl.control_incremental()
        elif key == '6': ctrl.print_current_angles()
        elif key == '7': ctrl.print_all_status()
        elif key == '9': ctrl.toggle_power()
        elif key == 'q': break

if __name__ == "__main__":
    main()