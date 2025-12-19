from machine import Pin, PWM
import time
import struct
import _thread
import micropython
import bluetooth

# 假设你已经上传了这些库文件
from servo import Servo
from pid import PIDController
import ble_simple_peripheral

# 分配紧急缓冲区，防止中断报错
micropython.alloc_emergency_exception_buf(100)

# --- 1. 协议定义 ---

# Dabble 协议定义 (长包)
dabble = {
    "left": 0x04, "right": 0x08, "up": 0x01, "down": 0x02, "stop": 0x00
}

# Bluefruit 协议定义 (你收到的 5 字节数据)
# 根据你的数据：!B516(Up), !B615(Down), !B813(Left/Right?), !B714(Left/Right?)
# 注意：这里保留了你代码原本的映射 (Left用B8, Right用B7)，如果方向反了，请互换这两个的值
bluefruit = {
    "left": b'!B813', 
    "right": b'!B714', 
    "up": b'!B516', 
    "down": b'!B615'
}

# Bluefruit 的停止指令集合 (按键松开)
bluefruit_stop_list = [b'!B507', b'!B606', b'!B705', b'!B804']


# --- 2. 核心控制类 ---
class Aim():
    def __init__(self):
        # 这里的 yaw 和 pitch 代表“目标偏差值”（相对角度）
        # 0 代表静止/对准，非 0 代表需要运动
        self.yaw = 0
        self.pitch = 0

        self.yaw_current = 0
        self.pitch_current = 0

        self.yaw_input = 0
        self.pitch_input = 0

        # 定义手动控制时的“虚拟误差值”
        # 这个值越大，PID 算出的运动速度越快
        self.manual_offset = 15.0

        # PID 参数
        self.pid_yaw = PIDController(Kp=0.01177, Ki=0., Kd=0.2748)
        self.pid_pitch = PIDController(Kp=0.0090, Ki=0, Kd=0.285)

        # 舵机初始化 (请确保 Servo 库支持 limit 参数)
        self.servo_yaw = Servo(1, limit_min_angle=90-40, limit_max_angle=90+90)
        self.servo_pitch = Servo(2, limit_min_angle=90-18, limit_max_angle=90+90)

    def servo_move(self, callback):
        print("Servo Thread Started")
        while True:
            time.sleep(0.001)

            # --- PID 计算逻辑 ---
            # 直接使用 self.yaw (由蓝牙修改) 作为目标偏差输入

            # 1. PID 计算 Yaw
            self.yaw_input = self.pid_yaw.update(self.yaw, self.yaw_current)
            yaw_targe_angle_last = self.servo_yaw.targe_angle

            # 2. PID 计算 Pitch
            self.pitch_input = self.pid_pitch.update(self.pitch, self.pitch_current)
            pitch_targe_angle_last = self.servo_pitch.targe_angle

            # 3. 驱动舵机 (控制相对角度)
            self.servo_yaw.set_angle_relative(self.yaw_input)
            # 计算实际移动量作为下一次的 current 输入 (反馈)
            self.yaw_current = self.servo_yaw.targe_angle - yaw_targe_angle_last

            self.servo_pitch.set_angle_relative(self.pitch_input)
            self.pitch_current = self.servo_pitch.targe_angle - pitch_targe_angle_last

    def start(self):
        _thread.start_new_thread(self.servo_move, ("2",))


# --- 3. 全局对象初始化 ---
aim = Aim()


# --- 4. 蓝牙指令处理逻辑 (核心修复部分) ---
def handle_command(cmd):
    """
    主线程中处理指令，区分 Bluefruit(5字节) 和 Dabble(>=7字节)
    """
    global aim
    
    # 调试打印，如果不想要刷屏可以注释掉
    # print(f"CMD: {cmd}")

    # ==========================
    # CASE A: Bluefruit 协议 (5 Bytes)
    # ==========================
    if len(cmd) == 5:
        if cmd == bluefruit["left"]:
            aim.yaw = aim.manual_offset
            print("Action: Left (Bluefruit)")
            
        elif cmd == bluefruit["right"]:
            aim.yaw = -aim.manual_offset
            print("Action: Right (Bluefruit)")
            
        elif cmd == bluefruit["up"]:
            aim.pitch = aim.manual_offset
            print("Action: Up (Bluefruit)")
            
        elif cmd == bluefruit["down"]:
            aim.pitch = -aim.manual_offset
            print("Action: Down (Bluefruit)")
            
        elif cmd in bluefruit_stop_list:
            # 任意按键松开，停止运动
            aim.yaw = 0
            aim.pitch = 0
            print("Action: Stop (Bluefruit)")

    # ==========================
    # CASE B: Dabble 协议 (>= 7 Bytes)
    # ==========================
    elif len(cmd) >= 7:
        try:
            val = cmd[6] # 获取指令位
            
            if val == dabble["left"]:
                aim.yaw = aim.manual_offset
                print("Action: Left (Dabble)")
                
            elif val == dabble["right"]:
                aim.yaw = -aim.manual_offset
                print("Action: Right (Dabble)")
                
            elif val == dabble["up"]:
                aim.pitch = aim.manual_offset
                print("Action: Up (Dabble)")
                
            elif val == dabble["down"]:
                aim.pitch = -aim.manual_offset
                print("Action: Down (Dabble)")
                
            elif val == dabble["stop"]:
                aim.yaw = 0
                aim.pitch = 0
                print("Action: Stop (Dabble)")
                
        except IndexError:
            print("Error: Dabble packet index out of range")


# --- 5. 蓝牙接收中断回调 ---
def on_rx(data):
    """
    蓝牙接收到数据时触发。
    在这里只做简单的筛选，复杂的逻辑用 schedule 抛给主线程。
    """
    # 只要长度符合任意一种协议，就调度处理
    if len(data) >= 5:
        micropython.schedule(handle_command, data)


# --- 6. 主程序入口 ---
if __name__ == "__main__":
    # 初始化蓝牙
    ble = bluetooth.BLE()
    p = ble_simple_peripheral.BLESimplePeripheral(ble, name='Kie')
    p.on_write(on_rx)
    
    # 启动舵机控制线程
    time.sleep(1) # 等待蓝牙启动稳定
    aim.start()

    print("System Ready... Waiting for Bluetooth command.")

    # 主循环，保持主线程存活
    while True:
        time.sleep(1)