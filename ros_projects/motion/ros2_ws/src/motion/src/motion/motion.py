from pynput import keyboard
from pynput.keyboard import Key
from Rosmaster_Lib import Rosmaster
import time
import math
import threading

# 舵机角度
CENTER = 90
LEFT = 0
RIGHT = 180

# 全局状态
keys_pressed = set()
current_angle = CENTER
current_speed = 0.0
target_speed = 0.6
steering_target = CENTER
is_braking = False  # 是否正在刹车
is_damping = False  # 是否增加减速阻尼

# 参数
speed_step = 0.02
interval = 0.05  # 主循环周期

bot = Rosmaster()
bot.create_receive_threading()

# 非线性速度增长（指数形状）
def accelerate(current, target, rate=0.08):
    diff = target - current
    return current + diff * rate

# 非线性转向（指数形状）
def steer(current, target, rate=0.4):
    diff = target - current
    return int(current + diff * rate)

# 实时状态更新循环（非阻塞）
def control_loop():
    global current_speed, current_angle, is_braking

    prev_keys = set()

    while True:
        if 'quit' in keys_pressed:
            break

        # 空格键刹车优先级最高
        if is_braking and abs(current_speed) > 0.05:
            print("[空格刹车] 快速反向停止！")
            brake_force = -0.3 if current_speed > 0 else 0.3
            bot.set_car_motion(brake_force, 0, 0)
            time.sleep(0.15)
            bot.set_car_motion(0, 0, 0)
            current_speed = 0.0
            continue

        # 目标速度判断
        speed_target = 0.0
        steering_target = CENTER

        # 判断是否需要启动阻尼
        if not any(k in keys_pressed for k in ['w', 's']) and abs(current_speed) > 0.05:
            is_damping = True
        else:
            is_damping = False

        # ---------- 转向独立控制 ----------
        if 'a' in keys_pressed:
            steering_target = LEFT
        elif 'd' in keys_pressed:
            steering_target = RIGHT
        else:
            steering_target = CENTER

        # ---------- 速度控制 ----------
        if 'w' in keys_pressed:
            speed_target = target_speed
        elif 's' in keys_pressed:
            speed_target = -target_speed
        else:
            speed_target = 0.0  # 可触发阻尼

        # ---------- 阻尼减速逻辑 ----------
        if is_damping:
            damping_rate = 0.2  # 每次减速比例
            current_speed *= (1 - damping_rate)
            if abs(current_speed) < 0.01:
                current_speed = 0.0
            bot.set_car_motion(current_speed, 0, 0)


        # 平滑速度和转向
        current_speed = accelerate(current_speed, speed_target)
        bot.set_car_motion(current_speed, 0, 0)

        current_angle = steer(current_angle, steering_target)
        bot.set_pwm_servo(1, current_angle)

        prev_keys = set(keys_pressed)
        time.sleep(interval)


# 键盘监听
def on_press(key):
    global is_braking
    try:
        if key.char in ['w', 'a', 'd', 's']:
            keys_pressed.add(key.char)
    except AttributeError:
        if key == Key.space:
            is_braking = True  # 按下空格，开始刹车

def on_release(key):
    global is_braking
    try:
        if key.char in ['w', 'a', 'd', 's']:
            keys_pressed.discard(key.char)
    except AttributeError:
        if key == Key.space:
            is_braking = False  # 松开空格，停止刹车
    if key == keyboard.Key.esc:
        keys_pressed.add('quit')
        return False

# 启动控制线程
t = threading.Thread(target=control_loop)
t.start()

print("控制说明：按住 W 持续加速，A/D 同时转向，S 后退，ESC 退出")
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# 停止小车
bot.set_car_motion(0, 0, 0)
bot.set_pwm_servo(1, CENTER)
del bot
print("程序退出，资源已释放。")
