import time

CENTER = 90
LEFT = 0
RIGHT = 180

def accelerate(current, target, rate=0.08):
    return current + (target - current) * rate

def steer(current, target, rate=0.4):
    return int(current + (target - current) * rate)

def control_loop(bot, get_command, is_running_func, interval=0.05, target_speed=0.6):
    current_speed = 0.0
    current_angle = 90  # CENTER

    while is_running_func():
        command = get_command()

        speed_target = 0.0
        steering_target = 90  # CENTER

        # == 1~6：正常行驶 ==
        if command == 1:  # FORWARD
            speed_target = target_speed
        elif command == 2:  # BACKWARD
            speed_target = -target_speed
        elif command == 3:  # FORWARD_LEFT
            speed_target = target_speed
            steering_target = 0
        elif command == 4:  # FORWARD_RIGHT
            speed_target = target_speed
            steering_target = 180
        elif command == 5:  # LEFT_BACK
            speed_target = -target_speed
            steering_target = 0
        elif command == 6:  # RIGHT_BACK
            speed_target = -target_speed
            steering_target = 180

        # == 0：刹车 ==
        elif command == 0:
            if abs(current_speed) > 0.05:
                print("[刹车] 快速反向停止")
                brake_force = -0.3 if current_speed > 0 else 0.3
                bot.set_car_motion(brake_force, 0, 0)
                time.sleep(0.15)
                bot.set_car_motion(0, 0, 0)
            current_speed = 0.0
            steering_target = 90  # 舵机回正

        # 正常加减速处理
        if command != 0:
            current_speed = accelerate(current_speed, speed_target)
            current_angle = steer(current_angle, steering_target)
            bot.set_car_motion(current_speed, 0, 0)
            bot.set_pwm_servo(1, current_angle)
        else:
            bot.set_pwm_servo(1, 90)  # command==0 时确保舵机回正

        time.sleep(interval)
