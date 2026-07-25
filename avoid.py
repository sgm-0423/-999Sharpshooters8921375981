#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import numpy as np
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
*****功能:慢速前进 -> 避障绕行 -> 前进2秒 -> 掉头 -> 再避障 -> 前进2秒 -> 停下*****
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

chassis = mecanum.MecanumChassis()
hwsonar = Sonar.Sonar()

start = True

def Stop(signum, frame):
    global start
    start = False
    print('关闭中...')
    chassis.set_velocity(0, 0, 0)

signal.signal(signal.SIGINT, Stop)

# ============ 参数配置 ============
SPEED = 30              # 前进速度, mm/s (较慢)
FORWARD_DIR = 90        # 前进方向, 90度=前进
THRESHOLD = 30.0        # 避障触发距离, cm
AVOID_TURN_TIME = 0.8   # 绕开时转向时间, 秒
AVOID_FORWARD_TIME = 1.5 # 绕开后前冲时间, 秒
TURN_AROUND_TIME = 2.0  # 掉头180度时间, 秒
FORWARD_AFTER_AVOID = 2.0  # 避障后继续前进时间, 秒


def get_filtered_distance(sonar, samples=5):
    """读取超声波距离,取多次滤波均值"""
    distances = []
    for _ in range(samples):
        dist = sonar.getDistance() / 10.0  # mm -> cm
        distances.append(dist)
        time.sleep(0.01)
    return float(np.mean(distances))


def avoid_obstacle():
    """绕开障碍物: 右转避开 -> 前进一段 -> 左转回到原方向"""
    print('    >>> 检测到障碍物, 开始绕行...')

    # 1. 右转约90度避开障碍物
    print('    >>> 右转避开...')
    chassis.set_velocity(0, FORWARD_DIR, 0.5)   # 顺时针旋转
    time.sleep(AVOID_TURN_TIME)

    # 2. 前进绕过障碍物
    print('    >>> 前进绕过...')
    chassis.set_velocity(SPEED, FORWARD_DIR, 0)
    time.sleep(AVOID_FORWARD_TIME)

    # 3. 左转回到原前进方向
    print('    >>> 左转回原方向...')
    chassis.set_velocity(0, FORWARD_DIR, -0.5)  # 逆时针旋转
    time.sleep(AVOID_TURN_TIME)

    print('    >>> 绕行完成')


def wait_for_obstacle():
    """前进并等待检测到障碍物, 返回True表示检测到, False表示被中断"""
    global start
    chassis.set_velocity(SPEED, FORWARD_DIR, 0)
    while start:
        dist = get_filtered_distance(hwsonar)
        print('    距离: {:.1f} cm'.format(dist))
        if dist <= THRESHOLD:
            return True
        time.sleep(0.05)
    return False


if __name__ == '__main__':
    print('>>> 开始任务: 慢速前进, 等待障碍物...')

    # ===== 第一段: 前进直到遇到障碍物 =====
    if not wait_for_obstacle():
        chassis.set_velocity(0, 0, 0)
        print('已中断')
        sys.exit(0)

    # ===== 绕开障碍物 =====
    avoid_obstacle()

    # ===== 绕开后继续前进2秒 =====
    print('>>> 绕开后前进{}秒...'.format(FORWARD_AFTER_AVOID))
    chassis.set_velocity(SPEED, FORWARD_DIR, 0)
    time.sleep(FORWARD_AFTER_AVOID)
    print('>>> 前进{}秒完成'.format(FORWARD_AFTER_AVOID))

    # 短暂停止
    chassis.set_velocity(0, 0, 0)
    time.sleep(0.5)

    # ===== 掉头180度 =====
    print('>>> 掉头180度...')
    chassis.set_velocity(0, FORWARD_DIR, 0.3)  # 顺时针旋转
    time.sleep(TURN_AROUND_TIME)
    chassis.set_velocity(0, 0, 0)
    time.sleep(0.5)
    print('>>> 掉头完成')

    # ===== 返回途中, 前进等待再次遇到障碍物 =====
    print('>>> 返回途中, 等待再次遇到障碍物...')
    if not wait_for_obstacle():
        chassis.set_velocity(0, 0, 0)
        print('已中断')
        sys.exit(0)

    # ===== 再次绕开障碍物 =====
    avoid_obstacle()

    # ===== 绕开后继续前进2秒 =====
    print('>>> 绕开后前进{}秒...'.format(FORWARD_AFTER_AVOID))
    chassis.set_velocity(SPEED, FORWARD_DIR, 0)
    time.sleep(FORWARD_AFTER_AVOID)

    # ===== 结束 =====
    chassis.set_velocity(0, 0, 0)
    print('>>> 任务完成! 小车已停下')
