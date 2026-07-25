#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
**********功能:小车前进5秒 -> 掉头180° -> 前进5秒返回*********
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

start = True
# 关闭前处理
def Stop(signum, frame):
    global start

    start = False
    print('关闭中...')
    chassis.set_velocity(0, 0, 0)  # 关闭所有电机


signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    # ============ 参数配置 ============
    FORWARD_SPEED = 50       # 前进速度, 单位: mm/s
    FORWARD_DIRECTION = 90   # 前进方向, 90度=前进
    FORWARD_TIME = 5.0       # 前进时间, 单位:秒
    TURN_RATE = 0.3          # 旋转角速度, 正值=顺时针旋转
    TURN_TIME = 3.0          # 旋转180度所需时间, 单位:秒 (需根据实际校准)

    # ============ 第一步: 前进5秒 ============
    print('>>> 第一步: 小车前进5秒 (速度:{}mm/s)'.format(FORWARD_SPEED))
    chassis.set_velocity(FORWARD_SPEED, FORWARD_DIRECTION, 0)
    time.sleep(FORWARD_TIME)

    # 短暂停止
    chassis.set_velocity(0, 0, 0)
    time.sleep(0.5)
    print('>>> 前进5秒完成, 停止')

    # ============ 第二步: 掉头180度 ============
    print('>>> 第二步: 掉头180度 (旋转中...)')
    chassis.set_velocity(0, FORWARD_DIRECTION, TURN_RATE)
    time.sleep(TURN_TIME)

    # 短暂停止
    chassis.set_velocity(0, 0, 0)
    time.sleep(0.5)
    print('>>> 掉头完成')

    # ============ 第三步: 前进5秒返回 ============
    print('>>> 第三步: 返回 (速度:{}mm/s, 前进5秒)'.format(FORWARD_SPEED))
    chassis.set_velocity(FORWARD_SPEED, FORWARD_DIRECTION, 0)
    time.sleep(FORWARD_TIME)

    # ============ 结束 ============
    chassis.set_velocity(0, 0, 0)  # 关闭所有电机
    print('>>> 已完成! 小车前进5秒 -> 掉头 -> 前进5秒返回')
