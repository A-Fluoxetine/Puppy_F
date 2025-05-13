'''
2025/2/21
Author:cheese
最初版本的小跑步态，非常简陋
以下是舵机引脚定义
PCA9685的1-8通道分别对应左前大腿，左前小腿，右前大腿，右前小腿，以及对应的后腿
小跑步态，就是对角位置的腿完全一致，13支撑相24摆动相
刚开始可以把单个周期放大，比较直观

'''

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import math

# 小跑步态的实现，较为简略
Ts = 0.4  # 单个周期，这里trot步态
h = 15.0  # ̧抬腿高度，这个建议不要超过40
d = 30.0  # 每一个周期的行进距离
leg_scale = [1,1,1,1]  # 转弯系数，通过调整这个参数来实现对应的转弯效果
standing_height = 50.0  # 站立高度

def cycloid_trajectory(t):
    """
    摆线足端路径规划
    :param t: 计时器
    :return: 对应的xy坐标
    """
    t = t % Ts  # 这里是为了限制计时器不要超出周期长度
    theta = 2 * math.pi * (t / Ts)
    if 0 <= theta <= math.pi:
        x = -d / 2 + d * theta / (2 * math.pi)
        y = h * (1 - math.cos(theta)) / 2 + standing_height
    else:
        x = d / 2 - d * (theta - math.pi) / (2 * math.pi)
        y = standing_height
    # ����һλС��
    x = round(x, 1)
    y = round(y, 1)
    return -x, y

def trajectory_generation():
    rospy.init_node('trajectory_generator', anonymous=True)
    pub = rospy.Publisher('leg_endpoint_coordinates', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(20)  # 系统频率设定为20HZ换算成秒就是周期1/20=0.05s

    t = 0
    time_step = 0.1  # 刷新率0.05s，这个不建议调整太高，会导致PCA9685阻塞
    while not rospy.is_shutdown():
        # 主程序
        t1 = t
        t2 = (t + Ts / 2) % Ts  # 让另外两条腿产生相位差

        # 计算坐标
        x1, y1 = cycloid_trajectory(t1)
        x2, y2 = cycloid_trajectory(t2)

        # 设置转弯
        x1 *= leg_scale[0]
        x2 *= leg_scale[1]

        # 打包数据，发布话题
        msg = Float64MultiArray()
        msg.data = [x1, x2, x2, x1, y1, y2, y2, y1]

        pub.publish(msg)
        rospy.loginfo("Published leg endpoint coordinates: %s", msg.data)

        t += time_step  # 计时器递增
        rate.sleep()

if __name__ == '__main__':
    try:
        trajectory_generation()
    except rospy.ROSInterruptException:
        pass