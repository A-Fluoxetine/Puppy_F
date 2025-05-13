'''
2025/2/21
Author:cheese
最初版本的逆运动学解算，非常简陋
以下是舵机引脚定义
PCA9685的1-8通道分别对应左前大腿，左前小腿，右前大腿，右前小腿，以及对应的后腿
注意这里大小腿的长度很大程度影响最后结果
建系计算方式是以一条腿大腿根为原点，y轴正方向垂直向下，x轴正方向水平向小腿初始反方向
具体逆运动学计算方式可以参考灯哥开源，虽然个人觉得可以更加详细呢
'''

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

# 分别是大小腿传动实际有效长度的定义
l1 = 80
l2 = 70

def inverse_kinematics(x, y):
    """
    逆运动学计算
    :param x: 传入的x坐标
    :param y: 传入的y坐标
    :return:返回对应的大小腿舵机角度theta1,theta2
    """
    # 计算 theta2
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    # 进行角度限制，防止arccos运算报错
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    theta2 = np.arccos(cos_theta2)

    # 计算辅助角 phi
    phi = np.arctan2(y, x)

    # 计算 theta1
    cos_alpha = (l1**2 + x**2 + y**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + y**2))
    # 进行角度限制，防止arccos运算报错
    cos_alpha = np.clip(cos_alpha, -1, 1)
    alpha = np.arccos(cos_alpha)
    theta1 = phi - alpha

    return theta1, theta2

def callback(data):
    x1, x2, x3, x4, y1, y2, y3, y4 = data.data
    angles = []
    for x, y in zip([-x1, -x2, -x3, -x4], [y1, y2, y3, y4]):
        theta1, theta2 = inverse_kinematics(x, y)
        # 弧度转角度
        theta1_deg = np.degrees(theta1)
        theta2_deg = np.degrees(theta2)
        angles.extend([theta1_deg, theta2_deg])

    # 针对舵机状态进行打包
    for i in range(len(angles)):
        angles[i] = int(np.floor(angles[i]))
        if i == 3 or i == 2 or i==7 or i==6:  
            angles[i] = angles[i]
        else:
            angles[i] = 180-angles[i]
        if (i % 2):
            angles[i] = 180-angles[i]

    pub = rospy.Publisher('joint_angles', Float64MultiArray, queue_size=10)
    msg = Float64MultiArray()
    msg.data = angles
    pub.publish(msg)
    rospy.loginfo("Published joint angles: %s", msg.data)

def inverse_kinematics_node():
    rospy.init_node('inverse_kinematics', anonymous=True)
    rospy.Subscriber('leg_endpoint_coordinates', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    inverse_kinematics_node()