'''
2025/2/21
Author:cheese
最初版本的底层控制代码，非常简陋
以下是舵机引脚定义
PCA9685的1-8通道分别对应左前大腿，左前小腿，右前大腿，右前小腿，以及对应的后腿
关于左右侧舵机运动反向问题在IK节点解决了，说实话通过ros架构搭建是一件比较绕的事情
顺带一提，如果要使用I2C串口之类的外设，需要提前sudo su拿到root权限
初始调中方式是大腿垂直地面，小腿平行地面，舵机初始角度90，可能需要在此基础上微调来达成更好的步态效果

'''




import rospy
from std_msgs.msg import Float64MultiArray
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50


servos = []
for i in range(8):  # 一共有8个舵机，创建对应对象，这里使用sg90或者类似的9g舵机,如果使用其他舵机对应的脉冲频率范围需要自行查阅
    servos.append(servo.Servo(pca.channels[i], min_pulse=500, max_pulse=2400))

def callback(data):
    angles = data.data
    for i, angle in enumerate(angles):#enumerate的使用方式自行查找
        try:
            # 做一个角度范围限制，防止报错卡死
            angle = max(0, min(180, angle))
            servos[i].angle = angle
            rospy.loginfo(f"Set servo {i} to angle {angle} degrees")
        except ValueError as e:
            rospy.logerr(f"Error setting servo {i} angle: {e}")

def servo_control():
    rospy.init_node('servo_controller', anonymous=True)
    rospy.Subscriber('joint_angles', Float64MultiArray, callback)
    rospy.loginfo("runnning")
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        pca.deinit()