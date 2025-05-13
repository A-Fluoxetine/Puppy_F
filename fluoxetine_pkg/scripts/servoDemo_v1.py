#!/usr/bin/env python3
# coding=utf-8
# 仔细想想还是在这里补充一点注释吧
# 这是一个混入了简单ros框架的舵机控制程序示例，环境配置：树莓派4B + Ubuntu 20.04 + ros noetic
'''
首先是舵机的控制，如果使用RPI.GPIO会导致抖动不止，这肯定不是我们想要的效果。
所以这里使用了PCA9685+IIC的方式来驱动舵机，这也是更加符合实际的做法。
PCA9685是一款16位的IIC舵机驱动板，用法简单，外部供电解决主控供电不足的大问题
在ubuntu 的某些版本下，开启IIC 并没有那么容易，配置可以查看我的B站专栏，会比较详细
'''
# import rospy
# from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = board.I2C()  # uses board.SCL and board.SDA，这里是第3，5号可以使用pinout显示
pca = PCA9685(i2c)
pca.frequency = 50
'''
这里需要注意这个程序是用于测试微型舵机（180）比如sg90,mg90s这一类。
不同舵机的占空比计算方式也有所不同，可以查看前面通过pwm直接驱动的代码
其它舵机的控制方式我放在最下面了
在舵机控制中，通常有两种方式来指定舵机的位置：
角度控制：通过直接设置舵机的角度，例如 servo7.angle = i，这里的 i 是一个具体的角度值，范围通常是 0 到 180 度。
分数控制：通过设置舵机位置的分数，范围是 0.0 到 1.0。其中，0.0 表示舵机的最小角度位置，1.0 表示舵机的最大角度位置。
这里我把分数控制的也注释在下面啦
'''

servo1 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400)
servo2 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2400)
servo3 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2400)
servo4 = servo.Servo(pca.channels[3], min_pulse=500, max_pulse=2400)
servo5 = servo.Servo(pca.channels[4], min_pulse=500, max_pulse=2400)

servo6 = servo.Servo(pca.channels[5], min_pulse=500, max_pulse=2400)
servo7 = servo.Servo(pca.channels[6], min_pulse=500, max_pulse=2400)
servo8 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)
# 这个是sg90对应的参数

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(11,GPIO.OUT)
# GPIO.setup(18,GPIO.OUT)

# arduino后遗症

def setup():
    pass
    # rospy.init_node("servoDrive")
    # rospy.logwarn("initialized")
    # pub = rospy.Publisher("cheese",String,queue_size = 10)
    #这个后续需要修改
    # rate = rospy.Rate(10)
    # 默认10HZ

def loop():
    while 1:
        # rospy.loginfo("running!")
        servo1.angle=60
        servo2.angle=90
        servo3.angle=90
        servo5.angle=60
        servo4.angle=90
        servo6.angle=90
        servo7.angle=90
        servo8.angle=90

def destroy():
    GPIO.cleanup() 
    pca.deinit()
    # 不要忘记程序终止的操作嗷！


if __name__ == "__main__":
    setup()
    loop()
    destroy()
    

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)

'''
# 舵机的分数控制方式，虽然非常少见
fraction = 0.0
while fraction < 1.0:
    servo7.fraction = fraction
    fraction += 0.01
    time.sleep(0.03)

pca.deinit()
'''